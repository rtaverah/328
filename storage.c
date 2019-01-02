
/*
	NOTE:
		The 'store to eeprom' functions has been totally rewritten.
		All the work are now done outside the interrupt routine
		as it should be.
*/


#include "storage.h"
#include "global.h"
#include "ve.h"

#include <inttypes.h>
#include <avr/eeprom.h>
#include <avr/interrupt.h>

extern volatile struct squirt_t inj_port1, inj_port2;

//uint8_t WWURANGE[10] = {0, 20, 40, 60, 80, 100, 120, 140, 170, 200};
//uint8_t tpsdotrate[4] = {5, 20, 40, 77};

struct config_t config;
struct config_t config_ee SEEPROM;

struct ve_t ve;
struct ve_t ve_ee SEEPROM;

#ifdef RV8MT
const uint8_t code_ver = 121;
#else
const uint8_t code_ver = 20;
#endif

void init_stuff(void);

// this function has to be extracted from 'load_variables()',
// else gcc won't initialize the memory correct.
void init_stuff(void) {
	/*
		the squirt-datastructure contains its own pwm constants,
		MegaTune supports _one_ shared set of constants. Copy that set
		to both data structures until configuration program is done.
	*/
	
	inj_port1.pwm_delay = config.injpwmt;
	inj_port1.pwm_dc = config.injpwm;

	inj_port2.pwm_delay = config.injpwmt;
	inj_port2.pwm_dc = config.injpwm;

	// NOTE:
	// All initializations below should be removed. They are currently
	// working as a bad replacement for a configuration program

	config.fuelcut_thres = 0x0F;  // fuelcut until 1500 rpm
	config.cranking_thres = 3; // less+equal than 300 rpm

	config.primep_warm = config.primep;  // primepulse when warm
	config.primep_cold = config.primep;  // primepulse when cold

	config.tps_low = 0; //12;  // lowest possible tps adc-reading
	config.tps_high = 255; //220;  // highest possible tps adc-reading

	config.wwurange[0] = 0;
	config.wwurange[1] = 20;
	config.wwurange[2] = 40;
	config.wwurange[3] = 60;
	config.wwurange[4] = 80;
	config.wwurange[5] = 100;
	config.wwurange[6] = 120;
	config.wwurange[7] = 140;
	config.wwurange[8] = 170;
	config.wwurange[9] = 200;

	config.tpsdotrate[0] = 5;
	config.tpsdotrate[1] = 20;
	config.tpsdotrate[2] = 40;
	config.tpsdotrate[3] = 77;

	config.fan_temp = 234;  // temperature to activate coolant fan
	config.fan_hyst = 5;  // coolant fan will turn off fan_hyst degress below fan_temp

	// these are test values for my iac, remove the lines some day
	//  config.iac_step_seq = 0xD8;
	//  config.iac_conf = 0x60 | _BV(power_off_iac) | _BV(disable_iac);
	//  config.iac_conf = 0x60;
	/*
	config.iac_idlespeed[0] = 90;  // this is really cold
	config.iac_idlespeed[1] = 90;
	config.iac_idlespeed[2] = 90;
	config.iac_idlespeed[3] = 90;
	config.iac_idlespeed[4] = 90;
	config.iac_idlespeed[5] = 90;
	config.iac_idlespeed[6] = 90;
	config.iac_idlespeed[7] = 90;
	config.iac_idlespeed[8] = 90;
	config.iac_idlespeed[9] = 78;
	*/
/*
	config.iac_idle_offset[0] = 80;
	config.iac_idle_offset[1] = 80;
	config.iac_idle_offset[2] = 70;
	config.iac_idle_offset[3] = 70;
	config.iac_idle_offset[4] = 60;
	config.iac_idle_offset[5] = 60;
	config.iac_idle_offset[6] = 50;
	config.iac_idle_offset[7] = 45;
	config.iac_idle_offset[8] = 37;
	config.iac_idle_offset[9] = 25;
*/
	//  config.iac_sync = 120;

	//  config.iac_deadband = 1; // +/- 10 rpm deadband

	//  config.iac_warmup = 40;   // reclaim the counts in n seconds
	//  config.iac_start = 20;    // how many counts extra to open at cranking / rpm extra

	//  config.iac_tps_thres = 13;  // tps threshold for PID ~ throttle follower
	//  config.iac_rpm_thres = 50;  // PID control only below rpm (rpm = thres + idlespeed) 

	//  config.iac_integral_limit = 64;  // limit for integral control
	//  config.iac_max_tps_open = 30;   // upper limit (count) for how much the throttle follower can open the iac
	//  config.iac_reg_limit = 110;

	//  config.iac_kp = 0x60;
	// config.iac_kd = 0x00;
	//  config.iac_ki = 0x10;

	//  config.iac_pid_conf = _BV(pid_assymetric); // | _BV(tf_disable);
	//  config.iac_pid_conf = _BV(tf_disable);

	config.baro = 100;  // mean barometric reading
	config.dbaro = 20;  // max deviation from mean reading

	// This must ALLWAYS be cleared else unpredicatable results can happend!
	config.master_debug = 0x00;
}

/***************************************************************************/
/*** load config                                                         ***/
/***************************************************************************/
void load_variables(void) {

	/* read configuration variables from eeprom to sram */
	eeprom_read_block(&config, &config_ee, sizeof(struct config_t));
	eeprom_read_block(&ve, &ve_ee, sizeof(struct ve_t));
	init_stuff();
}



/***************************************************************************/
/*** init storage                                                        ***/
/***************************************************************************/
void init_storage(void) {
	eeprom_flags = 0;
}


/***************************************************************************/
/*** save variables                                                      ***/
/***************************************************************************/
// call this function to store all variables
uint8_t save_variables(void) {
	if (eeprom_flags & _BV(active))
		return -1; // busy storing configuration, take it easy!

	eeprom_flags |= _BV(start) | _BV(active);
	return 0;
}


/***************************************************************************/
/*** store control                                                       ***/
/***************************************************************************/
// place this in the mainloop, it polls the eeprom_flags to see if
// there is any work to do
void store_control(void) {
	if (eeprom_flags & _BV(active)) { // are the eeprom going to do anything at all
		if (eeprom_flags & _BV(start)) {   // any new work for me?
			eeprom_flags &= ~_BV(start);
			state = BEGIN_STORING;
			//*CAA ledstate |= _BV(LED_MISC); // turn on led
		} else {
			if (!(eeprom_flags & _BV(busy))) { // has the write completed
				if (store_setup()) {
					eeprom_flags &= ~_BV(active);  // done, clear flags
					//*CAA ledstate &= ~_BV(LED_MISC); // turn off led
				}
			}
		}
	}
}


/***************************************************************************/
/*** store setup                                                         ***/
/***************************************************************************/
// setup pointers to sram/eeprom storage and call function 'store_bytes'
// returns 1 when done
uint8_t store_setup(void) {
	uint8_t ret_val=0;

	switch (state) {
	case BEGIN_STORING:
		// setup pointers for the config-structure
		p_eeprom = (uint8_t *)&config_ee;
		p_sram = (uint8_t *)&config;
		length = sizeof(struct config_t);
		offset = 0;
		state = STORE_CONFIG;
		break;

	case STORE_CONFIG:
		if (store_bytes()) {
			// setup pointers for the ve-structure
			p_eeprom = (uint8_t *)&ve_ee;
			p_sram = (uint8_t *)&ve;
			length = sizeof(struct ve_t);
			offset = 0;
			state = STORE_VE;
		}
		break;

	case STORE_VE:
		if (store_bytes()) {
			state = DONE;
		}
		break;

	default:
		ret_val = 1;
		break;
	}

	return ret_val;
}


/***************************************************************************/
/*** store bytes                                                         ***/
/***************************************************************************/
// returns 1 when entire stucture has been saved
uint8_t store_bytes(void) {
	uint8_t edata, sdata;

	edata = eeprom_read_byte(p_eeprom+offset);
	sdata = *(p_sram+offset);

	// search the structure until a difference is found between memory copy and eeprom,
	// or if the entire structure has been searched
	while ((edata == sdata) && (offset < (length-1))) {
		offset++;
		edata = eeprom_read_byte(p_eeprom+offset);
		sdata = *(p_sram+offset);
	}
	
	// store the value in eeprom 
	if (edata != sdata) { // test that the loop wasn't exited due to offset==sizeof...
		eeprom_write_byte(p_eeprom+offset, sdata);
		
		// set flags, and enable eeprom isr
		eeprom_flags |= _BV(busy);
		EECR |= _BV(EERIE);
	}
	
	if (offset >= (length-1)) {
		return 1;
	} else {
		offset++;
		return 0;
	}
}


/***************************************************************************/
/*** INTERRUPT: EEPROM ready                                             ***/
/***************************************************************************/
ISR(EE_READY_vect) {
	eeprom_flags &= ~_BV(busy);  // signal completion of write
	EECR &= ~_BV(EERIE);         // disable interrupt
}


