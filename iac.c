
/*
TODO:
COMPLICATED_IAC
iac_count will never be initialized since calc_iac_pos()
only is run when engine.status & _BV(running)!

SIMPLE_IAC
The stepper moves to fully open when ignition is turned on.
*/


/*
Q: Why both a simple and a complicated controller?
A: The complicated is very complicated to get working! It can get the 
	 idle speed adjusted very fast and precise, but has problems working
	 optimally both at cold and warm engine temperatures.
	 
	 The simple is very easy and fast to configure, and seems to be
	 adequate for most purposes.

Q: Which should I use?
A: The simple!
*/


/*
General information:
The global variable 'idle_rpm' contains the target-rpm.
The global variable 'iac_pos' is the calculated absolute position
of the stepper.
*/


#include "global.h"
#include "iac.h"
#include "helpers.h"
#include "adc.h"

#include <inttypes.h>
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>


volatile extern struct engine_t engine;
volatile extern uint8_t sensors[];
extern struct config_t config;
volatile struct iac_t iac;


#ifdef SIMPLE_IAC

/***************************************************************************/
/*** calc iac pos                                                        ***/
/***************************************************************************/
// The unit of all rpm variables are [actual_rpm/10]
void calc_iac_pos(void) {
	static uint16_t last_rpm;
	uint16_t current_rpm;

	cli(); // everything that can go wrong, will go wrong
	current_rpm = engine.rpm_hr;
	sei(); // ... unless we use cli/sei :)


	// the FAST_IDLE_THRES avoids idle_rpm switching back and forth when coolant ~ fastidle
	if (engine.coolant >= config.iac[FAST_IDLE_TEMP]) {
		idle_rpm = config.iac[WARM_RPM];
	} else if (engine.coolant < (config.iac[FAST_IDLE_TEMP] - FAST_IDLE_THRES)) {
		idle_rpm = config.iac[COLD_RPM];
	}


	if (engine.status & _BV(crank)) {
		cli();
		engine.status &= ~_BV(idle);
		sei();

		// no movement of iac while cranking

	} else {
		
		if ((engine.tps <= config.iac[TPS_THRES])) {
			// try using the iac position from last time we were in idle-mode
			if (!(engine.status & _BV(idle))) {  // wasn't in idle-mode before
				cli();
				iac.status &= ~_BV(left_idle);
				sei();
				// limit the iac adjustment
				iac_pos = limit_i16(last_iac_pos, config.iac[MAX_STEPS], 0);
				set_iac_pos(iac_pos);
			}

			cli();
			engine.status |= _BV(idle);
			sei();

			if (!(iac.status & _BV(busy))) { // if not busy moving the iac
				static uint8_t fine_count, coarse_count;
				static uint8_t sync_count;
				int8_t adjust=0;

				if (++coarse_count >= config.iac[COARSE_PERIOD]) { // how often can the iac be moved for coarse adjustments
					coarse_count=0;
			
					if (current_rpm > (idle_rpm + config.iac[COARSE_DEV])) { // going way too fast
						adjust = -(config.iac[ADJUST_COARSE] & 0x0F);
						sync_count=0;
					} else if (current_rpm < (idle_rpm - config.iac[COARSE_DEV])) {  // going way too slow
						adjust = config.iac[ADJUST_COARSE] >> 4;
						sync_count=0;
					} else {
						adjust = 0;  // no adjustments here
					}
				}
			
				if (++fine_count >= config.iac[FINE_PERIOD]) { // how often can the iac be moved for fine adjustments
					fine_count=0;
			
					if (adjust == 0) {  // if any coarse adjustments, then don't mess with them
						if (current_rpm > (idle_rpm + config.iac[FINE_DEV])) { // going a little too fast
							adjust = -(config.iac[ADJUST_FINE] & 0x0F);
			
							// resync the iac if it has drifted.
							// will only be resync'ed when there is a small deviation from target-rpm
							// and if that deviation can't be eliminated otherwise.
							if (iac.position== 0) {
								if (sync_count < 255) sync_count++;
								if (sync_count >= config.iac[RESYNC]) { // time for resync'ing the iac
									sync_count=0;
									cli();
									if (iac.position==0) iac.position++;  // cheat the iac, now it thinks it is at position 1
									sei();
								}
							}
						} else if (current_rpm < (idle_rpm - config.iac[FINE_DEV])) {  // going a little too slow
							adjust = config.iac[ADJUST_FINE] >> 4;
							sync_count=0;
						} else { // hey, then we must have reached the right rpm, hurray!
							adjust = 0; // no adjustments here
							sync_count=0;
						}
					}
				}
				// limit the iac adjustment
				iac_pos = limit_i16(iac.position + adjust, config.iac[MAX_STEPS], 0);
				set_iac_pos(iac_pos);

			}

		} else {  // not in idle-mode
			cli();
			engine.status &= ~_BV(idle);
			sei();

			// remember last iac position
			if (!(iac.status & _BV(left_idle))) {
				last_iac_pos = iac_pos;
				cli();
				iac.status |= _BV(left_idle);
				sei();
			}
		}
	}
	last_rpm = current_rpm;
}
#endif







#ifdef COMPLICATED_IAC


/***************************************************************************/
/*** calc iac pos                                                        ***/
/***************************************************************************/
// The unit of all rpm variables are [actual_rpm/10]
void calc_iac_pos(void) {
	static uint16_t last_rpm;
	uint16_t current_rpm;
	static uint8_t iac_count; // timer for controlling the extra air at start

	cli(); // everything that can go wrong, will go wrong
	current_rpm = engine.rpm_hr;
	sei(); // ... unless we use cli/sei :)


	// the FAST_IDLE_THRES avoids idle_rpm switching back and forth when coolant ~ fastidle
	if (engine.coolant >= config.iac[FAST_IDLE_TEMP]) {
		idle_rpm = config.iac[WARM_RPM];
	} else if (engine.coolant < (config.iac[FAST_IDLE_TEMP] - FAST_IDLE_THRES)) {
		idle_rpm = config.iac[COLD_RPM];
	}


	if (engine.status & _BV(crank)) {
		cli();
		engine.status &= ~_BV(idle);
		sei();

		// no movement of iac while cranking
//#warning "whoops, iac_count will never be initialized!"
		iac_count = config.iac[START_COUNT];  // initialize iac warmup counter

	} else {
		
		if ((engine.tps <= config.iac[TPS_THRES])) {
			uint8_t afterstart;
			uint8_t target_rpm;

			cli();
			engine.status |= _BV(idle);
			sei();

			// Afterstart Air Enrichment
			// this can be used for kickstarting the alternator, or just to get the engine running :-)
			// Add some extra 'afterstart' rpms to idle_rpm
			if (config.iac[START_COUNT] == 0) {
				afterstart = 0;
			} else {
				if (iac_count) iac_count--;  // decremented every 0.1 sec

				// decrease the added 'rpm' towards zero
				afterstart = linear_interp(config.iac[START_COUNT], 0, config.iac[START_RPM], 0, iac_count);
			}
			idle_rpm += afterstart;


			// exponentially approach the target rpm
			// set the TAU_[INCREASE|DECREASE]_RPM to zero to disable this
			if (current_rpm > idle_rpm)
				target_rpm = ((current_rpm - idle_rpm) * config.iac[TAU_DECREASE_RPM]) / 256 + idle_rpm;
			else
				target_rpm = ((current_rpm - idle_rpm) * config.iac[TAU_INCREASE_RPM]) / 256 + idle_rpm;

		 
			// error term
			// a positive error means the engine is running slower than it is supposed to do
			idle_error = target_rpm - current_rpm;
			
			// asymmetric behavior. If rpm is low, increase rpm fast
			if (config.iac[PID_CONF] & _BV(pid_asymmetric)) {
				if (idle_error > 0)
					idle_error = idle_error * 2;
			}

			// check if the error is large enough to allow adjustments.
			// if the deadband is too small, the iac will be adjusting all the time (not recommend!)
			if (abs(idle_error) > config.iac[DEADBAND]) {
				int8_t pid;
						
				// Differential change of rpm
				// A positive change in rpm results in negative d(rpm). This is caused by the way 'error' is defined
				idle_rpmdot = last_rpm - current_rpm;
			
				// integrate error term
				idle_integral += idle_error;
				// yes please, we really want to limit the integrator
				idle_integral = limit_i16(idle_integral, config.iac[INTEGRAL_LIMIT], -config.iac[INTEGRAL_LIMIT]);
			
			
				// final computed iac position change
				pid = (config.iac[KP] * idle_error + config.iac[KI] * idle_integral + config.iac[KD] * idle_rpmdot) / 256;
			
				// limit the iac adjustment
				iac_pos = limit_i16(pid, config.iac[MAX_STEPS], 0);
				set_iac_pos(iac_pos);
			
			}

		} else {  // not in idle-mode
			cli();
			engine.status &= ~_BV(idle);
			sei();

		}
	}
	last_rpm = current_rpm;
}
#endif



/***************************************************************************/
/*** set iac pos                                                         ***/
/***************************************************************************/
void set_iac_pos(uint8_t pos) {
	// don't mess with the sync sequence!
	if (!(iac.status & (_BV(sync_close) | _BV(sync_open)))) {

		if (pos < iac.position) {   // close iac
			cli();
			iac.count = iac.position - pos;
			iac.status &= ~_BV(direction);
			iac.status |= _BV(busy);
			sei();
		} else {   // open iac
			cli();
			iac.count = pos - iac.position;
			iac.status |= _BV(direction) | _BV(busy);
			sei();
		}
	}
}



/***************************************************************************/
/*** init iac                                                            ***/
/***************************************************************************/
void init_iac(void) {
	uint8_t t;

	// at power on, the iac is fully closed (iac_sync counts), thereby the 
	// position of the iac is known. Right after, the iac is moved to
	// position = f(coolant)

#ifdef COMPLICATED_IAC
	idle_rpm = config.iac[COLD_RPM] + config.iac[START_RPM];  // this is needed later
#else
	idle_rpm = config.iac[COLD_RPM];  // this is needed later
#endif
	t = linear_interp(0, 205, config.iac[COLD_START_POS], config.iac[WARM_START_POS], engine.coolant);
	iac.start_pos = limit_i16(t, config.iac[MAX_STEPS], 0);
	last_iac_pos = iac.start_pos;

	iac.status = _BV(busy) | _BV(sync_close);
	iac.status &= ~_BV(direction);

	iac.position = config.iac[MAX_STEPS] + IAC_SYNC;
	iac.count = config.iac[MAX_STEPS] + IAC_SYNC;

}




/***************************************************************************/
/*** move iac                                                            ***/
/***************************************************************************/
#define BATT_8V 68

void move_iac(void) {
	static uint8_t phase;

	if (iac.count) {
		// test battery voltage
		if (!(config.iac[CONF] & _BV(low_power_off)) ||  // don't care about batt voltage
				((config.iac[CONF] & _BV(low_power_off)) && (sensors[ADC_BATT] > BATT_8V))) { // battery > 8V?

			iac.count--;
			iac.status &= ~_BV(iac_batt_low);

			if (iac.status & _BV(direction)) {	// open iac
				phase++; iac.position++;
			} else {	// close iac
				phase--; iac.position--;
			}

			// *CAA PORTC = next_step(phase & 0x03) | _BV(STEP_EN);  // move the iac

			if (iac.count == 0) {
				iac.status &= ~_BV(busy);
			
				if (iac.status & _BV(sync_close)) { // when done sync'ing, the position is know
					iac.status &= ~_BV(sync_close);
					iac.status |= _BV(sync_open);  // THIS IS TOO CROWDED!
					iac.count = iac.start_pos;
					iac.status |= _BV(direction) | _BV(busy);
				} else if (iac.status & _BV(sync_open)) {
					iac.status &= ~_BV(sync_open);
				}
			}
		} else {// batt too low, disable driver
			// *CAA PORTC &= ~(_BV(STEP_EN) | _BV(STEP_A) | _BV(STEP_B) | _BV(STEP_C) | _BV(STEP_D));
			iac.status |= _BV(iac_batt_low);
		}
	} else {  // no step requested, disable driver
		iac.status &= ~(_BV(busy) | _BV(sync_close) | _BV(sync_open));
		if (config.iac[CONF] & _BV(power_off_iac)) {
			// *CAA PORTC &= ~(_BV(STEP_EN) | _BV(STEP_A) | _BV(STEP_B) | _BV(STEP_C) | _BV(STEP_D));
		}
	}
}



// *CAA uint8_t PROGMEM STEP_PINS[] = { _BV(STEP_A), _BV(STEP_B), _BV(STEP_C), _BV(STEP_D) };

/***************************************************************************/
/*** next_step                                                           ***/
/***************************************************************************/
// Allow allmost total reconfiguration of the steppers windings connections 
// through PC configuration program. Nice for people too lazy for soldering the
// stepper wires correctly!
uint8_t next_step(uint8_t phase) {
	uint8_t val, idx;

	switch (phase) {
		case 0: idx = config.iac[STEP_SEQ] & 0x03; break;
		case 1: idx = (config.iac[STEP_SEQ] >> 2) & 0x03; break;
		case 2: idx = (config.iac[STEP_SEQ] >> 4) & 0x03; break;
		case 3: idx = (config.iac[STEP_SEQ] >> 6) & 0x03; break;
		default: idx = 0; break;
	}
	
	// *CAA val = pgm_read_byte(&STEP_PINS[idx]);
	//*CAA return val;
	return 0; //*CAA should be return val;
}


