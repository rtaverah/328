
#include "arduems.h"
#include "global.h"
#include "comm.h"
#include "fuelcalc.h"
#include "helpers.h"
#include "storage.h"
#include "adc.h"
#include "actuators.h"
#include "tables.h"
#include "ve.h"
#include "iac.h"
#include "buffer.h"

#include <avr/io.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>
#include <avr/pgmspace.h>


/*
TODO:
	Check reset source at power-up
	Watchdog timer 
*/

extern int16_t idle_integral;
extern int16_t idle_error;
extern int16_t idle_rpmdot;
extern int16_t pos;



extern uint8_t sensors[];
extern volatile struct squirt_t inj_port1, inj_port2;
extern volatile struct engine_t engine;
extern volatile struct config_t config;
extern volatile struct time_t rtc;
extern struct corr_t corr;
extern volatile struct iac_t iac;
extern uint8_t idle_rpm;

extern uint8_t pw2, pw3;

extern struct ve_at_t tune_me;
extern struct ve_stat_res_t res;

extern uint8_t tpsfuelcut;
uint8_t ego_warmup_count;

#ifdef RV8MT
uint8_t scantime, scantime_cnt;
#endif


volatile uint8_t alive;
//extern volatile uint8_t crank_enable_timeout;

/* to avoid conflict with LCD pins, LEDPORT is always set from userspace,
 * not interrupt 
 */
volatile uint8_t ledstate;
void setleds(void)
{
	/* DONE:
	 * choose new LED pins on ATmega328
	 * not all can be used so choose most important
	 */
	
	LEDPORT = ((ledstate & LEDMASK 
		/*  LED_WARMUP  6 is masked, so that I see in the dark if voltage is too high
		 * or low on the TACHIN pin
		 */
#ifdef PINE2LED
		& ~64) | (PINE & 64)
#else
	)
#endif
	) | (LEDPORT & ~LEDMASK);
}
/***************************************************************************/
/*** main entry point                                                    ***/
/***************************************************************************/
int main(void) {
	uint8_t first_run=1;

	init();			// init HW and misc. variables
	primepulse();	// schedule a priming pulse
	sei();			// enable interrupts

	// main loop
	for (;;) {
		setleds();
		calc_parameters();
		setleds();
		coolant_fan();
		setleds();
		fast_idle_valve();
		setleds();

		// only recalculate rpm if necessary
		if (engine.status_ext & _BV(new_rpm)) {
			cli();
			engine.status_ext &= ~_BV(new_rpm);
			sei();
			calc_rpm();
		}

		setleds();

		// make sure the engine is spinning
		if (engine.rpm) {
			// only enter cranking mode on power-on or if rpm drops to zero.
			if ((engine.status_ext & _BV(crank_enable)) && (engine.rpm <= config.cranking_thres)) {
				cranking();
			} else {    // running
				cli();
				engine.status_ext &= ~_BV(crank_enable);
				sei();

				// check if idle, and do idle control / anti-stall ?

				if (first_run) {  // this will force warmup enrichment
					first_run=0;
					cli();
					engine.status |= _BV(crank);
					sei();
				}
				warmup_enrich();

				setleds();
				tps_acc_enrich();
				closed_tps_decel();
				o2();
				ve_table_lookup();
				setleds();
				calc_total_enrichment();
			}
		}

		setleds();
		// every 1/10th second    
		if (rtc.flags & _BV(tenth_second)) {
			cli();
			rtc.flags &= ~_BV(tenth_second);
			sei();
	
			if (engine.status & _BV(running)) {
				if (!(config.iac[CONF] & _BV(disable_iac))) {
					calc_iac_pos();
				}
			}
		}

		// every second
		if (rtc.flags & _BV(second)) {
			cli();
			rtc.flags &= ~_BV(second);
			sei();

			if (engine.status & _BV(running)) {  // engine is running,
				if (engine.status_ext & _BV(o2_not_ready)) { // and the O2 isn't ready
					if (ego_warmup_count > 0) 
						ego_warmup_count--; // wait some time
					else
						engine.status_ext &= ~_BV(o2_not_ready);  // and enable the O2
				}
			} else {
				if (ego_warmup_count < EGO_WARMUP)
					ego_warmup_count++;
			}
		}

		setleds();
		store_control(); // store variables to eeprom
		
		// process comm recieve buffer
		buffer_process(&comm_rx_buffer, &comm);
		
		
		alive = 1;  // indicate that we've reached end of loop

#ifdef RV8MT
		scantime = scantime_cnt;
		scantime_cnt = 0;
#endif

	}

	return (0);
}



/***************************************************************************/
/*** init                                                                ***/
/***************************************************************************/
void init(void) {

	// init io
	//PORTA = 0x0F; // misc LEDs
	//DDRA = 0xF8;

	//PORTB = 0x63; // pwm active low
	//DDRB = 0xFC;

	//PORTC = 0xE0; // stepmotor
	//DDRC = 0x1F;

	//PORTD = 0xFF;
	//DDRD = 0x00;
	
	//PORTE = 0xFF - 64; // we do NOT want pullup at the TACH in
	//DDRE = 0x00;

	PORTC = 0x00; // adc channels
	DDRC = 0x00;

	//PORTG = 0xFF;
	//DDRG = 0x18;

	//if (PINB & _BV(J1))
		ms_version = 20;
	//else
		//ms_version = RV8_EMULATION;

	// init timers
	// setup timer0 to interrupt at 10 kHz
	TCCR0A = _BV(WGM01); 	// CTC mode
	TCCR0B = _BV(CS01);		// prescalar 8
	OCR0A = 199;			// timer counts
	TIMSK0 = _BV(OCIE0A);  	// enable output-compare interrupt TIMER0_COMPA

	// init pwm
	// using timer1 16-bit
	// this enables Mode 15 on page 136 in ATmega328 datasheet
	// Compare Output Mode, Fast PWM. clear timer at TOP, TOP is stored in ICR1
	TCCR1A = _BV(WGM11);
	TCCR1B = _BV(WGM13) | _BV(WGM12) | _BV(CS10); // CS10: prascaler 1
	ICR1 = PWM_FREQUENCY; // change pwm frequency

	// setup external interrupt
#ifdef PWANAL 
	EICRA = _BV(ISC00);		// INT0 spark interrupt: falling or rising edge generates interrupt
#else
	EICRA = _BV(ISC01);		// INT0 spark interrupt: falling edge generates interrupt
#endif
	
	EIMSK = _BV(INT0);  // INT0 spark interrupt: enable spark interrupt

	// init rs232
	initUART();

	// load constants from eeprom to ram
	init_storage();
	load_variables();

	// init the sensor array
	initSensors();

	// get current barometric pressure
	sensors[ADC_BARO] = sensors[ADC_MAP];

	// init global variables
	calc_parameters();

	engine.status = 0;
	engine.status_ext = _BV(o2_not_ready) | _BV(crank_enable);
	engine.rpm = 0;
	engine.rpm_p = 0xFFFF; // just set it to some value that translates into a low rpm
	engine.rpm_c = 0;
	engine.rpm_hr = 0;
	engine.last_tps = engine.tps;

	inj_port1.pw = 0;
	inj_port2.pw = 0;

	ego_warmup_count = EGO_WARMUP;

	//  crank_enable_timeout = CRANK_TIMEOUT;

	init_fuelcalc();

	// init iac parameters
	init_iac();

	// enable fuelpump, 
	// it will be turned off within a couple of seconds, if engine isn't started
	PORTB |= _BV(FUELPUMP);
	// done, we have pressure, the fuelpump is really fast!
}


