
#include "fuelcontrol.h"
#include "fuelcalc.h"
#include "global.h"
#include "adc.h"
#include "comm.h"
#include "iac.h"

#include <avr/io.h>
#include <inttypes.h>
#include <avr/interrupt.h>
#include <avr/sfr_defs.h>

extern uint8_t sensors[];

volatile struct squirt_t inj_port1, inj_port2;
volatile struct engine_t engine;
struct time_t rtc;

extern struct config_t config;
extern struct step_t step;

extern uint8_t alive;
uint8_t led_timeout;

extern uint8_t tpsaclk;
extern uint8_t egocount;
extern uint8_t asecount;

#ifdef RV8MT
extern uint8_t scantime_cnt;
#endif


/***************************************************************************/
/*** INTERRUPT: Timer @ 0.1 ms ***/
/***************************************************************************/
// this is a long and painful interrupt handler!
ISR(TIMER0_COMPA_vect) {

	//start new injection, channel 1
	if (inj_port1.status & _BV(scheduled)) {
		inj_port1.status &= ~_BV(scheduled);
		inj_port1.status |= _BV(firing) | _BV(enabled);
		ledstate |= _BV(LED_SQUIRT);  // sled on
		PORTB &= ~_BV(PWM_A);      // inject1 active
		PORTD |= _BV(FLYBACK_A);   // flyback enable
	}

	//start new injection, channel 2
	if (inj_port2.status & _BV(scheduled)) {
		inj_port2.status &= ~_BV(scheduled);
		inj_port2.status |= _BV(firing) | _BV(enabled);
		ledstate |= _BV(LED_SQUIRT);  // sled on
		PORTB &= ~_BV(PWM_B);      // inject2 active
		PORTD |= _BV(FLYBACK_B);   // flyback enable
	}

	//check for end of injection, channel 1
	if (inj_port1.status & _BV(firing)) {
		if (inj_port1.pwrun >= inj_port1.pw) {
			inj_port1.status &= ~(_BV(firing) | _BV(enabled) | _BV(scheduled));
			TCCR1A &= ~( _BV(COM1A1) | _BV(COM1A0) );	// disable pwm
			PORTB |= _BV(PWM_A);                      // inject1 deactive
			PORTD &= ~_BV(FLYBACK_A);                 // flyback disable
		} else if (inj_port1.pwrun == inj_port1.pwm_delay) {
			OCR1A = inj_port1.pwm_dc * 10;  // pwm duty cycle, multiply by 10 to increase range
			TCCR1A |= _BV(COM1A1) | _BV(COM1A0);	// enable pwm
		}
		inj_port1.pwrun++;
	}

	//check for end of injection, channel 2
	if (inj_port2.status & _BV(firing)) {
		if (inj_port2.pwrun >= inj_port2.pw) {
			inj_port2.status &= ~(_BV(firing) | _BV(enabled) | _BV(scheduled));
			TCCR1A &= ~( _BV(COM1B1) | _BV(COM1B0) );	// disable pwm
			PORTB |= _BV(PWM_B);                      // inject2 deactive
			PORTD &= ~_BV(FLYBACK_B);                 // flyback disable
		} else if (inj_port2.pwrun == inj_port2.pwm_delay) {
			OCR1B = inj_port2.pwm_dc * 10;  // pwm duty cycle, multiply by 10 to increase range
			TCCR1A |= _BV(COM1B1) | _BV(COM1B0);	// enable pwm
		}
		inj_port2.pwrun++;
	}


	// is engine still alive?
	if (engine.status & _BV(running)) {
		if (! ((inj_port1.status & _BV(enabled)) || (inj_port2.status & _BV(enabled))) ) {
			ledstate &= ~_BV(LED_SQUIRT); //sled off
		}

		//filter noise on interrupt0, eg. wait some time before reenabling the interrupt
		if ((engine.rpm_c == (engine.rpm_p >> 1)) || (engine.rpm_c == TACH_FILTER_MAX)) {
			EIFR |= _BV(INT0);  // INT0 spark interrupt: clear interrupt flag
			EIMSK = _BV(INT0);  // INT0 spark interrupt: enable interrupt
		}


		// update the rpm counter
		engine.rpm_c++;
		if ((engine.rpm_c>>8) >= 100) { // is period above threshold (2.5 sec)
			engine.status = 0; // engine is stalled, clear all in engine
			engine.status_ext |= _BV(o2_not_ready) | _BV(crank_enable);

			engine.rpm_p = 0xFFFF;
			engine.rpm_c = 0;
			engine.rpm = 0;
	
			PORTB &= ~( _BV(FUELPUMP) | _BV(IDLEVALVE) ); //fuelpump off, idlevalve off
			ledstate &= ~( _BV(LED_SQUIRT) | _BV(LED_WARMUP) );   //sled off, wled off

			inj_port1.pw = 0;
			inj_port2.pw = 0;
		}
	} else { // not running, enable the interrupt
		EIMSK = _BV(INT0);  // INT0 spark interrupt: enable spark interrupt
	}


	/*
	// use the 4th led to indicate sensor failure
	if (sensors[EGO] == 0 || sensors[MAT] == 255 || sensors[CLT] == 255 ||
			sensors[TPS] == 255 || sensors[TPS] == 0 || 
			sensors[MAP] == 255 ||  sensors[MAP] == 0) {

		ledstate |= _BV(LED_MISC); //indicate sensor failure
	} else {
		ledstate &= ~_BV(LED_MISC);
	}
	*/

#ifdef RV8MT
	scantime_cnt++;
#endif

	// RTC
	if (++rtc.tick == 10) { // every ms
		static uint8_t step_clk, adc_clk, button_clk;

		rtc.tick=0;
		statusLed();

		if (++adc_clk == ADC_PERIOD) {
			adc_clk=0;
			startADC(); //start adc
		}

		// acc. clock
		if (++rtc.tsec == 100) {  // 1/10 second
			rtc.tsec=0;
			tpsaclk++;
			engine.last_tps = engine.tps;
			rtc.flags |= _BV(tenth_second);

		}

		// seconds
		if (++rtc.ms == 1000) {
			rtc.ms=0;
			rtc.sec++;
			rtc.flags |= _BV(second);

		}

		// this is used for setting a flag every 10 ms
		if (++button_clk == 10) {
			button_clk=0;
			rtc.flags |= _BV(ten_ms);
		}

		// service IAC movements
		if (step_clk++ >= ((config.iac[CONF] & 0xF0)>>4)) {
			step_clk=0;
			if ((engine.status & _BV(running)) || (config.iac[CONF] & _BV(test_iac)))
				move_iac();
		}
	}
}


/***************************************************************************/
/*** INTERRUPT: ignition (triggers on falling edge)                      ***/
/***************************************************************************/
ISR(INT0_vect) {
#ifdef PWANAL
#warning squirt function is broken
	// NOTE: with PWANAL we get an interrupt after rising AND falling edge
	// Check external pin to decide if it was a rising or falling edge

	uint16_t now;
	uint16_t diff;
	uint8_t rise;

	rise = PINE & 64;
	now = TCNT3; // TODO: check if read 16bits correctly

	diff = now - tachin.rise_prev;
 	// diff >>= ROTBY0 ;

	if(rise == 0){       // it was a rising edge (before the inverting NPN transistor)
		tachin.persum += diff;
		//        diff >>= 7;     // 128 * 255 * 4usec = 130msec
		//       tachin.persqrsum += diff * diff ;
		tachin.rise_prev = now;
		if(diff > MINPER) tachin.edge_cnt ++;
		else	tachin.edge_bad ++;

		if (tachin.edge_cnt & 16){
		 	ledstate |= _BV(LED_MISC);
		} else {
		 	ledstate &= ~_BV(LED_MISC);
		}
		if((tachin.edge_cnt & AGGRNUM) == AGGRNUM){
			// copy to pwdisplay_t
			pwdisplay_from_tachin(now);
			tachin_reset();
			tachin.rise_0 = tachin.rise_prev = now;
		}
	} else { // it was a falling edge
		tachin.pwsum += diff;
		//                diff >>= 4;     // 4095 (*4us) to fit <255
		//               tachin.pwsqrsum +=  diff * diff ;
		/* original falling-edge behaviour: */
		engine.rpm_p = engine.rpm_c;
		engine.rpm_c = 0;
		engine.status_ext |= _BV(new_rpm);  // start a rpm calculation
	}
#else
	uint8_t sched_squirt;
	static uint8_t altcount; // Alternate count selector
	static uint8_t igncount; // Ignition pulse counter

	EIMSK &= ~_BV(INT0);   // INT0 spark interrupt: dissable interrupt

	if (egocount < 255)
		egocount++;

	if (asecount < 255)
		asecount++;

	// try using input capture, or read the timer value instead. 
	engine.rpm_p = engine.rpm_c;
	engine.rpm_c = 0;
	engine.status_ext |= _BV(new_rpm);  // start a rpm calculation

	PORTB |= _BV(FUELPUMP); // fuelpump on
	engine.status |= _BV(running);

	if (engine.status & _BV(crank)) {
		engine.status &= ~(_BV(tpsaen) | _BV(tpsden));
		sched_squirt = true;
	} else {
		igncount++;

		if (igncount == config.divider) {
			sched_squirt = true;
		} else {
			sched_squirt = false;
			if (igncount >= 0x08) {
				igncount = 0;
			}
		}
	}


	if (sched_squirt) {
		igncount = 0;

		if ( (engine.status & _BV(crank)) || (config.alternate == 0) ) {

			//schedule both injectors
			inj_port1.pw = inj_port1.pwcalc;
			inj_port2.pw = inj_port2.pwcalc;

			inj_port1.pwrun = 0;
			inj_port2.pwrun = 0;

			inj_port1.status |= _BV(scheduled) | _BV(enabled);
			inj_port2.status |= _BV(scheduled) | _BV(enabled);

		} else {
			altcount++;
			if (altcount & 0x01) {
				inj_port2.pw = inj_port2.pwcalc;
				inj_port2.pwrun = 0;
				inj_port2.status |= _BV(scheduled) | _BV(enabled);
			} else {
				inj_port1.pw = inj_port1.pwcalc;
				inj_port1.pwrun = 0;
				inj_port1.status |= _BV(scheduled) | _BV(enabled);
			}
		}
	}
#endif
}




/*
	The status led indicates that the timer interrupt is active 
	and the main loop hasn't stuck somewhere.
*/
// this is part of an interrupt routine!
void statusLed(void) {
	static uint8_t alivecount;

	if ((alive == 1) && (alivecount == 0)) {
		alive = 0;
		alivecount++;
		ledstate &= ~_BV(LED_STATUS);  //statusled on
	} else {
		if (alivecount == 20) {
			ledstate |= _BV(LED_STATUS); //statusled off
		}
		if (alivecount) { // only count when enabled
			alivecount++;
			if (engine.status_ext & _BV(o2_not_ready)) {
	if (alivecount == 125)
		alivecount=0; // blink faster at warmup
			}
		}
	}
}
