
/*
TODO:

There is a timeout during acceleration, this should be fixed.
Is the same timeout appearing in the B&G asm code?

*/


#include "global.h"
#include "fuelcalc.h"
#include "helpers.h"
#include "adc.h"
#include "tables.h"
#include "iac.h"

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <string.h>


volatile extern struct squirt_t inj_port1, inj_port2;
volatile extern struct engine_t engine;
volatile extern struct time_t rtc;
volatile extern uint8_t sensors[];
extern struct config_t config;

volatile uint8_t tpsaclk; // TPS enrichment timer clock in 0.1 second resolution

uint8_t tpsaclkcmp; // Comparison value for TPS acceleration time - from lookup table

volatile uint8_t egocount; // Counter value for EGO step - incremented every ignition pulse
volatile uint8_t asecount; // Counter value for after-start enrichment counter - every ignition pulse

struct corr_t corr;

extern uint8_t PROGMEM KPAFACTOR4250[];
extern uint8_t PROGMEM KPAFACTOR4115[];
extern uint8_t PROGMEM BAROFAC4250[];
extern uint8_t PROGMEM BAROFAC4115[];
extern uint8_t PROGMEM THERMFACTOR[];
extern uint8_t PROGMEM AIRDENFACTOR[];

#ifdef RV8MT
uint8_t wwu_pos, tps_pos;
#endif

void init_fuelcalc(void) {
	corr.ego = 100;
	corr.air = 100;
	corr.warm = 100;
	corr.baro = 100;
	corr.gammae = 100;
	corr.ve = 100;
	corr.tpsaccel = 0;
	corr.tpsfuelcut = 100;
	corr.tpsfuelcut2 = 100;
}

/***************************************************************************/
/*** warm-up and after-start enrichment                                  ***/
/***************************************************************************/
void warmup_enrich(void) {
	uint8_t warm_enrich;
	uint8_t my_status;
	struct search_table_t st;

	/* Warm-up enrichment */
	cli();
	my_status = engine.status;
	if (my_status & (uint8_t) _BV(crank)) {
		my_status &= ~_BV(crank);
		my_status |= _BV(startw) | _BV(warmup);
		engine.status = my_status;
		asecount = 0;
		ledstate |= _BV(LED_WARMUP); // wled on
	}
	sei();

	/* use coolant to find wwurange */
	/* "compress" the 256 possible sensorvalues to a [0-9] range */

	// note, coolant is actual temperature +40 degrees F
	search_table(config.wwurange, sizeof(config.wwurange), engine.coolant, &st);
#ifdef RV8MT
	wwu_pos = st.uindex;
#endif
	/* then interpolate in wwu-table */
	/* use the [0-9] range to find enrichment */

	warm_enrich = linear_interp(config.wwurange[st.lindex],
						config.wwurange[st.uindex],
						config.wwu[st.lindex],
						config.wwu[st.uindex],
						engine.coolant);


	if (engine.status & _BV(startw)) {
		if (asecount < config.awc) {
			uint16_t wue;
//#warning "do we want the afterstart to be temperature dependent?"	
			wue = warm_enrich + linear_interp(0, config.awc, config.awev, 0, asecount);
			// check for overflow
			if ((wue >> 8) != 0)
				wue = 0xFF;
			warm_enrich = (uint8_t)wue;
		} else {
			cli();
			engine.status &= ~_BV(startw);
			sei();
		}
	}


	if (warm_enrich <= 100) {
		/* outside warm-up range, clear warm-up enrichment code */
		warm_enrich = 100;
		cli();
		engine.status &= ~( _BV(startw) | _BV(warmup) );
		sei();
		ledstate &= ~_BV(LED_WARMUP); // wled off
	}

	corr.warm = warm_enrich;
}





/***************************************************************************/
/*** Throttle posistion acceleration enrichment                          ***/
/***************************************************************************/
void tps_acc_enrich(void) {
	uint8_t tps_diff;
	uint8_t my_tps, my_last_tps;

	cli();
	my_tps = engine.tps;
	my_last_tps = engine.last_tps;
	sei();

	if (my_tps < my_last_tps) {
		tps_diff = my_last_tps - my_tps;
	
		/* deceleration */
		if (tps_diff >= config.tps_thresh) {  // above threshold?
			if (engine.status & _BV(tpsaen)) {
				corr.tpsfuelcut = 100;
				corr.tpsaccel = 0;
				cli();
				engine.status &= ~( _BV(tpsaen) | _BV(tpsden) );
				sei();
				//*CAA ledstate &= ~_BV(LED_ACCEL); //aled off
			} else { // fuel cut
				if (engine.rpm >= config.fuelcut_thres) {
					corr.tpsfuelcut = config.tpsdq;
					cli();
					engine.status &= ~_BV(tpsaen);
					engine.status |= _BV(tpsden);
					sei();
					//*CAA ledstate &= ~_BV(LED_ACCEL); //aled off
				}
			}
		} else {
			if (engine.status & _BV(tpsden)) {
				cli();
				engine.status &= ~( _BV(tpsaen) | _BV(tpsden) );
				sei();
				corr.tpsfuelcut = 100;
				corr.tpsaccel = 0;
			}
		}


	} else {
		tps_diff = my_tps - my_last_tps;

		/* acceleration */
		if (tps_diff >= config.tps_thresh) {

			/*
			The acceleration calculation needs to be reworked, it is unprecise.
			It times out, and restarts computation of enrichment.
			*/

			if (engine.status & _BV(tpsaen)) {
				uint8_t acc_temp_offset, acc_temp_mult, acc_enrich;
				uint16_t mtmp;
				struct search_table_t st;
			
				/* multiplier amount based on cold temperature */
				acc_temp_mult = linear_interp(0, 205, config.acmult, 100, engine.coolant);
				
				/* lookup table amount based on tpsdot */
				search_table(config.tpsdotrate, sizeof(config.tpsdotrate), tps_diff, &st);
#ifdef RV8MT
				tps_pos = st.uindex;
#endif
				acc_enrich = linear_interp(config.tpsdotrate[st.lindex],
								 config.tpsdotrate[st.uindex],
								 config.tpsaq[st.lindex],
								 config.tpsaq[st.uindex],
								 tps_diff);
			
				//	acc_enrich = linear_interp(st.lbound, st.ubound, config.tpsaq[st.index-1], config.tpsaq[st.index], tps_diff);
			
				/* calculate acc-enrichment, catch overflow */
				mtmp = mult_div100(acc_temp_mult, acc_enrich);
				if ((mtmp >> 8) != 0) // will the result fit in a byte?
					mtmp = 200;         // ... no!
			
				/* extra fuel due to temperature */
				acc_temp_offset = linear_interp(0, 205, config.tpsacold, 0, engine.coolant);
			
				mtmp += acc_temp_offset;
				if ((mtmp >> 8) != 0) // will the result fit in a byte?
					mtmp = 0xFE;        // ... no!
				
				if (mtmp > corr.tpsaccel) 
					corr.tpsaccel = (uint8_t)mtmp; // tpsaccel is the result of this function
			
				if ((engine.status & _BV(tpsden)) || (tpsaclk >= tpsaclkcmp))  {
					cli();
					engine.status &= ~( _BV(tpsaen) | _BV(tpsden) );
					sei();
					corr.tpsfuelcut = 100;
					corr.tpsaccel = 0;
					//*CAA ledstate &= ~_BV(LED_ACCEL); //aled off
				}
			} else { //start of acceleration, initialize some variables
				corr.tpsaccel = config.tpsaq[0];
				tpsaclk = 0;
				tpsaclkcmp = config.tpsasync;
				cli();
				engine.status |= _BV(tpsaen);
				engine.status &= ~_BV(tpsden);
				sei();
				//*CAA ledstate |= _BV(LED_ACCEL); //aled on
				//	ledstate &= ~_BV(LED_WARMUP); //wled off
			}
		} else { //is acceleration done?
			if ((engine.status & _BV(tpsden)) || (tpsaclk >= tpsaclkcmp)) {
				cli();
				engine.status &= ~( _BV(tpsaen) | _BV(tpsden) );
				sei();
				corr.tpsfuelcut = 100;
				corr.tpsaccel = 0;
				//*CAA ledstate &= ~_BV(LED_ACCEL); //aled off
			}
		}
	}
}

/***************************************************************************/
/*** Closed throttle deceleration                                        ***/
/***************************************************************************/
// remember to specify TPS_THRES else this won't work!
// please be sure to make fuelresume < fuelcut
void closed_tps_decel(void) {
	if (engine.tps > config.iac[TPS_THRES]) {  // normal driving, don't interfere
		corr.tpsfuelcut2 = 100;
	} else {  // throttle released, but rpm still high
		if ((corr.tpsfuelcut2 == 0) && (engine.rpm <= config.fuelresume))
			corr.tpsfuelcut2 = 100;
		else if ((corr.tpsfuelcut2 == 100) && (engine.rpm >= config.fuelcut))
			corr.tpsfuelcut2 = 0;
	}
}



/***************************************************************************/
/*** Exhaust gas oxygen sensor measurement section                       ***/
/***************************************************************************/
void o2(void) {
	uint8_t limit, new_ego, o2_is_lean;

	if ( (config.egodelta == 0) || 
			 (engine.rpm < config.rpmoxlimit) || 
			 (engine.status & (_BV(tpsaen) | _BV(tpsden))) || 
			 (engine.coolant < config.egotemp) || 
			 ((!(config.config13 & _BV(O2_WB_SENSOR))) && (engine.tps > O2_MAX_TPS)) ||   // Yeeehaaaaa mode!
			 (engine.status_ext & _BV(o2_not_ready)) ||  // skip the first 30 sec
			 (engine.kpa > O2_MAX_KPA) ) {

		corr.ego = 100;
		egocount = 0;
		cli();
		engine.status_ext &= ~_BV(o2_closed_loop);
		sei();

	} else {
		cli();
		engine.status_ext |= _BV(o2_closed_loop);
		sei();

		if (egocount > config.egocountcmp) {
			egocount = 0;

			// do we want variable AFR?
			// then search bin and interpolate(kpa)
		
			if (sensors[ADC_EGO] != config.voltoxtarget) { // no need to do adjustments
				if (sensors[ADC_EGO] < config.voltoxtarget) {
					if (config.config13 & _BV(O2_WB_SENSOR))
						o2_is_lean = false;     //wideband sensor has reversed slope
					else
						o2_is_lean = true;
				} else {
					if (config.config13 & _BV(O2_WB_SENSOR))
						o2_is_lean = true;    //wideband sensor has reversed slope
					else
						o2_is_lean = false;
				}
			
				if (o2_is_lean) {      //lean
					limit = 100 + config.egolimit;
					new_ego = corr.ego + config.egodelta;
					
					if (new_ego > limit)
						corr.ego = limit;
					else
						corr.ego = new_ego;
			
				} else {      //rich
					limit = 100 - config.egolimit;
					new_ego = corr.ego - config.egodelta;
			
					if (new_ego < limit)
						corr.ego = limit;
					else
						corr.ego = new_ego;
				}
			}
		}
	}
}



/***************************************************************************/
/*** calc total enrichment                                               ***/
/***************************************************************************/
void calc_total_enrichment(void) {
	uint8_t fuel_tmp, batt_tmp, pw;
	uint8_t batt_high, batt_low;
	uint16_t res;

	// 8-bit x 16-bit multiplications
	res = (uint16_t)corr.warm;
	res = mult_div100(corr.tpsfuelcut, res);
	res = mult_div100(corr.tpsfuelcut2, res);
	res = mult_div100(corr.air, res);
	res = mult_div100(corr.ego, res);
	res = mult_div100(corr.baro, res);

	if ((res >> 8) != 0)  // check for overflow
		corr.gammae = 0xFF;
	else
		corr.gammae = (uint8_t)res; // used for datalogging

	res = mult_div100(corr.ve, res);
	
	if (!(config.config13 & _BV(CONTROL_STRATEGY)))  // speed-density
		res = mult_div100(engine.kpa, res);

	res = mult_div100(config.req_fuel, res);

	if ((res >> 8) != 0) // blue screen of death.... nasty overflow!
		fuel_tmp = 0xFF;
	else
		fuel_tmp = (uint8_t)res;


	// battery voltage compensation
	// remember, low battery voltage only delays injector opening time
	batt_low = config.injopen + config.battfac;

	if (config.injopen <= config.battfac)
		batt_high = 0;
	else
		batt_high = config.injopen - config.battfac;

//#warning "this should be configurable via configuration struct"
	batt_tmp = linear_interp(61, 164, batt_low, batt_high, sensors[ADC_BATT]);

	// final pulsewidth calculation, wuhuw!
	if (fuel_tmp) {
		res = batt_tmp + fuel_tmp + corr.tpsaccel - config.injocfuel;
		if ((res >> 8) != 0)
			pw = 0xFF;
		else
			pw = (uint8_t)res;
	} else {
		pw = 0;
	}

	// anybody wanting a revlimiter, can insert it here --> X

	cli();
	inj_port1.pwcalc = pw;
	inj_port2.pwcalc = pw;
	sei();

}


/***************************************************************************/
/*** calc parameters                                                     ***/
/***************************************************************************/
void calc_parameters(void) {
	uint8_t t;

	// Manifold Air Pressure in kiloPascals
	if (config.config11 & _BV(MAP_SENSOR))
		engine.kpa = pgm_read_byte(&KPAFACTOR4250[sensors[ADC_MAP]]);
	else
		engine.kpa = pgm_read_byte(&KPAFACTOR4115[sensors[ADC_MAP]]);

	// Barometric correction enabled?
	if (config.config13 & _BV(BARO_CORRECTION)) {
		uint8_t val = sensors[ADC_BARO];

		// If the mcu resets while the engine is running, we don't believe 
		// 20 kPa being the atmospheric pressure.
		if ( (val < (config.baro - config.dbaro)) || 
				(val > (config.baro + config.dbaro)) ) {
//#warning "MegaTune wants the raw sensor value and will not discover the cheat!"
			corr.baro = config.baro;
			cli();
			engine.status_ext |= _BV(baro_problem);
			sei();
		} else {
			if (config.config11 & _BV(MAP_SENSOR))
				corr.baro = pgm_read_byte(&BAROFAC4250[sensors[ADC_BARO]]);
			else
				corr.baro = pgm_read_byte(&BAROFAC4115[sensors[ADC_BARO]]);
		}
	} else {
		corr.baro = 100; // no, not enabled
	}

	// Coolant temperature in degrees F + 40
	engine.coolant = pgm_read_byte(&THERMFACTOR[sensors[ADC_CLT]]);

	// Air Density Correction Factor
	t = pgm_read_byte(&AIRDENFACTOR[sensors[ADC_MAT]]);

	// This is a hack! When idling for a long time, the airtemperature ususally rises
	// resulting in the intake air to be warm. When it gets too warm, the idling will be rough.
	if (!(engine.status & (_BV(crank) | _BV(warmup))) && (t < AIRDEN_DISABLE)) {
		// normal running mode, but with low air density
		t = linear_interp(7, 15, AIRDEN_DISABLE, t, engine.rpm);
	}

	corr.air = t;


	// Interpolate throttle position
	// this makes it really smooth to install the tps
	// Should the tps be scaled 0..255 or 0..100% ?
	engine.tps = linear_interp(config.tps_low, config.tps_high, 0, 255, sensors[ADC_TPS]);

	// Do we want battery interpolation?
	// if the resistors in the voltagedivider are unprecise, or the vref is 
	// a little off, the batt_tmp in "calc_total_enrichment" will be wrong

}


/***************************************************************************/
/*** calc rpm                                                            ***/
/***************************************************************************/
void calc_rpm(void) {
	uint16_t rpm, rpmk, rpm_p;

	// make sure the two bytes are fetched atomically
	cli();
	rpm_p = engine.rpm_p;
	sei();

	rpmk = (config.rpmk[0] << 8) | config.rpmk[1];

	if (rpm_p) {  // make sure the engine is spinning

		// calculate the "highres" rpm
		// rpm_p is the number of 100 usec's in a tach-event
		// rpm_hr is the number of rotations in 6 sec (real RPM/10)
#ifdef NOCONF_HACK
		uint16_t divi;
		divi = 60000/INJ_PER_REV;
		engine.rpm_hr = divi / rpm_p; // assumes INJ_PER_REV event per rotation
#else 
		engine.rpm_hr = (rpmk * 10) / rpm_p;
#endif

		rpm = engine.rpm_hr / 10;

		if ((rpm >> 8) != 0)
			engine.rpm = 255; // 25.500 rpm should be enough for anybody (Bill Gates '81)
		else
			engine.rpm = rpm & 0xFF;
		
	} else {
#ifdef PWANAL
		engine.rpm = 1;
		engine.rpm_hr = 10;
#else
		engine.rpm = 0;
		engine.rpm_hr = 0;
#endif
	}
}


/***************************************************************************/
/*** cranking                                                            ***/
/***************************************************************************/
void cranking(void) {
	uint8_t pw;
	uint8_t my_status;

	cli();
	my_status = engine.status;
	my_status |= _BV(crank);
	my_status &= ~( _BV(startw) | _BV(warmup) );
	engine.status = my_status;
	sei();

	/* calculate crank enrichment */
	if (engine.tps < FLOOD_CLEAR) {
		// calculate pulsewidth [cwl..cwh] 
		// interpolate with the coolant temperature
		pw = linear_interp(0, 205, config.cwl, config.cwh, engine.coolant);
		// battery correction?
	} else { 
		pw = 0; // flood clear
	}
	
	cli();
	inj_port1.pwcalc = pw;
	inj_port2.pwcalc = pw;
	sei();
}


/***************************************************************************/
/*** primepulse                                                          ***/
/***************************************************************************/
void primepulse(void) {
	if (config.primep_cold || config.primep_warm) {
		// don't prime if the engine is flooded
		if (engine.tps < FLOOD_CLEAR) {
			uint8_t pw;
			pw = linear_interp(0, 205, config.primep_cold, config.primep_warm, engine.coolant);
			// do we want battery correction?

			inj_port1.pw = pw;
			inj_port2.pw = pw;
			inj_port1.pwrun = 0;
			inj_port2.pwrun = 0;
			inj_port1.status |= _BV(enabled) | _BV(scheduled);
			inj_port2.status |= _BV(enabled) | _BV(scheduled);
			engine.status |= _BV(running);
		}
	}
}

