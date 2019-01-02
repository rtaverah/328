
#include "global.h"
#include "helpers.h"
#include "ve.h"
#include "fuelcalc.h"

#include <inttypes.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <string.h>


volatile extern struct engine_t engine;
extern struct config_t config;
extern struct corr_t corr;
extern struct ve_t ve;


/***************************************************************************/
/*** VE table lookup                                                     ***/
/***************************************************************************/
void ve_table_lookup(void) {
	uint8_t ve_11, ve_12, ve_21, ve_22;
	uint8_t ve_low_kpa, ve_high_kpa;
	struct search_table_t kpa, rpm;

	if (config.config13 & _BV(CONTROL_STRATEGY)) { //Alpha-N
		engine.kpa = engine.tps;
	}

	search_table(ve.kpa_range, sizeof(ve.kpa_range), engine.kpa, &kpa);
	search_table(ve.rpm_range, sizeof(ve.rpm_range), engine.rpm, &rpm);

	// currently we ignore the fraction-part of the VE value
	ve_11 = ve.table[ VE_SIZE_RPM * (kpa.lindex) + (rpm.lindex)] >> 8;
	ve_12 = ve.table[ VE_SIZE_RPM * (kpa.lindex) +  rpm.uindex]    >> 8;
	ve_21 = ve.table[ VE_SIZE_RPM * kpa.uindex     + (rpm.lindex)] >> 8;
	ve_22 = ve.table[ VE_SIZE_RPM * kpa.uindex     +  rpm.uindex]    >> 8;

	ve_low_kpa = linear_interp(ve.rpm_range[rpm.lindex],
					 ve.rpm_range[rpm.uindex],
					 ve_11, ve_12, engine.rpm);

	ve_high_kpa = linear_interp(ve.rpm_range[rpm.lindex],
						ve.rpm_range[rpm.uindex],
						ve_21, ve_22, engine.rpm);

	corr.ve = linear_interp(ve.kpa_range[kpa.lindex],
				ve.kpa_range[kpa.uindex],
				ve_low_kpa, ve_high_kpa, engine.kpa);
}

