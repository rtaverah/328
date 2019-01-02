
#ifndef FUELCALC_H
#define FUELCALC_H

#include <inttypes.h>

struct corr_t {
	uint8_t ego;       // Oxygen Sensor Correction
	uint8_t air;       // Air Density Correction lookup
	uint8_t warm;      // Total Warmup Correction
	uint8_t baro;      // Barometer Lookup Correction
	uint8_t gammae;    // Total Gamma Enrichments
	uint8_t ve;        // Current VE value from lookup table
	uint8_t tpsaccel;  // Acceleration enrichment
	uint8_t tpsfuelcut; // fuelcut
	uint8_t tpsfuelcut2; // fuelcut
};

#define AIRDEN_DISABLE 98

void init_fuelcalc(void);
void calc_parameters(void);
void warmup_enrich(void);
void tps_acc_enrich(void);
void closed_tps_decel(void);
void o2(void);
void calc_total_enrichment(void);

void calc_rpm(void);
void cranking(void);
void primepulse(void);

#endif
