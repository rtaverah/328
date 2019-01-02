
#ifndef VE_H
#define VE_H

// how many entries in the VE-table
#define VE_SIZE_RPM 8
#define VE_SIZE_KPA 8


struct ve_t {
	uint16_t table[VE_SIZE_RPM*VE_SIZE_KPA];     // main VE table (modified by VE learning) 
	uint8_t ref_table[VE_SIZE_RPM*VE_SIZE_KPA];  // reference VE table
	uint8_t rpm_range[VE_SIZE_RPM];  // VE table RPM bins for 2-D interpolation
	uint8_t kpa_range[VE_SIZE_KPA];  // VE table MAP bins for 2_D interpolation
};


void ve_table_lookup(void);

#endif
