
#ifndef __HELPERS_H__
#define __HELPERS_H__

#include "global.h"
#include <inttypes.h>

// argument to search_table function
struct search_table_t {
	uint8_t lindex;
	uint8_t uindex;
};

struct mfilter_t {
	uint8_t buf[3];
};

uint8_t linear_interp(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2, uint8_t x);
void search_table(uint8_t *tbl, uint8_t tbl_length, uint8_t item, struct search_table_t *r);
uint16_t mult_div100(uint8_t a, uint16_t b);
uint8_t median_filter(struct mfilter_t *f, uint8_t in);
int16_t limit_i16(int16_t val, int16_t upper, int16_t lower);
int8_t limit_i8(int8_t val, int8_t upper, int8_t lower);
__inline__ uint8_t min(uint8_t a, uint8_t b);
__inline__ uint8_t max(uint8_t a, uint8_t b);


#endif
