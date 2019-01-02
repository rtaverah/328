
/*
	Generally all functions in this file should be optimized - 
	that is, hand coded avr-assembler.
	
	Especially the division routines should be optimized, since the avr is
	lacking a div-instruction. Check Atmel application note for at fast 
	assembler-division.
*/


#include "global.h"
#include "helpers.h"
#include <inttypes.h>
#include <stdlib.h>


/***************************************************************************/
/*** linear interpolation                                                ***/
/***************************************************************************/
uint8_t linear_interp(uint8_t x1, uint8_t x2, uint8_t y1, uint8_t y2, uint8_t x) {
	uint8_t x21, dx;
	div_t d;

	if (x <= x1)  // below/equal lower bound
		return y1;

	if (x >= x2)  // above/equal upper bound
		return y2;

	x21 = x2 - x1;
	if (x21 == 0) // arghh, can't divide by zero.
		return y1;

	dx = x - x1;

	if (y2 < y1) {  // negative slope
		d = div( (y1-y2)*dx, x21 );
		if (d.rem >= (x21/2))  // round result
			d.quot++;
		return y1 - d.quot;
	} else {        // positive slope
		d = div( (y2-y1)*dx, x21 );
		if (d.rem >= (x21/2))  // round result
			d.quot++;
		return y1 + d.quot;
	}
}


/***************************************************************************/
/*** search table                                                        ***/
/***************************************************************************/
// the array to be searched must be increasing (or equal)
void search_table(uint8_t *tbl, uint8_t tbl_length, uint8_t item, struct search_table_t *r) {
	uint8_t i, searching, index;

	searching = true;
	index = 0;

	for (i=0; (i<tbl_length) && searching; i++) {
		index = i;
		if (item < tbl[i]) {
			searching = false;
		}
	}

	if (index == 0) {
		r->lindex = r->uindex = 0;
	} else {
		r->lindex = index-1;
		r->uindex = index;
	}

/*
	if (r->index == 0) { // lbound and ubound can't point to the same element
		r->lbound = tbl[0];
		r->ubound = tbl[1];
		r->index = 1;
	} else {
		r->lbound = tbl[r->index-1];
		r->ubound = tbl[r->index];
	}
*/
}


/***************************************************************************/
/*** mult div100                                                         ***/
/***************************************************************************/
uint16_t mult_div100(uint8_t a, uint16_t b) {
	div_t d;

	// this takes ~260 cycles
	d = div(a*b, 100);

	if (d.rem >= 50)
		d.quot++;

	return d.quot;
}


/***************************************************************************/
/*** median filter                                                       ***/
/***************************************************************************/
// this is hardcoded for a three byte buffer
uint8_t median_filter(struct mfilter_t *f, uint8_t in) {
	uint8_t m1, m2;

	// shift all elements one place
	f->buf[2] = f->buf[1];
	f->buf[1] = f->buf[0];
	f->buf[0] = in;

	// find the largest of buf0 and buf1
	if (f->buf[0] > f->buf[1])
		m1 = f->buf[0];
	else
		m1 = f->buf[1];

	// find the largest of buf1 and buf2
	if (f->buf[1] > f->buf[2])
		m2 = f->buf[1];
	else
		m2 = f->buf[2];

	// return the smaller value
	if (m1 < m2)
		return m1;
	else
		return m2;

}


/***************************************************************************/
/*** limit                                                               ***/
/***************************************************************************/
int16_t limit_i16(int16_t val, int16_t upper, int16_t lower) {
	if (val > upper)
		return upper;
	else if (val < lower)
		return lower;
	else
		return val;
}

int8_t limit_i8(int8_t val, int8_t upper, int8_t lower) {
	if (val > upper)
		return upper;
	else if (val < lower)
		return lower;
	else
		return val;
}


/***************************************************************************/
/*** min                                                                 ***/
/***************************************************************************/
__inline__ uint8_t min(uint8_t a, uint8_t b) {
	if (a < b)
		return a;
	else
		return b;
}

/***************************************************************************/
/*** max                                                                 ***/
/***************************************************************************/
__inline__ uint8_t max(uint8_t a, uint8_t b) {
	if (a > b)
		return a;
	else
		return b;
}

