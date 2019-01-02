
#include <avr/io.h>
#include <avr/interrupt.h>
#include <inttypes.h>
#include "adc.h"
#include "global.h"

// sample and filter analog inputs

uint8_t adc_sel;
uint8_t sensors[7];
volatile extern struct config_t config;


// initialize the sensor array
void initSensors(void) {
	uint8_t i;

	for (i=0; i<ADC_CHANNELS; i++)
		sensors[i] = readADC(i);
}


void startADC(void) {
	// initilize mux, and set startbit

	adc_sel = 0;
	ADMUX = _BV(REFS0) |_BV(ADLAR) | (adc_sel & 0x07);
//#warning "what if the adc hasn't finished?"
	// note, a check is needed such that the adc isn't "overrun"
	ADCSRA = _BV(ADEN) |  _BV(ADSC) | _BV(ADIE) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);

}


/* poll adc until conversion is done */
/* used initially for getting the barometric pressure */
uint8_t readADC(uint8_t channel) {

	/* init adc */
	ADCSRA = _BV(ADEN) | _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);  // div-128 prescaler

	// left adjust result
	ADMUX = _BV(REFS0) | _BV(ADLAR) | (channel & 0x0f);
	ADCSRA |= _BV(ADSC);

	loop_until_bit_is_set(ADCSRA, ADIF);
	ADCSRA |= _BV(ADIF); // clear adif-bit

	// ADCH contains the upper 8 bits of the 10-bit result
	return ADCH;
}



/***************************************************************************/
/*** INTERRUPT: ADC conversion complete ***/
/***************************************************************************/
ISR(ADC_vect) {
	static uint8_t prev_sensor_reading[6];
	uint16_t tmp;
	
	prev_sensor_reading[adc_sel] = sensors[adc_sel];

	// ADCH contains the upper 8 bits of the 10-bit result
	tmp = prev_sensor_reading[adc_sel] + ADCH;
	sensors[adc_sel] = tmp >> 1; // 2-sample moving average filter
	// use a median filter instead, that would give nice readings

	// this is to make testing with a stimulator a lot easier
	if (config.master_debug & _BV(mat2map))
		if (adc_sel == ADC_MAP)
			sensors[ADC_MAP] = sensors[ADC_MAT];

	adc_sel++;

	if (adc_sel >= ADC_CHANNELS) { // done
		adc_sel = 0;
		ADCSRA &= ~_BV(ADIE); // disable interrupt
	} else {
		ADMUX = _BV(REFS0) | _BV(ADLAR) | (adc_sel & 0x07);
		ADCSRA |= _BV(ADSC); // start next conversion
	}
}
