
#include "global.h"
#include "actuators.h"
#include "helpers.h"

#include <inttypes.h>
#include <avr/io.h>
#include <stdlib.h>
#include <avr/interrupt.h>


volatile extern struct engine_t engine;
volatile extern uint8_t sensors[];
extern struct config_t config;



/***************************************************************************/
/*** check fast idle                                                     ***/
/***************************************************************************/
void fast_idle_valve(void) {

	// ths FAST_IDLE_THRES avoids erratic idle when coolant == fastidle
	if (engine.coolant < (config.fastidle - FAST_IDLE_THRES)) {
		PORTB |= _BV(IDLEVALVE); //idlevalve on
	} else if (engine.coolant >= config.fastidle) {
		PORTB &= ~_BV(IDLEVALVE); //idlevalve off
	}
}





/***************************************************************************/
/*** coolant fan                                                         ***/
/***************************************************************************/
void coolant_fan(void) {
	if (engine.coolant > config.fan_temp)
		PORTB |= _BV(CLT_FAN); // start fan
	else if (engine.coolant <= (config.fan_temp - config.fan_hyst))
		PORTB &= ~_BV(CLT_FAN); // stop fan
}


