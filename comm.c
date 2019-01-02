
/*
The uart receive interrupt places the byte in a circular buffer 
instead of calling the comm-function. The function to process the buffer
sits in the main loop and calls comm, passing it the buffer contents as
a parameter.

The letters of the non-standard comm. commands are a bit too random, 
fix it before it is too late!

Instead of the storeVE & storeVEref, a copy-VE-to-VEref command might 
be better (and discard the storeVEref)?

All this MegaTune compatibility are too messy. How about a statically 
initialized array consisting of pointers to the variables:

uint8_t *mt_realtime_var[3] = {(uint8_t *)&engine.status, 
															 (uint8_t *)&sensors[BARO], 
															 (uint8_t *)&sensors[MAP] };
...how can it be placed in PROGMEM?

*/


/*
	This file deserves a little explanation!

	Instead of creating a huge circular buffer for uart tx data, 
	a smaller circular buffer 'command_queue' are used instead. 
	This buffer contains the address and arguments to functions which 
	can fetch bytes from any place in the memory.

	eg. MegaTune sends an 'A' to request realtime variables:
	1) Uart rx-interrupt puts rx byte in circular buffer.
	2) The function to process the buffer sits in the main loop and calls
		comm, passing it the buffer contents (single uint8_t) as a parameter.
	2) comm pushes the address of sendRTvar and the number of 
		bytes to send (22) onto the command queue and enables tx-interrupt.
	3) tx-interrupt reads the command queue, executes the function and
		decrements the number of bytes to send (and stores it in the 
		command queue again). If number equals zero remove the function 
		from the queue and start over. If the queue is empty, then disable 
		interrupt.
*/



#include "global.h"
#include "comm.h"
#include "storage.h"
#include "actuators.h"
#include "ve.h"
#include "iac.h"
#include "adc.h"
#include "fuelcalc.h"
#include "buffer.h"

#include <avr/io.h>
#include <inttypes.h>
#include <avr/sfr_defs.h>
#include <avr/interrupt.h>

#ifdef RV8MT
extern uint8_t scantime, scantime_cnt;
extern uint8_t wwu_pos, tps_pos, iac_pos;
#endif


extern volatile uint8_t sensors[];
extern volatile struct squirt_t inj_port1, inj_port2;
extern volatile struct engine_t engine;
extern volatile struct config_t config;
extern volatile struct time_t rtc;
extern volatile struct iac_t iac;
extern struct corr_t corr;

extern struct ve_t ve;

extern uint8_t code_ver;

extern struct ve_at_t tune_me;
extern struct ve_stat_res_t res;

// command queue for output
struct cmd_queue_t cmd_queue[CMD_QUEUE_SIZE];
uint8_t cmd_queue_head, cmd_queue_tail;

// buffer for storing input
struct buffer_t comm_rx_buffer;

// We could enumerate instead, but ansi-C promotes enumerations to
// integers (16 bit), no need for that!
#define CMD    0
#define OFFSET 1
#define VALUE  2

/***************************************************************************/
/*** comm                                                                ***/
/***************************************************************************/
uint8_t comm(uint8_t rx_data) {
	static uint8_t comm_state = CMD;
	static uint8_t rx_offset = 0;
	static uint8_t rx_cmd = 0;

	switch (comm_state) {
	case OFFSET:
		rx_offset = rx_data;  // the offset for storing the new byte
		comm_state = VALUE;
		break;

	case VALUE:
		switch (rx_cmd) {
			case 'n':
				storeVE(rx_offset, rx_data);
				break;
			case 'N':
				storeVEref(rx_offset, rx_data);
				break;
			case 'W':
				MTstoreConfigVar(rx_offset, rx_data);
				break;
#ifdef RV8MT
			case 'w':
				// we don't use the w-command yet
				break;
#endif
			default:
				break;
		}
		comm_state = CMD; // back to normal mode
		break;

	default:
		switch (rx_data) {
		case 'A': // send all realtime values
			if (ms_version == RV8_EMULATION)
				pushfunc(&MTsendRTvar_more, 27);
			else
				pushfunc(&MTsendRTvar, 22);
			break;

		case 'B': // store to eeprom
			save_variables();
			break;

		case 'C': // test communications
			pushfunc(&MTtestComm, 1);
			break;

		case 'F': // flush transmit buffer
			// NOT STANDARD MEGASQUIRT COMMAND!
			UCSR0B &= ~_BV(UDRIE0);             // disable TX interrupts
			cmd_queue_head = cmd_queue_tail = 0;
			break;

		case 'i': // get current IAC position
			// NOT STANDARD MEGASQUIRT COMMAND!
			pushfunc(&sendIACstatus, 7);
			break;

		case 'L': // send VE table
			// NOT STANDARD MEGASQUIRT COMMAND!
			pushfunc(&sendVE, sizeof(ve.table)/2);  // ve.table is 16 bit
			break;

		case 'l': // send fractions from VE table
			// NOT STANDARD MEGASQUIRT COMMAND!
			pushfunc(&sendVEfraction, sizeof(ve.table)/2);  // ve.table is 16 bit
			break;

		case 'm': // send VEref table
			// NOT STANDARD MEGASQUIRT COMMAND!
			pushfunc(&sendVEref, sizeof(ve.ref_table));
			break;      

		case 'N': // write VE table
			// NOT STANDARD MEGASQUIRT COMMAND!
			comm_state = OFFSET;
			rx_cmd = 'N';
			break;      

		case 'n': // write VE reference table
			// NOT STANDARD MEGASQUIRT COMMAND!
			comm_state = OFFSET;
			rx_cmd = 'n';
			break;      

		case 'Q': // send code version
			pushfunc(&MTcodeVer, 1);
			break;

		case 'p': // reset iac pos
			iac.position = 0;
			break;

		case 's': //open iac 1 count
			// good for debugging
			cli();
			iac.count = 1;
			iac.status |= _BV(busy) | _BV(direction);
			sei();
			break;
		
		case 'S': //close iac 1 count
			// good for debugging
			cli();
			iac.count = 1;
			iac.status |= _BV(busy);
			iac.status &= ~_BV(direction);
			sei();
			break;

		case 'T': // send entire config structure, and ONLY config structure (not ve tables)
			// NOT STANDARD MEGASQUIRT COMMAND!
			pushfunc(&sendConfigVar, sizeof(struct config_t));
			break;

		case 'V': // send VE table and constants
			if (ms_version == RV8_EMULATION)
				pushfunc(&MTsendConfigVar, 0x80);
			else
				pushfunc(&MTsendConfigVar, 0x7d);
			break;

#ifdef RV8MT
		case 'v': // send VE table and constants
			pushfunc(&MTsendConfigVar_more, 0x80);
			break;
#endif

		case 'W': // receive new VE or constant at 'W'+<offset>+<newbyte>
			comm_state = OFFSET;
			rx_cmd = 'W';
			break;

#ifdef RV8MT
		case 'w': // receive new VE or constant at 'w'+<offset>+<newbyte>
			comm_state = OFFSET;
			rx_cmd = 'w';
			break;
#endif
		default:
			break;
		}
	}
	return 0;
}


/***************************************************************************/
/***************************************************************************/
/*** MegaTune compatibility functions                                    ***/
/***************************************************************************/
/***************************************************************************/


/***************************************************************************/
/*** codeVer                                                             ***/
/***************************************************************************/
// return codeversion
uint8_t MTcodeVer(uint8_t i) {
	//  return (code_ver);
	return ms_version;
}


/***************************************************************************/
/*** testComm                                                            ***/
/***************************************************************************/
// return low part of second-count
uint8_t MTtestComm(uint8_t i) {
	return (rtc.sec & 0xFF);
}


/***************************************************************************/
/*** sendRTvar                                                           ***/
/***************************************************************************/
// convert datastructures to megasquirt compatible format
uint8_t MTsendRTvar(uint8_t i) {
	uint8_t a, b;

	switch (i) {
	case 0: 
		return (uint8_t)rtc.sec;  //secl

	case 1:
		// each squirt_t has its own status field, merge these fields
		a = inj_port1.status;
		b = inj_port2.status;

		return ((a & 0x01) << 0) | ((a & 0x02) << 1) | ((a & 0x04) << 1) |
					 ((b & 0x01) << 1) | ((b & 0x02) << 3) | ((b & 0x04) << 3); //squirt

	case 2:  return engine.status;  //engine
	case 3:  return sensors[ADC_BARO];  //baro
	case 4:  return sensors[ADC_MAP];   //map
	case 5:  return sensors[ADC_MAT];   //mat
	case 6:  return sensors[ADC_CLT];   //clt
	case 7:  return sensors[ADC_TPS];   //tps
	case 8:  return sensors[ADC_BATT];  //batt
	case 9:  return sensors[ADC_EGO];   //ego
	case 10: return corr.ego;       //egocorr
	case 11: return corr.air;       //aircorr
	case 12: return corr.warm;      //warmcorr
	case 13: return engine.rpm;     //rpm
	case 14: return inj_port1.pw;   //pw
	case 15: return corr.tpsaccel;  //tpsaccel
	case 16: return corr.baro;      //barocorr
	case 17: return corr.gammae;    //gammae
	case 18: return corr.ve;        //vecurr
	case 19: return 0;              // blank spot 1
	case 20: return 0;              // blank spot 2
	case 21: return 0;              // blank spot 3

	default: return 0;
	}
}




uint8_t MTsendRTvar_more(uint8_t i) {
	uint8_t a, b;

	switch (i) {
	case 0: 
		return (uint8_t)rtc.sec;  //secl

	case 1:
		// each squirt_t has its own status field, merge these fields
		a = inj_port1.status;
		b = inj_port2.status;

		return ((a & 0x01) << 0) | ((a & 0x02) << 1) | ((a & 0x04) << 1) |
					 ((b & 0x01) << 1) | ((b & 0x02) << 3) | ((b & 0x04) << 3); //squirt

	case 2:  return engine.status;  //engine
	case 3:  return sensors[ADC_BARO];  //baro
	case 4:  return sensors[ADC_MAP];   //map
	case 5:  return sensors[ADC_MAT];   //mat
	case 6:  return sensors[ADC_CLT];   //clt
	case 7:  return sensors[ADC_TPS];   //tps
	case 8:  return sensors[ADC_BATT];  //batt
	case 9:  return sensors[ADC_EGO];   //ego
	case 10: return corr.ego;       //egocorr
	case 11: return corr.air;       //aircorr
	case 12: return corr.warm;      //warmcorr
	case 13: return engine.rpm;     //rpm
	case 14: return inj_port1.pw;   //pw
	case 15: return corr.tpsaccel;  //tpsaccel
	case 16: return corr.baro;      //barocorr
	case 17: return corr.gammae;    //gammae
	case 18: return corr.ve;        //vecurr
	case 19: return iac.position;   // "iac dutycycle"
	case 20: return (uint8_t)(engine.rpm_p >> 8);
	case 21: return (uint8_t)engine.rpm_p;
	case 22: return 0; //diagEngine; // not implemented
	//*CAA case 23: return scantime;
	//*CAA case 24: return wwu_pos;  // currently the same, iac speed depends on temperature as well
	//*CAA case 25: return wwu_pos;
	//*CAA case 26: return tps_pos;

	default: return 0;
	}
}




/***************************************************************************/
/*** MTstoreConfigVar                                                    ***/
/***************************************************************************/
void MTstoreConfigVar(uint8_t addr, uint8_t value) {

	if (addr < (sizeof(struct config_t)+64+8+8)) { // we DON'T want a loose pointer
		if (addr < 64) {
			// access the VE table

			// set the fraction to 127 (evenly hard increasing/decreasing VE value)
			ve.table[ (VE_SIZE_RPM/8)*(addr % 8) + (addr/8)*VE_SIZE_RPM ] = (value << 8) | 127;
			// might be a good idea to interpolate the neighbour points

			// write same value to VE reference table
			ve.ref_table[ (VE_SIZE_RPM/8)*(addr % 8) + (addr/8)*VE_SIZE_RPM ] = value;

		} else if ((addr >= 0x64) && (addr < 0x6C)) { // rpmrangeve[8]
			ve.rpm_range[addr - 0x64] = value;

		} else if ((addr >= 0x6C) && (addr < 0x74)) { // kparangeve[8]
			ve.kpa_range[addr - 0x6C] = value;

		} else {
			uint8_t my_addr;

			if (addr >= 0x74)
				my_addr = addr - 64 - 8 - 8; // sizeof(VE) - sizeof(rpmrangeve) - sizeof(kparangeve)
			else
				my_addr = addr - 64; // sizeof(VE)

			*( (uint8_t *)&config + my_addr ) = value;
			
			if (addr == 0x5F) { // copy injpwm to inj_portX also.
				inj_port1.pwm_dc = value;
				inj_port2.pwm_dc = value;
			} else if (addr == 0x60) { // copy injpwmt to inj_portX also
				inj_port1.pwm_delay = value;
				inj_port2.pwm_delay = value;
			}
		}
	} // else { 
		//   send a bugreport to the developers of the control program :)
		//  }
}

// sends the extended constants
uint8_t MTsendConfigVar_more(uint8_t addr) {
	uint16_t my_addr = addr + 128 - 64 - 8 - 8;

	if (my_addr < sizeof(struct config_t))
		return *( (uint8_t *)&config + my_addr );
	else 
		return 0;
}




/***************************************************************************/
/*** sendConfigVar                                                       ***/
/***************************************************************************/
uint8_t MTsendConfigVar(uint8_t addr) {
	if (addr < 64) {
		// MegaTune compatibility, the first 64 bytes are the VE table.
		// In the 16x16 table, we want every other point in both the rpm and kpa direction

		return ve.table[ (VE_SIZE_RPM/8)*(addr % 8) + (addr/8)*VE_SIZE_RPM ] >> 8;

	} else if ((addr >= 0x64) && (addr < 0x6C)) { // rpmrangeve[8]
		return ve.rpm_range[addr - 0x64];

	} else if ((addr >= 0x6C) && (addr < 0x74)) { // kparangeve[8]
		return ve.kpa_range[addr - 0x6C];

	} else {

		// if the size of the VE table are different from 64, then we must fix
		// some addresses (megatune compatibility)
		uint8_t my_addr;

		if (addr >= 0x74)
			my_addr = addr - 64 - 8 - 8; // sizeof(VE) - sizeof(rpmrangeve) - sizeof(kparangeve)
		else
			my_addr = addr - 64; // sizeof(VE)

		return *( (uint8_t *)&config + my_addr );
	}
}



/***************************************************************************/
/***************************************************************************/
/*** End of MegaTune compatibility functions                             ***/
/***************************************************************************/
/***************************************************************************/


/***************************************************************************/
/*** sendIACstatus                                                       ***/
/***************************************************************************/
uint8_t sendIACstatus(uint8_t i) {
	/*  static uint16_t pos, dest, rpm;

	switch (i) {
	case 0: // take a snapshot of everything
		pos = iac.position;
		//    dest = iac.dest;
		rpm = engine.rpm_hr;

		return iac.status;
	case 1:
		return pos >> 8;
	case 2: 
		return pos;
	case 3: 
		return dest >> 8;
	case 4: 
		return dest;
	case 5: 
		return rpm >> 8;
	case 6: 
		return rpm;
	default: 
		return 0;
		}*/

	return 0;
}


/***************************************************************************/
/*** sendConfigVar                                                       ***/
/***************************************************************************/
uint8_t sendConfigVar(uint8_t i) {
	if (i < sizeof(struct config_t))
		return *( (uint8_t *)&config + i );
	else 
		return 0;
}





// these functions are for the 16x16 VE tables

/***************************************************************************/
/*** sendVE                                                              ***/
/***************************************************************************/
uint8_t sendVE(uint8_t addr) {
	return ve.table[addr] >> 8;
}

/***************************************************************************/
/*** sendVEfraction                                                      ***/
/***************************************************************************/
uint8_t sendVEfraction(uint8_t addr) {
	return ve.table[addr];
}

/***************************************************************************/
/*** sendVEref                                                           ***/
/***************************************************************************/
uint8_t sendVEref(uint8_t addr) {
	return ve.ref_table[addr];
}

/***************************************************************************/
/*** storeVE                                                             ***/
/***************************************************************************/
void storeVE(uint8_t addr, uint8_t value) {
	if (addr < sizeof(ve.table)) {
		ve.table[addr] = (value << 8) | 127;
	}
}

/***************************************************************************/
/*** storeVEref                                                          ***/
/***************************************************************************/
void storeVEref(uint8_t addr, uint8_t value) {
	if (addr < sizeof(ve.table)) {
		ve.ref_table[addr] = value;
	}
}


/***************************************************************************/
/*** pushfunc                                                            ***/
/***************************************************************************/
uint8_t pushfunc(uint8_t (*f)(uint8_t i), uint8_t len) {
	uint8_t next_head;

	next_head = (cmd_queue_head + 1) % CMD_QUEUE_SIZE;
	if (next_head == cmd_queue_tail)
		return -1; // whoops, queue is full

	/* len starts at 0 */
	if (len > 0) // this looks like a hack - it is!
		len = len - 1;

	cmd_queue[cmd_queue_head].func = f;
	cmd_queue[cmd_queue_head].len = len;
	cmd_queue[cmd_queue_head].count = 0;

	cmd_queue_head = next_head;
	
	UCSR0B |= _BV(UDRIE0); // enable transmit interrupt
	return 0;
}



/***************************************************************************/
/*** initUART                                                            ***/
/***************************************************************************/
void initUART(void) {

	/* rx enable, tx enable */
	//  UCSR0B = _BV(RXCIE0) | _BV(UDRIE0) | _BV(RXEN0) | _BV(TXEN0);
	UCSR0B = _BV(RXCIE0) | _BV(RXEN0) | _BV(TXEN0);
	/* 8 databit, 1 stopbit */
	UCSR0C = _BV(UCSZ01) | _BV(UCSZ00);

	/* 9600 baud @Å†16MHz */
	UBRR0H = 0;
	UBRR0L = 103;

	cmd_queue_head = cmd_queue_tail = 0;
	
	/* initialize recieve buffer */
	buffer_init(&comm_rx_buffer);
}


/***************************************************************************/
/*** INTERRUPT: UART rx complete ***/
/***************************************************************************/
ISR(USART_RX_vect) {

	// check status register for frame or overrun errors
	if ( UCSR0A & (_BV(FE0) | _BV(DOR0)) ) { 
		uint8_t dummy;
		dummy = UDR0; // just read uart and throw away the result
		return;
	} else {
		// put UDR0 in buffer here, process it from main loop
		buffer_append(&comm_rx_buffer, UDR0);
		//comm(UDR0);
	}
}


/***************************************************************************/
/*** INTERRUPT: UART tx empty ***/
/***************************************************************************/
ISR(USART_UDRE_vect) {

	//call function and get data to send
	UDR0 = (*cmd_queue[cmd_queue_tail].func)(cmd_queue[cmd_queue_tail].count);

	if (cmd_queue[cmd_queue_tail].count < cmd_queue[cmd_queue_tail].len) {
		cmd_queue[cmd_queue_tail].count++;
	} else {
		// we're done with this function

		cmd_queue_tail = (cmd_queue_tail + 1) % CMD_QUEUE_SIZE;
		if (cmd_queue_head == cmd_queue_tail) //buffer empty
			UCSR0B &= ~_BV(UDRIE0);             // disable TX interrupts
	}
}


