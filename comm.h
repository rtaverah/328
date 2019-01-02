
#ifndef COMM_H
#define COMM_H

#include <inttypes.h>

#define CMD_QUEUE_SIZE 32

struct cmd_queue_t {
	uint8_t (*func)(uint8_t i);
	uint8_t len;
	uint8_t count;
};

void initUART(void);
uint8_t comm(uint8_t data);

void MTstoreConfigVar(uint8_t addr, uint8_t value);
uint8_t MTsendRTvar(uint8_t i);
uint8_t MTsendConfigVar(uint8_t i);

uint8_t MTsendConfigVar_more(uint8_t addr);
uint8_t MTsendRTvar_more(uint8_t i);

uint8_t MTtestComm(uint8_t i);
uint8_t MTcodeVer(uint8_t i);

uint8_t sendConfigVar(uint8_t i);
uint8_t pushfunc(uint8_t (*f)(uint8_t i), uint8_t len);

uint8_t sendVE(uint8_t i);
uint8_t sendVEfraction(uint8_t i);
uint8_t sendVEref(uint8_t i);
void storeVE(uint8_t addr, uint8_t value);
void storeVEref(uint8_t addr, uint8_t value);

uint8_t sendIACstatus(uint8_t i);

/* the uart recieve buffer */
extern struct buffer_t comm_rx_buffer;

#endif
