
#ifndef __STORAGE_H__
#define __STORAGE_H__

#include <inttypes.h>

#define SEEPROM __attribute__((section(".eeprom")))


enum {BEGIN_STORING, STORE_CONFIG, STORE_VE, DONE};


volatile uint8_t eeprom_flags;

#define busy 0
#define start 1
#define active 2



uint8_t state;
uint8_t *p_eeprom;
uint8_t *p_sram;
uint16_t length;
uint16_t offset;



void init_storage(void);
uint8_t save_variables(void);
void store_control(void);
uint8_t store_setup(void);
uint8_t store_bytes(void);
void load_variables(void);


#endif
