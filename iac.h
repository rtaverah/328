
#ifndef IAC_H
#define IAC_H


#define SIMPLE_IAC
//#define COMPLICATED_IAC   // not recommended!

#ifdef SIMPLE_IAC
// defines for config.iac[]

#define STEP_SEQ          0   // define the stepping sequence (compensate for incorrect wiring)
#define CONF              1   // misc configuration options for stepper (stepper speed, power-off)
#define MAX_STEPS         2   // the maximal number of steps the iac can be extended
#define RESYNC            3   // set to 0 to disable resync. Any other number is in 'FINE_PERIOD' resolution
#define TPS_THRES         4   // tps adc-count for idle threshold
#define FAST_IDLE_TEMP    5   // switchpoint for cold/warm idle speed
#define COLD_RPM          6   // target rpm when temperature < FAST_IDLE_TEMP, [rpm/10]
#define WARM_RPM          7   // target rpm when temperature > FAST_IDLE_TEMP, [rpm/10]
#define COLD_START_POS    8   // starting (during cranking) position at -40F
#define WARM_START_POS    9   // starting (during cranking) position at 2000F

#define COARSE_DEV       10   // [rpm/10], ex: 23 <=> 230 rpm deviation
#define ADJUST_COARSE    11   // number of steps moved at each adjustment, packed:
															// Upper four bits for increasing of rpm, lower 4 bits for decreasing rpm
#define COARSE_PERIOD    12   // 1: 100ms, 2: 200ms, 3: 300ms, etc.

#define FINE_DEV         13   // [rpm/10], ex: 23 <=> 230 rpm deviation
#define ADJUST_FINE      14   // number of steps moved at each adjustment, packed:
															// Upper four bits for increasing of rpm, lower 4 bits for decreasing rpm
#define FINE_PERIOD      15   // 1: 100ms, 2: 200ms, 3: 300ms, etc.

/*
Suggested configuration:
step_seq : depends on how the stepper is wired up, read the docs
conf : no power off, speed depends on the stepper
max_steps : 100
resync : 15 (every 15 seconds)
tps_thres : depends on throttle pot
fast_idle_temp : 60C degress
cold_rpm : 100 (1000 rpm)
warm_rpm : 85 (850 rpm)
cold_start_pos : 50
warm_start_pos : 20
decel_open : 0
coarse_dev : 20 (+/- 200 rpm)
adjust_coarse : 0x43
coarse_period : 1
fine_dev : 3 (+/- 30 rpm)
adjust_fine : 0x21
fine_period : 10 (every second)
*/

#endif




#ifdef COMPLICATED_IAC
// defines for config.iac[]

#define STEP_SEQ          0   // define the stepping sequence (compensate for incorrect wiring)
#define CONF              1   // misc configuration options for stepper (stepper speed, power-off)
#define MAX_STEPS         2   // the maximal number of steps the iac can be extended
#define TPS_THRES         3   // tps adc-count for idle threshold
#define FAST_IDLE_TEMP    4   // switchpoint for cold/warm idle speed
#define COLD_RPM          5   // target rpm when temperature < FAST_IDLE_TEMP
#define WARM_RPM          6   // target rpm when temperature > FAST_IDLE_TEMP
#define COLD_START_POS    7   // starting (during cranking) position at -40F
#define WARM_START_POS    8   // starting (during cranking) position at 2000F
#define START_RPM         9   // right after cranking, this (rpm/10) is added to target_rpm such that the idle rpm temporarily is raised
#define START_COUNT      10   // the extra START_RPM is decreased in START_COUNT [sec/10]
#define PID_CONF         11   // parameters for the pid controller (asymmetric)
#define KP               12   // proportional multiplier
#define KI               13   // integral multiplier
#define KD               14   // differential multiplier
#define DEADBAND         15   // no movement of iac when |current-target_rpm| < deadband. [rpm/10]
#define INTEGRAL_LIMIT   16   // limit the integrator
#define TAU_INCREASE_RPM 17   // timedelay increasing rpm when engine is spinning too slow (keep this small)
#define TAU_DECREASE_RPM 18   // timedelay decreasing rpm when engine is spinning too fast

#endif




#define busy         0
#define direction    1  // 1 == open, 0 == close
#define sync_close   2  // moving to fully closed position
#define sync_open    3  // moving to start position
#define iac_batt_low 4
#define left_idle    5  // has been in idle-mode

struct iac_t {
	uint8_t status;        // uses the defines above
	uint8_t count;         // how many steps are requested
	uint8_t position;      // current position
	uint8_t start_pos;     // go to this position after sync
};


// the number of steps besides config.iac[MAX_STEPS], to close the iac at powerup
#define IAC_SYNC 10

// hysteresis for fastidle
#define FAST_IDLE_THRES 3



uint8_t idle_rpm;
uint8_t iac_pos, last_iac_pos;

#ifdef COMPLICATED_IAC
int16_t idle_integral;
int16_t idle_error;
int16_t idle_rpmdot;
#endif


void calc_iac_pos(void);
void init_iac(void);
void move_iac(void);
void set_iac_pos(uint8_t pos);
uint8_t next_step(uint8_t phase);

#endif
