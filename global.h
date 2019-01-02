
// Type declarations global to everything

#ifndef GLOBAL_H
#define GLOBAL_H

#include <inttypes.h>

/* special hardware is defined in my_make (see doc/)
*/

#ifdef PWANAL 
#define INJ_PER_REV 2
#endif

#ifndef true 
#define true 1
#endif

#ifndef false
#define false 0
#endif


/***********************************************************************/
//flags
#define tenth_second 0
#define ten_ms       1
#define second       2

struct time_t {
	uint8_t tick;  // ms /10
	uint16_t ms;   // ms
	uint8_t tsec;  // seconds /10
	uint16_t sec;  // seconds
	uint8_t flags; 
};

/***********************************************************************/

#define enabled   0 // squirt
#define scheduled 1 // sched
#define firing    2 // firing

struct squirt_t {
	uint8_t status;    // uses the defines above
	uint8_t pw;        // injector squirt time in 1/10 millesec (0 to 25.5 millisec)
	uint8_t pwrun;     // Pulsewidth timing variable - from 0 to 25.5ms
	uint8_t pwcalc;    // next pulsewidth
	uint8_t pwm_dc;    // pwm duty cycle (injpwm)
	uint8_t pwm_delay; // delay (0.1 ms) before going to pwm-mode (injpwmt)
};

/***********************************************************************/

//status
#define running 0  // engine running
#define crank   1  // cranking
#define startw  2  // start warmup enrichment
#define warmup  3  // warmup
#define tpsaen  4  // tps acceleration
#define tpsden  5  // tps deceleration
//#define mapaen  6  // map acceleration
#define idle    7  // idle mode

//status_ext
#define new_rpm          0  // need for recalculation of rpm
#define baro_problem     1  // somehow the barometric pressure is way off
#define o2_not_ready     2  // this is set for the first 30 seconds
#define o2_closed_loop   3  // enabled when o2 sensor is active
#define crank_enable     4  // allowed to enter crank mode
#define left_crankmode   5  // _has been_ in crank mode

struct engine_t {
	uint8_t status;      // uses the defines above
	uint8_t status_ext;  // uses the defines below the above!
	uint8_t kpa;         // MAP value in units of KPa
	uint8_t coolant;     // Coolant temperature in Degrees F plus 40 (allows -40 degress to fit in integer)
	uint8_t batt;        // current battery voltage
	uint8_t tps;         // throttle posision (percent)
	uint8_t last_tps;    // throttle posision (percent), last reading
	uint16_t rpm_p;      // rpm period
	uint16_t rpm_c;      // rpm counter
	uint8_t rpm;         // computed engine rpm/100
	uint16_t rpm_hr;     // computed highres engine rpm/10
};

/***********************************************************************/

#define O2_MAX_TPS 180  // WOT, disable closed loop
#define O2_MAX_KPA 110  // Turbo, disable closed loop

// define the adc sampling period [ms]
#define ADC_PERIOD  5

// PWM_FREQUENCY = 16MHz / 15.984kHz - 1
// with PWM_FREQUENCY = 1000, PWM frequency = 15.984kHz
#define PWM_FREQUENCY   1000

// x seconds after leaving crankmode we can't reenter (unless rpm drops to 0)
//#define CRANK_TIMEOUT 10

// tps threshold
#define FLOOD_CLEAR 155

// hysteresis for fastidle
#define FAST_IDLE_THRES 3

// seconds for ego to warmup
#define EGO_WARMUP 150

// the maximum interrupt filtering period
#define TACH_FILTER_MAX 200


uint8_t ms_version;
#define RV8_EMULATION 122

/***********************************************************************/
/* Configuration variables */
/***********************************************************************/

// some day the config-structure should be cleaned up!

// beware when changing the order of the variables
struct config_t {
	//+64 tag, don't remove - VE[64]
	uint8_t cwl;          // Crank Enrichment at -40 F
	uint8_t cwh;          // Crank Enrichment at 170 F
	uint8_t awev;         // After-start Warmup Percent enrichment add-on value
	uint8_t awc;          // After-start number of cycles
	uint8_t wwu[10];      // Warmup bins(fn temp)
	uint8_t tpsaq[4];     // TPS acceleration amount (fn TPSDOT) in 0.1 ms units
	uint8_t tpsacold;     // Cold acceleration amount (at -40 degrees) in 0.1 ms units
	uint8_t tps_thresh;   // Accel TPS DOT threshold
	uint8_t tpsasync;     // Acceleration clock value
	uint8_t tpsdq;        // Deacceleration fuel cut
	uint8_t egotemp;      // Coolant Temperature where EGO is active
	uint8_t egocountcmp;  // Counter value where EGO step is to occur
	uint8_t egodelta;     // EGO Percent step size for rich/lean
	uint8_t egolimit;     // Upper/Lower EGO rail limit (egocorr is inside 100 +/- Limit)
	uint8_t req_fuel;     // Fuel Constant
	uint8_t divider;      // IRQ divide factor for pulse
	uint8_t alternate;    // Alternate injector drivers
	uint8_t injopen;      // Injector Open Time
	uint8_t injocfuel;    // PW-correlated amount of fuel injected during injector open
	//START waste, backwards compatibility
	uint8_t injpwm;       // Injector PWM duty cycle at current limit
	uint8_t injpwmt;      // Injector PWM mmillisec time at which to activate.
	//END waste, backwards compatibility
	uint8_t battfac;      // Battery Gamma Factor
	uint8_t rpmk[2];      // Constant for RPM = 12,000/ncyl - downloaded constant
	//+8 tag, don't remove - rpmrangeve[8]
	//+8 tag, don't remove - kparangeve[8] 
	uint8_t config11;     // Configuration for PC Configurator
	uint8_t config12;     // Configuration for PC Configurator
	uint8_t config13;     // Configuration for PC Configurator
	//START waste, backwards compatibility
	uint8_t primep;       // Priming pulses (0.1 millisec units)
	//END waste, backwards compatibility
	uint8_t rpmoxlimit;   // Minimum RPM where O2 Closed Loop is Active
	uint8_t fastidle;     // Fast Idle Temperature
	uint8_t voltoxtarget; // O2 sensor flip target value
	uint8_t acmult;       // Acceleration cold multiplication factor (percent/100)
	uint8_t DO_NOT_USE_THIS_FOR_ANYTHING[4]; // MegaTune thrashes it, grrrr
	//END OF MEGASQUIRT COMPATIBILITY!
	//THE REST OF THE VARIABLES ARE VERY UNSTABLE, THEY COME AND GO AS THEY LIKE!

	uint8_t wwurange[10];   // WWURANGE has an offset of +40 F
	uint8_t tpsdotrate[4];  // tps acc bins

	uint8_t cranking_thres; // cranking threshold
	uint8_t fuelcut_thres;  // fuelcut enabled above threshold (rpm / 100)

	uint8_t primep_cold;    // priming pulse length at -40F
	uint8_t primep_warm;    // priming pulse length at 170F

	uint8_t tps_low;        // tps min adc-count
	uint8_t tps_high;       // tps max adc-count

	uint8_t baro;           // mean barometric pressure
	uint8_t dbaro;          // max difference in barometric pressure

	uint8_t fan_temp;       // start temperature for coolant fan
	uint8_t fan_hyst;       // hysteresis for coolant fan

	uint8_t master_debug;   // this allows all kinds of fun stuff!

	// IAC stuff
	uint8_t iac[29];  // only using the first 16 bytes (mik laziness)
	/*
	uint8_t iac_step_seq;        // define the stepping sequence (compensate for incorrect wiring)
	uint8_t iac_conf;            // misc configuration options for stepper (stepper speed, power-off)
	//  uint8_t iac_idlespeed[10];   // idle speed bins for temperature interpolation

	uint8_t iac_cold_rpm;
	uint8_t iac_warm_rpm;
	uint8_t iac_cold_start_offset;
	uint8_t iac_warm_start_offset;

	uint8_t iac_reg_limit;

	//  uint8_t iac_idle_offset[10]; // number of counts to open (starting point) at the speed defined in idlespeed
	//  uint8_t iac_sync;            // number of steps to close at power on (syncronize position to zero)

	uint8_t iac_start_rpm;           // how many rpm 'extra' just when the engine has started (rpm/10)
	uint8_t iac_start_count;          // 'iac_count' decreases from 'iac_start' towards zero in 'iac_warmup' sec/10
	//  uint8_t iac_freeze;          // 'iac_count' starts to be decremented after number of sec/10 have passed
	uint8_t iac_tps_thres;       // tps threshold for PID / throttle follower

	//  uint8_t iac_rpm_thres;   // PID control enabled only below idle_rpm + rpm/10

	uint8_t iac_integral_limit;  // limit for integral control
	//  uint8_t iac_max_tps_open;    // upper limit (count) for how much the throttle follower can open the iac
	uint8_t iac_deadband;
	//  uint8_t iac_tps_speed;   // limit the tps follower speed (steps / 10th ms)
	//  uint8_t iac_reg_limit;       // upper bound on number of counts/10ms
	uint8_t iac_kp;              // gain factor for the proportional part of PID controller
	uint8_t iac_kd;              // gain factor for the differential part of PID controller
	uint8_t iac_ki;              // gain factor for the integral part of PID controller
	uint8_t iac_pid_conf;        // configure the pid controller
	*/

	uint8_t fuelcut;
	uint8_t fuelresume;
};


/***********************************************************************/

//config11
#define MAP_SENSOR      0  // 0:mpx4115ap, 1:mpx4250ap
#define ENGINE_STROKE   2  // 0:4-stroke, 1:2-stroke
#define INJ_TYPE        3  // 0:port injection, 1:throttle body
//bit 7-4 defines no of cylinders

/***********************************************************************/

//config12
//bit 1-0 coolant sensor type
//bit 3-2 mat sensor type
//bit 7-4 number of injectors

/***********************************************************************/

//config13
#define ODDFIRE               0 // 0:normal, 1:odd-fire
#define O2_WB_SENSOR          1 // 0:narrowband, 1:diy-wb
#define CONTROL_STRATEGY      2 // 0:speed-density, 1:alpha-N
#define BARO_CORRECTION       3 // 0:off, 1:on
//bit 7-4 unsused

/***********************************************************************/
// master_debug
#define mat2map        0    // reroute the mat sensor value to the map sensor value


/***********************************************************************/
// iac_conf
#define power_off_iac  0    // power off stepper 1 step cycle after each move
#define low_power_off  1    // don't move iac if batt < 8V
#define test_iac       2    // enable stepper movement when engine stopped
#define disable_iac    3    // dont calculate any iac positions
//bit 7-4 defines speed:
// 0 (fastest, 0ms delay between steps)
// f (slowest, 15ms delay between steps)

/***********************************************************************/
//iac_pid_conf
#define pid_asymmetric 0    // asymmetric error
#define tf_disable     1    // disable throttle follower


/***********************************************************************/
/***********************************************************************/
/***********************************************************************/

/* Hardware definitions */

/* analog */

//Port: C
#define ADC_MAP		0 // manifold absolute pressure
#define ADC_CLT		1 // coolant
#define ADC_BATT	2 // 12V voltage measurement
#define ADC_EGO 	3 // O2 sensor
#define ADC_TPS 	4 // throttle position sensor
#define ADC_MAT		5 // manifold air temperature
#define ADC_BARO	6 // Barometer ADC Raw Reading - KPa (0 - 255)

#define ADC_CHANNELS 6 	// 6 is Max on ATmega328. THIS STARTS FROM 1, NOT 0.


/* digital */

//Port: A
#define LEDPORT			PORTD
#define LEDMASK			0x7
#define LED_STATUS		5 // active LOW!
#define LED_SQUIRT		6 //
#define LED_WARMUP		7
// not enough pins for more LEDs
//#define LED_MISC		4 // active HIGH!
//#define LED_ACCEL		5

extern volatile uint8_t ledstate;       // interrupt sets this, not port directly. port is written from userspace 
void setleds(void);

//Port: B
#define PWM_A       1 // injector port1	OSC1A
#define PWM_B       2 // injector port2	OSC1B
#define IDLEVALVE   3 // idle solenoid	OSC2A
#define FUELPUMP    4 // fuel pump
#define CLT_FAN     5 // coolant fan

// Port: D
#define FLYBACK_A	3
#define FLYBACK_B	4

/*
// not enough pins on ATmega328 for this
//Port: ?
#define STEP_A       0 // idle-stepper winding A
#define STEP_EN      1 // idle-stepper driver enable
#define STEP_C       2 // idle-stepper winding C
#define STEP_D       3 // idle-stepper winding D
#define STEP_B       4 // idle-stepper winding B
*/

#endif

