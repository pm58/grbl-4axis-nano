

#ifndef config_h
#define config_h
#include "grbl.h" // For Arduino IDE compatibility.


// Default settings. Used when resetting EEPROM. Change to desired name in defaults.h
#define DEFAULTS_GENERIC

// Serial baud rate
#define BAUD_RATE 115200


#define CPU_MAP_ATMEGA328P // Arduino Uno CPU


#define CMD_STATUS_REPORT '?'
#define CMD_FEED_HOLD '!'
#define CMD_CYCLE_START '~'
#define CMD_RESET 0x18 // ctrl-x.
#define CMD_SAFETY_DOOR '@'


#define HOMING_INIT_LOCK // Comment to disable


#define HOMING_CYCLE_0 (1<<Z_AXIS)                // REQUIRED: First move Z to clear workspace.
#define HOMING_CYCLE_1 ((1<<Y_AXIS) | (1<<X_AXIS))  // OPTIONAL: Then move X,Y at the same time.
//#define HOMING_CYCLE_2 ((1<<X_AXIS))                         // OPTIONAL: Uncomment and add axes mask to enable


#define N_HOMING_LOCATE_CYCLE 1 // Integer (1-128)


#define N_STARTUP_LINE 2 // Integer (1-2)


#define N_DECIMAL_COORDVALUE_INCH 4 // Coordinate or position value in inches
#define N_DECIMAL_COORDVALUE_MM   3 // Coordinate or position value in mm
#define N_DECIMAL_RATEVALUE_INCH  1 // Rate or velocity value in in/min
#define N_DECIMAL_RATEVALUE_MM    0 // Rate or velocity value in mm/min
#define N_DECIMAL_SETTINGVALUE    3 // Decimals for floating point setting values

#define MESSAGE_PROBE_COORDINATES // Enabled by default. Comment to disable.
 
#define SAFETY_DOOR_SPINDLE_DELAY 4000
#define SAFETY_DOOR_COOLANT_DELAY 1000

 #define INVERT_SPINDLE_ENABLE_PIN // Default disabled. Uncomment to enable.


#define REPORT_CONTROL_PIN_STATE // Default disabled. Uncomment to enable.


#define ACCELERATION_TICKS_PER_SECOND 100 

#define ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING  // Default enabled. Comment to disable.

#define TOOL_LENGTH_OFFSET_AXIS Z_AXIS // Default z-axis. Valid values are X_AXIS, Y_AXIS, or Z_AXIS.


#define SPINDLE_MAX_RPM 1000.0 // Max spindle RPM. This value is equal to 100% duty cycle on the PWM.
#define SPINDLE_MIN_RPM 0.0    // Min spindle RPM. This value is equal to (1/256) duty cycle on the PWM.


#define MINIMUM_JUNCTION_SPEED 0.0 // (mm/min)


#define MINIMUM_FEED_RATE 1.0 // (mm/min)

#define N_ARC_CORRECTION 12 // Integer (1-255)


#define ARC_ANGULAR_TRAVEL_EPSILON 5E-7 // Float (radians)


#define DWELL_TIME_STEP 50 // Integer (1-255) (milliseconds)

 #define BLOCK_BUFFER_SIZE 15  // Uncomment to override default in planner.h.


#define ENABLE_SOFTWARE_DEBOUNCE // Default disabled. Uncomment to enable.


#ifndef HOMING_CYCLE_0
  #error "Required HOMING_CYCLE_0 not defined."
#endif

#if defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && !defined(VARIABLE_SPINDLE)
  #error "USE_SPINDLE_DIR_AS_ENABLE_PIN may only be used with VARIABLE_SPINDLE enabled"
#endif

#if defined(USE_SPINDLE_DIR_AS_ENABLE_PIN) && !defined(CPU_MAP_ATMEGA328P)
  #error "USE_SPINDLE_DIR_AS_ENABLE_PIN may only be used with a 328p processor"
#endif

// ---------------------------------------------------------------------------------------


#endif
