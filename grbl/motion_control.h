

#ifndef motion_control_h
#define motion_control_h


#define HOMING_CYCLE_LINE_NUMBER -1


#ifdef USE_LINE_NUMBERS
void mc_line(float *target, float feed_rate, uint8_t invert_feed_rate, int32_t line_number);
#else
void mc_line(float *target, float feed_rate, uint8_t invert_feed_rate);
#endif


#ifdef USE_LINE_NUMBERS
void mc_arc(float *position, float *target, float *offset, float radius, float feed_rate, 
  uint8_t invert_feed_rate, uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc, int32_t line_number);
#else
void mc_arc(float *position, float *target, float *offset, float radius, float feed_rate,
  uint8_t invert_feed_rate, uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc);
#endif
  
// Dwell for a specific number of seconds
void mc_dwell(float seconds);

// Perform homing cycle to locate machine zero. Requires limit switches.
void mc_homing_cycle();

// Perform tool length probe cycle. Requires probe switch.
#ifdef USE_LINE_NUMBERS
void mc_probe_cycle(float *target, float feed_rate, uint8_t invert_feed_rate, uint8_t is_probe_away,
  uint8_t is_no_error, int32_t line_number);
#else
void mc_probe_cycle(float *target, float feed_rate, uint8_t invert_feed_rate, uint8_t is_probe_away,
  uint8_t is_no_error);
#endif

// Performs system reset. If in motion state, kills all motion and sets system alarm.
void mc_reset();

#endif
