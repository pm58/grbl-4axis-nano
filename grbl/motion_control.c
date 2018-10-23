

#include "grbl.h"


#ifdef USE_LINE_NUMBERS
  void mc_line(float *target, float feed_rate, uint8_t invert_feed_rate, int32_t line_number)
#else
  void mc_line(float *target, float feed_rate, uint8_t invert_feed_rate)
#endif
{
  // If enabled, check for soft limit violations. Placed here all line motions are picked up
  // from everywhere in Grbl.
  if (bit_istrue(settings.flags,BITFLAG_SOFT_LIMIT_ENABLE)) { limits_soft_check(target); }    
      
  // If in check gcode mode, prevent motion by blocking planner. Soft limits still work.
  if (sys.state == STATE_CHECK_MODE) { return; }
    
    do {
    protocol_execute_realtime(); // Check for any run-time commands
    if (sys.abort) { return; } // Bail, if system abort.
    if ( plan_check_full_buffer() ) { protocol_auto_cycle_start(); } // Auto-cycle start when buffer is full.
    else { break; }
  } while (1);

  // Plan and queue motion into planner buffer
  #ifdef USE_LINE_NUMBERS
    plan_buffer_line(target, feed_rate, invert_feed_rate, line_number);
  #else
    plan_buffer_line(target, feed_rate, invert_feed_rate);
  #endif
}


#ifdef USE_LINE_NUMBERS
  void mc_arc(float *position, float *target, float *offset, float radius, float feed_rate, 
    uint8_t invert_feed_rate, uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc, int32_t line_number)
#else
  void mc_arc(float *position, float *target, float *offset, float radius, float feed_rate,
    uint8_t invert_feed_rate, uint8_t axis_0, uint8_t axis_1, uint8_t axis_linear, uint8_t is_clockwise_arc)
#endif
{
  float center_axis0 = position[axis_0] + offset[axis_0];
  float center_axis1 = position[axis_1] + offset[axis_1];
  float r_axis0 = -offset[axis_0];  // Radius vector from center to current location
  float r_axis1 = -offset[axis_1];
  float rt_axis0 = target[axis_0] - center_axis0;
  float rt_axis1 = target[axis_1] - center_axis1;
  
  // CCW angle between position and target from circle center. Only one atan2() trig computation required.
  float angular_travel = atan2(r_axis0*rt_axis1-r_axis1*rt_axis0, r_axis0*rt_axis0+r_axis1*rt_axis1);
  if (is_clockwise_arc) { // Correct atan2 output per direction
    if (angular_travel >= -ARC_ANGULAR_TRAVEL_EPSILON) { angular_travel -= 2*M_PI; }
  } else {
    if (angular_travel <= ARC_ANGULAR_TRAVEL_EPSILON) { angular_travel += 2*M_PI; }
  }

 
  uint16_t segments = floor(fabs(0.5*angular_travel*radius)/
                          sqrt(settings.arc_tolerance*(2*radius - settings.arc_tolerance)) );
  
  if (segments) { 
    // Multiply inverse feed_rate to compensate for the fact that this movement is approximated
    // by a number of discrete segments. The inverse feed_rate should be correct for the sum of 
    // all segments.
    if (invert_feed_rate) { feed_rate *= segments; }
   
    float theta_per_segment = angular_travel/segments;
    float linear_per_segment = (target[axis_linear] - position[axis_linear])/segments;
    // Computes: cos_T = 1 - theta_per_segment^2/2, sin_T = theta_per_segment - theta_per_segment^3/6) in ~52usec
    float cos_T = 2.0 - theta_per_segment*theta_per_segment;
    float sin_T = theta_per_segment*0.16666667*(cos_T + 4.0);
    cos_T *= 0.5;

    float sin_Ti;
    float cos_Ti;
    float r_axisi;
    uint16_t i;
    uint8_t count = 0;
  
    for (i = 1; i<segments; i++) { // Increment (segments-1).
      
      if (count < N_ARC_CORRECTION) {
        // Apply vector rotation matrix. ~40 usec
        r_axisi = r_axis0*sin_T + r_axis1*cos_T;
        r_axis0 = r_axis0*cos_T - r_axis1*sin_T;
        r_axis1 = r_axisi;
        count++;
      } else {      
        // Arc correction to radius vector. Computed only every N_ARC_CORRECTION increments. ~375 usec
        // Compute exact location by applying transformation matrix from initial radius vector(=-offset).
        cos_Ti = cos(i*theta_per_segment);
        sin_Ti = sin(i*theta_per_segment);
        r_axis0 = -offset[axis_0]*cos_Ti + offset[axis_1]*sin_Ti;
        r_axis1 = -offset[axis_0]*sin_Ti - offset[axis_1]*cos_Ti;
        count = 0;
      }
  
      // Update arc_target location
      position[axis_0] = center_axis0 + r_axis0;
      position[axis_1] = center_axis1 + r_axis1;
      position[axis_linear] += linear_per_segment;
      
      #ifdef USE_LINE_NUMBERS
        mc_line(position, feed_rate, invert_feed_rate, line_number);
      #else
        mc_line(position, feed_rate, invert_feed_rate);
      #endif
      
      // Bail mid-circle on system abort. Runtime command check already performed by mc_line.
      if (sys.abort) { return; }
    }
  }
  // Ensure last segment arrives at target location.
  #ifdef USE_LINE_NUMBERS
    mc_line(target, feed_rate, invert_feed_rate, line_number);
  #else
    mc_line(target, feed_rate, invert_feed_rate);
  #endif
}


// Execute dwell in seconds.
void mc_dwell(float seconds) 
{
   if (sys.state == STATE_CHECK_MODE) { return; }
   
   uint16_t i = floor(1000/DWELL_TIME_STEP*seconds);
   protocol_buffer_synchronize();
   delay_ms(floor(1000*seconds-i*DWELL_TIME_STEP)); // Delay millisecond remainder.
   while (i-- > 0) {
     // NOTE: Check and execute realtime commands during dwell every <= DWELL_TIME_STEP milliseconds.
     protocol_execute_realtime();
     if (sys.abort) { return; }
     _delay_ms(DWELL_TIME_STEP); // Delay DWELL_TIME_STEP increment
   }
}


void mc_homing_cycle()
{
  
  #ifdef LIMITS_TWO_SWITCHES_ON_AXES  
    if (limits_get_state()) { 
      mc_reset(); // Issue system reset and ensure spindle and coolant are shutdown.
      bit_true_atomic(sys_rt_exec_alarm, (EXEC_ALARM_HARD_LIMIT|EXEC_CRITICAL_EVENT));
      return;
    }
  #endif
   
  limits_disable(); // Disable hard limits pin change register for cycle duration
  
  limits_go_home(HOMING_CYCLE_0);  // Homing cycle 0

  #ifdef HOMING_CYCLE_1
    limits_go_home(HOMING_CYCLE_1);  // Homing cycle 1
  #endif
  #ifdef HOMING_CYCLE_2
    limits_go_home(HOMING_CYCLE_2);  // Homing cycle 2
  #endif
    
  protocol_execute_realtime(); // Check for reset and set system abort.
  if (sys.abort) { return; } // Did not complete. Alarm state set by mc_alarm.

  gc_sync_position();

  // If hard limits feature enabled, re-enable hard limits pin change register after homing cycle.
  limits_init();
}


// Perform tool length probe cycle. Requires probe switch.
// NOTE: Upon probe failure, the program will be stopped and placed into ALARM state.
#ifdef USE_LINE_NUMBERS
  void mc_probe_cycle(float *target, float feed_rate, uint8_t invert_feed_rate, uint8_t is_probe_away, 
    uint8_t is_no_error, int32_t line_number)
#else
  void mc_probe_cycle(float *target, float feed_rate, uint8_t invert_feed_rate, uint8_t is_probe_away,
    uint8_t is_no_error)
#endif
{ 
  // TODO: Need to update this cycle so it obeys a non-auto cycle start.
  if (sys.state == STATE_CHECK_MODE) { return; }

  // Finish all queued commands and empty planner buffer before starting probe cycle.
  protocol_buffer_synchronize();

  // Initialize probing control variables
  sys.probe_succeeded = false; // Re-initialize probe history before beginning cycle.  
  probe_configure_invert_mask(is_probe_away);
  
  // After syncing, check if probe is already triggered. If so, halt and issue alarm.
  // NOTE: This probe initialization error applies to all probing cycles.
  if ( probe_get_state() ) { // Check probe pin state.
    bit_true_atomic(sys_rt_exec_alarm, EXEC_ALARM_PROBE_FAIL);
    protocol_execute_realtime();
  }
  if (sys.abort) { return; } // Return if system reset has been issued.

  // Setup and queue probing motion. Auto cycle-start should not start the cycle.
  #ifdef USE_LINE_NUMBERS
    mc_line(target, feed_rate, invert_feed_rate, line_number);
  #else
    mc_line(target, feed_rate, invert_feed_rate);
  #endif
  
  // Activate the probing state monitor in the stepper module.
  sys_probe_state = PROBE_ACTIVE;

  // Perform probing cycle. Wait here until probe is triggered or motion completes.
  bit_true_atomic(sys_rt_exec_state, EXEC_CYCLE_START);
  do {
    protocol_execute_realtime(); 
    if (sys.abort) { return; } // Check for system abort
  } while (sys.state != STATE_IDLE);
  
  // Probing cycle complete!
  
  // Set state variables and error out, if the probe failed and cycle with error is enabled.
  if (sys_probe_state == PROBE_ACTIVE) {
    if (is_no_error) { memcpy(sys.probe_position, sys.position, sizeof(float)*N_AXIS); }
    else { bit_true_atomic(sys_rt_exec_alarm, EXEC_ALARM_PROBE_FAIL); }
  } else { 
    sys.probe_succeeded = true; // Indicate to system the probing cycle completed successfully.
  }
  sys_probe_state = PROBE_OFF; // Ensure probe state monitor is disabled.
  protocol_execute_realtime();   // Check and execute run-time commands
  if (sys.abort) { return; } // Check for system abort

  // Reset the stepper and planner buffers to remove the remainder of the probe motion.
  st_reset(); // Reest step segment buffer.
  plan_reset(); // Reset planner buffer. Zero planner positions. Ensure probing motion is cleared.
  plan_sync_position(); // Sync planner position to current machine position.

  // TODO: Update the g-code parser code to not require this target calculation but uses a gc_sync_position() call.
  // NOTE: The target[] variable updated here will be sent back and synced with the g-code parser.
  system_convert_array_steps_to_mpos(target, sys.position);

  #ifdef MESSAGE_PROBE_COORDINATES
    // All done! Output the probe position as message.
    report_probe_parameters();
  #endif
}

void mc_reset()
{
  // Only this function can set the system reset. Helps prevent multiple kill calls.
  if (bit_isfalse(sys_rt_exec_state, EXEC_RESET)) {
    bit_true_atomic(sys_rt_exec_state, EXEC_RESET);

    // Kill spindle and coolant.   
    spindle_stop();
    coolant_stop();

    
    if ((sys.state & (STATE_CYCLE | STATE_HOMING)) || (sys.suspend == SUSPEND_ENABLE_HOLD)) {
      if (sys.state == STATE_HOMING) { bit_true_atomic(sys_rt_exec_alarm, EXEC_ALARM_HOMING_FAIL); }
      else { bit_true_atomic(sys_rt_exec_alarm, EXEC_ALARM_ABORT_CYCLE); }
      st_go_idle(); // Force kill steppers. Position has likely been lost.
    }
  }
}
