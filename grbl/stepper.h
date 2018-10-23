

#ifndef stepper_h
#define stepper_h 

#ifndef SEGMENT_BUFFER_SIZE
  #define SEGMENT_BUFFER_SIZE 6
#endif

// Initialize and setup the stepper motor subsystem
void stepper_init();

// Enable steppers, but cycle does not start unless called by motion control or realtime command.
void st_wake_up();

// Immediately disables steppers
void st_go_idle();

// Generate the step and direction port invert masks.
void st_generate_step_dir_invert_masks();

// Reset the stepper subsystem variables       
void st_reset();
             
// Reloads step segment buffer. Called continuously by realtime execution system.
void st_prep_buffer();

// Called by planner_recalculate() when the executing block is updated by the new plan.
void st_update_plan_block_parameters();

// Called by realtime status reporting if realtime rate reporting is enabled in config.h.
#ifdef REPORT_REALTIME_RATE
float st_get_realtime_rate();
#endif

#endif
