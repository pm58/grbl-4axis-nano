
#ifndef planner_h
#define planner_h


// The number of linear motions that can be in the plan at any give time
#ifndef BLOCK_BUFFER_SIZE
  #ifdef USE_LINE_NUMBERS
    #define BLOCK_BUFFER_SIZE 16
  #else
    #define BLOCK_BUFFER_SIZE 18
  #endif
#endif

// This struct stores a linear movement of a g-code block motion with its critical "nominal" values
// are as specified in the source g-code. 
typedef struct {
  // Fields used by the bresenham algorithm for tracing the line
  // NOTE: Used by stepper algorithm to execute the block correctly. Do not alter these values.
  uint8_t direction_bits;    // The direction bit set for this block (refers to *_DIRECTION_BIT in config.h)
  uint32_t steps[N_AXIS];    // Step count along each axis
  uint32_t step_event_count; // The maximum step axis count and number of steps required to complete this block. 

  // Fields used by the motion planner to manage acceleration
  float entry_speed_sqr;         // The current planned entry speed at block junction in (mm/min)^2
  float max_entry_speed_sqr;     // Maximum allowable entry speed based on the minimum of junction limit and 
                                 //   neighboring nominal speeds with overrides in (mm/min)^2
  float max_junction_speed_sqr;  // Junction entry speed limit based on direction vectors in (mm/min)^2
  float nominal_speed_sqr;       // Axis-limit adjusted nominal speed for this block in (mm/min)^2
  float acceleration;            // Axis-limit adjusted line acceleration in (mm/min^2)
  float millimeters;             // The remaining distance for this block to be executed in (mm)
  // uint8_t max_override;       // Maximum override value based on axis speed limits

  #ifdef USE_LINE_NUMBERS
    int32_t line_number;
  #endif
} plan_block_t;

      
// Initialize and reset the motion plan subsystem
void plan_reset();

// Add a new linear movement to the buffer. target[N_AXIS] is the signed, absolute target position 
// in millimeters. Feed rate specifies the speed of the motion. If feed rate is inverted, the feed
// rate is taken to mean "frequency" and would complete the operation in 1/feed_rate minutes.
#ifdef USE_LINE_NUMBERS
  void plan_buffer_line(float *target, float feed_rate, uint8_t invert_feed_rate, int32_t line_number);
#else
  void plan_buffer_line(float *target, float feed_rate, uint8_t invert_feed_rate);
#endif

// Called when the current block is no longer needed. Discards the block and makes the memory
// availible for new blocks.
void plan_discard_current_block();

// Gets the current block. Returns NULL if buffer empty
plan_block_t *plan_get_current_block();

// Called periodically by step segment buffer. Mostly used internally by planner.
uint8_t plan_next_block_index(uint8_t block_index);

// Called by step segment buffer when computing executing block velocity profile.
float plan_get_exec_block_exit_speed();

// Reset the planner position vector (in steps)
void plan_sync_position();

// Reinitialize plan with a partially completed block
void plan_cycle_reinitialize();

// Returns the number of active blocks are in the planner buffer.
uint8_t plan_get_block_buffer_count();

// Returns the status of the block ring buffer. True, if buffer is full.
uint8_t plan_check_full_buffer();

#endif
