
#include "grbl.h"

#define SOME_LARGE_VALUE 1.0E+38 // Used by rapids and acceleration maximization calculations. Just needs
                                 // to be larger than any feasible (mm/min)^2 or mm/sec^2 value.

static plan_block_t block_buffer[BLOCK_BUFFER_SIZE];  // A ring buffer for motion instructions
static uint8_t block_buffer_tail;     // Index of the block to process now
static uint8_t block_buffer_head;     // Index of the next block to be pushed
static uint8_t next_buffer_head;      // Index of the next buffer head
static uint8_t block_buffer_planned;  // Index of the optimally planned block

// Define planner variables
typedef struct {
  int32_t position[N_AXIS];          // The planner position of the tool in absolute steps. Kept separate
                                     // from g-code position for movements requiring multiple line motions,
                                     // i.e. arcs, canned cycles, and backlash compensation.
  float previous_unit_vec[N_AXIS];   // Unit vector of previous path line segment
  float previous_nominal_speed_sqr;  // Nominal speed of previous path line segment
} planner_t;
static planner_t pl;


// Returns the index of the next block in the ring buffer. Also called by stepper segment buffer.
uint8_t plan_next_block_index(uint8_t block_index) 
{
  block_index++;
  if (block_index == BLOCK_BUFFER_SIZE) { block_index = 0; }
  return(block_index);
}


// Returns the index of the previous block in the ring buffer
static uint8_t plan_prev_block_index(uint8_t block_index) 
{
  if (block_index == 0) { block_index = BLOCK_BUFFER_SIZE; }
  block_index--;
  return(block_index);
}


static void planner_recalculate()
{   
  // Initialize block index to the last block in the planner buffer.
  uint8_t block_index = plan_prev_block_index(block_buffer_head);
        
  // Bail. Can't do anything with one only one plan-able block.
  if (block_index == block_buffer_planned) { return; }
      
  
  float entry_speed_sqr;
  plan_block_t *next;
  plan_block_t *current = &block_buffer[block_index];

  // Calculate maximum entry speed for last block in buffer, where the exit speed is always zero.
  current->entry_speed_sqr = min( current->max_entry_speed_sqr, 2*current->acceleration*current->millimeters);
  
  block_index = plan_prev_block_index(block_index);
  if (block_index == block_buffer_planned) { // Only two plannable blocks in buffer. Reverse pass complete.
    // Check if the first block is the tail. If so, notify stepper to update its current parameters.
    if (block_index == block_buffer_tail) { st_update_plan_block_parameters(); }
  } else { // Three or more plan-able blocks
    while (block_index != block_buffer_planned) { 
      next = current;
      current = &block_buffer[block_index];
      block_index = plan_prev_block_index(block_index);

      // Check if next block is the tail block(=planned block). If so, update current stepper parameters.
      if (block_index == block_buffer_tail) { st_update_plan_block_parameters(); } 

      // Compute maximum entry speed decelerating over the current block from its exit speed.
      if (current->entry_speed_sqr != current->max_entry_speed_sqr) {
        entry_speed_sqr = next->entry_speed_sqr + 2*current->acceleration*current->millimeters;
        if (entry_speed_sqr < current->max_entry_speed_sqr) {
          current->entry_speed_sqr = entry_speed_sqr;
        } else {
          current->entry_speed_sqr = current->max_entry_speed_sqr;
        }
      }
    }
  }    

  // Forward Pass: Forward plan the acceleration curve from the planned pointer onward.
  // Also scans for optimal plan breakpoints and appropriately updates the planned pointer.
  next = &block_buffer[block_buffer_planned]; // Begin at buffer planned pointer
  block_index = plan_next_block_index(block_buffer_planned); 
  while (block_index != block_buffer_head) {
    current = next;
    next = &block_buffer[block_index];
    
    // Any acceleration detected in the forward pass automatically moves the optimal planned
    // pointer forward, since everything before this is all optimal. In other words, nothing
    // can improve the plan from the buffer tail to the planned pointer by logic.
    if (current->entry_speed_sqr < next->entry_speed_sqr) {
      entry_speed_sqr = current->entry_speed_sqr + 2*current->acceleration*current->millimeters;
      // If true, current block is full-acceleration and we can move the planned pointer forward.
      if (entry_speed_sqr < next->entry_speed_sqr) {
        next->entry_speed_sqr = entry_speed_sqr; // Always <= max_entry_speed_sqr. Backward pass sets this.
        block_buffer_planned = block_index; // Set optimal plan pointer.
      }
    }
    
    // Any block set at its maximum entry speed also creates an optimal plan up to this
    // point in the buffer. When the plan is bracketed by either the beginning of the
    // buffer and a maximum entry speed or two maximum entry speeds, every block in between
    // cannot logically be further improved. Hence, we don't have to recompute them anymore.
    if (next->entry_speed_sqr == next->max_entry_speed_sqr) { block_buffer_planned = block_index; }
    block_index = plan_next_block_index( block_index );
  } 
}


void plan_reset() 
{
  memset(&pl, 0, sizeof(pl)); // Clear planner struct
  block_buffer_tail = 0;
  block_buffer_head = 0; // Empty = tail
  next_buffer_head = 1; // plan_next_block_index(block_buffer_head)
  block_buffer_planned = 0; // = block_buffer_tail;
}


void plan_discard_current_block() 
{
  if (block_buffer_head != block_buffer_tail) { // Discard non-empty buffer.
    uint8_t block_index = plan_next_block_index( block_buffer_tail );
    // Push block_buffer_planned pointer, if encountered.
    if (block_buffer_tail == block_buffer_planned) { block_buffer_planned = block_index; }
    block_buffer_tail = block_index;
  }
}


plan_block_t *plan_get_current_block() 
{
  if (block_buffer_head == block_buffer_tail) { return(NULL); } // Buffer empty  
  return(&block_buffer[block_buffer_tail]);
}


float plan_get_exec_block_exit_speed()
{
  uint8_t block_index = plan_next_block_index(block_buffer_tail);
  if (block_index == block_buffer_head) { return( 0.0 ); }
  return( sqrt( block_buffer[block_index].entry_speed_sqr ) ); 
}


// Returns the availability status of the block ring buffer. True, if full.
uint8_t plan_check_full_buffer()
{
  if (block_buffer_tail == next_buffer_head) { return(true); }
  return(false);
}


#ifdef USE_LINE_NUMBERS
  void plan_buffer_line(float *target, float feed_rate, uint8_t invert_feed_rate, int32_t line_number) 
#else
  void plan_buffer_line(float *target, float feed_rate, uint8_t invert_feed_rate) 
#endif
{
  // Prepare and initialize new block
  plan_block_t *block = &block_buffer[block_buffer_head];
  block->step_event_count = 0;
  block->millimeters = 0;
  block->direction_bits = 0;
  block->acceleration = SOME_LARGE_VALUE; // Scaled down to maximum acceleration later
  #ifdef USE_LINE_NUMBERS
    block->line_number = line_number;
  #endif

  // Compute and store initial move distance data.
  // TODO: After this for-loop, we don't touch the stepper algorithm data. Might be a good idea
  // to try to keep these types of things completely separate from the planner for portability.
  int32_t target_steps[N_AXIS];
  float unit_vec[N_AXIS], delta_mm;
  uint8_t idx;
  #ifdef COREXY
    target_steps[A_MOTOR] = lround(target[A_MOTOR]*settings.steps_per_mm[A_MOTOR]);
    target_steps[B_MOTOR] = lround(target[B_MOTOR]*settings.steps_per_mm[B_MOTOR]);
    block->steps[A_MOTOR] = labs((target_steps[X_AXIS]-pl.position[X_AXIS]) + (target_steps[Y_AXIS]-pl.position[Y_AXIS]));
    block->steps[B_MOTOR] = labs((target_steps[X_AXIS]-pl.position[X_AXIS]) - (target_steps[Y_AXIS]-pl.position[Y_AXIS]));
  #endif

  for (idx=0; idx<N_AXIS; idx++) {
    // Calculate target position in absolute steps, number of steps for each axis, and determine max step events.
    // Also, compute individual axes distance for move and prep unit vector calculations.
    // NOTE: Computes true distance from converted step values.
    #ifdef COREXY
      if ( !(idx == A_MOTOR) && !(idx == B_MOTOR) ) {
        target_steps[idx] = lround(target[idx]*settings.steps_per_mm[idx]);
        block->steps[idx] = labs(target_steps[idx]-pl.position[idx]);
      }
      block->step_event_count = max(block->step_event_count, block->steps[idx]);
      if (idx == A_MOTOR) {
        delta_mm = ((target_steps[X_AXIS]-pl.position[X_AXIS]) + (target_steps[Y_AXIS]-pl.position[Y_AXIS]))/settings.steps_per_mm[idx];
      } else if (idx == B_MOTOR) {
        delta_mm = ((target_steps[X_AXIS]-pl.position[X_AXIS]) - (target_steps[Y_AXIS]-pl.position[Y_AXIS]))/settings.steps_per_mm[idx];
      } else {
        delta_mm = (target_steps[idx] - pl.position[idx])/settings.steps_per_mm[idx];
      }
    #else
      target_steps[idx] = lround(target[idx]*settings.steps_per_mm[idx]);
      block->steps[idx] = labs(target_steps[idx]-pl.position[idx]);
      block->step_event_count = max(block->step_event_count, block->steps[idx]);
      delta_mm = (target_steps[idx] - pl.position[idx])/settings.steps_per_mm[idx];
    #endif
    unit_vec[idx] = delta_mm; // Store unit vector numerator. Denominator computed later.
        
    // Set direction bits. Bit enabled always means direction is negative.
    if (delta_mm < 0 ) { block->direction_bits |= get_direction_pin_mask(idx); }
    
    // Incrementally compute total move distance by Euclidean norm. First add square of each term.
    if (idx != A_AXIS)
      block->millimeters += delta_mm*delta_mm;
  }
  
  float xyz_mm = sqrt(block->millimeters);
  block->millimeters = sqrt(block->millimeters+(delta_mm*delta_mm)); // Complete millimeters calculation with sqrt()
  
  // Bail if this is a zero-length block. Highly unlikely to occur.
  if (block->step_event_count == 0) { return; } 
  
  // Adjust feed_rate value to mm/min depending on type of rate input (normal, inverse time, or rapids)
  // TODO: Need to distinguish a rapids vs feed move for overrides. Some flag of some sort.
  if (feed_rate < 0) { feed_rate = SOME_LARGE_VALUE; } // Scaled down to absolute max/rapids rate later
  else if (invert_feed_rate) { feed_rate *= block->millimeters; }
  if (feed_rate < MINIMUM_FEED_RATE) { feed_rate = MINIMUM_FEED_RATE; } // Prevents step generation round-off condition.

  // Calculate the unit vector of the line move and the block maximum feed rate and acceleration scaled 
  // down such that no individual axes maximum values are exceeded with respect to the line direction. 
  // NOTE: This calculation assumes all axes are orthogonal (Cartesian) and works with ABC-axes,
  // if they are also orthogonal/independent. Operates on the absolute value of the unit vector.
  float inverse_unit_vec_value;
  float inverse_millimeters = 1.0/block->millimeters;  // Inverse millimeters to remove multiple float divides	
  float junction_cos_theta = 0;
  for (idx=0; idx<N_AXIS; idx++) {
    if (unit_vec[idx] != 0) {  // Avoid divide by zero.
      unit_vec[idx] *= inverse_millimeters;  // Complete unit vector calculation
      inverse_unit_vec_value = fabs(1.0/unit_vec[idx]); // Inverse to remove multiple float divides.

      // Check and limit feed rate against max individual axis velocities and accelerations
      feed_rate = min(feed_rate,settings.max_rate[idx]*inverse_unit_vec_value);
      block->acceleration = min(block->acceleration,settings.acceleration[idx]*inverse_unit_vec_value);

      // Incrementally compute cosine of angle between previous and current path. Cos(theta) of the junction
      // between the current move and the previous move is simply the dot product of the two unit vectors, 
      // where prev_unit_vec is negative. Used later to compute maximum junction speed.
      if (idx != A_AXIS)
          junction_cos_theta -= pl.previous_unit_vec[idx] * unit_vec[idx];
    }
  }
  
  // TODO: Need to check this method handling zero junction speeds when starting from rest.
  if (block_buffer_head == block_buffer_tail) {
  
    // Initialize block entry speed as zero. Assume it will be starting from rest. Planner will correct this later.
    block->entry_speed_sqr = 0.0;
    block->max_junction_speed_sqr = 0.0; // Starting from rest. Enforce start from zero velocity.
  
  } else {
        // NOTE: Computed without any expensive trig, sin() or acos(), by trig half angle identity of cos(theta).
    if (junction_cos_theta > 0.999999) {
      //  For a 0 degree acute junction, just set minimum junction speed. 
      block->max_junction_speed_sqr = MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED;
    } else {
      junction_cos_theta = max(junction_cos_theta,-0.999999); // Check for numerical round-off to avoid divide by zero.
      float sin_theta_d2 = sqrt(0.5*(1.0-junction_cos_theta)); // Trig half angle identity. Always positive.

      // TODO: Technically, the acceleration used in calculation needs to be limited by the minimum of the
      // two junctions. However, this shouldn't be a significant problem except in extreme circumstances.
      block->max_junction_speed_sqr = max( MINIMUM_JUNCTION_SPEED*MINIMUM_JUNCTION_SPEED,
                                   (block->acceleration * settings.junction_deviation * sin_theta_d2)/(1.0-sin_theta_d2) );

    }
  }

  // Store block nominal speed
  block->nominal_speed_sqr = feed_rate*feed_rate; // (mm/min). Always > 0
  
  // Compute the junction maximum entry based on the minimum of the junction speed and neighboring nominal speeds.
  block->max_entry_speed_sqr = min(block->max_junction_speed_sqr, 
                                   min(block->nominal_speed_sqr,pl.previous_nominal_speed_sqr));
  
  // Update previous path unit_vector and nominal speed (squared)
  memcpy(pl.previous_unit_vec, unit_vec, sizeof(unit_vec)); // pl.previous_unit_vec[] = unit_vec[]
  pl.previous_nominal_speed_sqr = block->nominal_speed_sqr;
    
  // Update planner position
  memcpy(pl.position, target_steps, sizeof(target_steps)); // pl.position[] = target_steps[]

  // New block is all set. Update buffer head and next buffer head indices.
  block_buffer_head = next_buffer_head;  
  next_buffer_head = plan_next_block_index(block_buffer_head);
  
  // Finish up by recalculating the plan with the new block.
  planner_recalculate();
}


// Reset the planner position vectors. Called by the system abort/initialization routine.
void plan_sync_position()
{
  // TODO: For motor configurations not in the same coordinate frame as the machine position,
  // this function needs to be updated to accomodate the difference. 
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    #ifdef COREXY
     if (idx==A_MOTOR) { 
        pl.position[idx] = (sys.position[A_MOTOR] + sys.position[B_MOTOR])/2;
      } else if (idx==B_MOTOR) { 
        pl.position[idx] = (sys.position[A_MOTOR] - sys.position[B_MOTOR])/2;
      } else {
        pl.position[idx] = sys.position[idx];
      }
    #else
      pl.position[idx] = sys.position[idx];
    #endif
  }
}


// Returns the number of active blocks are in the planner buffer.
uint8_t plan_get_block_buffer_count()
{
  if (block_buffer_head >= block_buffer_tail) { return(block_buffer_head-block_buffer_tail); }
  return(BLOCK_BUFFER_SIZE - (block_buffer_tail-block_buffer_head));
}


// Re-initialize buffer plan with a partially completed block, assumed to exist at the buffer tail.
// Called after a steppers have come to a complete stop for a feed hold and the cycle is stopped.
void plan_cycle_reinitialize()
{
  // Re-plan from a complete stop. Reset planner entry speeds and buffer planned pointer.
  st_update_plan_block_parameters();
  block_buffer_planned = block_buffer_tail;
  planner_recalculate();  
}
