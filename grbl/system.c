

#include "grbl.h"


void system_init() 
{
  CONTROL_DDR &= ~(CONTROL_MASK); // Configure as input pins
  #ifdef DISABLE_CONTROL_PIN_PULL_UP
    CONTROL_PORT &= ~(CONTROL_MASK); // Normal low operation. Requires external pull-down.
  #else
    CONTROL_PORT |= CONTROL_MASK;   // Enable internal pull-up resistors. Normal high operation.
  #endif
  CONTROL_PCMSK |= CONTROL_MASK;  // Enable specific pins of the Pin Change Interrupt
  PCICR |= (1 << CONTROL_INT);   // Enable Pin Change Interrupt
}


// Pin change interrupt for pin-out commands, i.e. cycle start, feed hold, and reset. Sets
// only the realtime command execute variable to have the main program execute these when 
// its ready. This works exactly like the character-based realtime commands when picked off
// directly from the incoming serial data stream.
ISR(CONTROL_INT_vect) 
{
  uint8_t pin = (CONTROL_PIN & CONTROL_MASK);
  #ifndef INVERT_ALL_CONTROL_PINS
    pin ^= CONTROL_INVERT_MASK;
  #endif
  // Enter only if any CONTROL pin is detected as active.
  if (pin) { 
    if (bit_istrue(pin,bit(RESET_BIT))) {
      mc_reset();
    } else if (bit_istrue(pin,bit(CYCLE_START_BIT))) {
      bit_true(sys_rt_exec_state, EXEC_CYCLE_START);
    #ifndef ENABLE_SAFETY_DOOR_INPUT_PIN
      } else if (bit_istrue(pin,bit(FEED_HOLD_BIT))) {
        bit_true(sys_rt_exec_state, EXEC_FEED_HOLD); 
    #else
      } else if (bit_istrue(pin,bit(SAFETY_DOOR_BIT))) {
        bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
    #endif
    } 
  }
}


// Returns if safety door is ajar(T) or closed(F), based on pin state.
uint8_t system_check_safety_door_ajar()
{
  #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    #ifdef INVERT_CONTROL_PIN
      return(bit_istrue(CONTROL_PIN,bit(SAFETY_DOOR_BIT)));
    #else
      return(bit_isfalse(CONTROL_PIN,bit(SAFETY_DOOR_BIT)));
    #endif
  #else
    return(false); // Input pin not enabled, so just return that it's closed.
  #endif
}


// Executes user startup script, if stored.
void system_execute_startup(char *line) 
{
  uint8_t n;
  for (n=0; n < N_STARTUP_LINE; n++) {
    if (!(settings_read_startup_line(n, line))) {
      report_status_message(STATUS_SETTING_READ_FAIL);
    } else {
      if (line[0] != 0) {
        printString(line); // Echo startup line to indicate execution.
        report_status_message(gc_execute_line(line));
      }
    } 
  }  
}


uint8_t system_execute_line(char *line) 
{   
  uint8_t char_counter = 1; 
  uint8_t helper_var = 0; // Helper variable
  float parameter, value;
  switch( line[char_counter] ) {
    case 0 : report_grbl_help(); break;
    case '$': case 'G': case 'C': case 'X':
      if ( line[(char_counter+1)] != 0 ) { return(STATUS_INVALID_STATEMENT); }
      switch( line[char_counter] ) {
        case '$' : // Prints Grbl settings
          if ( sys.state & (STATE_CYCLE | STATE_HOLD) ) { return(STATUS_IDLE_ERROR); } // Block during cycle. Takes too long to print.
          else { report_grbl_settings(); }
          break;
        case 'G' : // Prints gcode parser state
          // TODO: Move this to realtime commands for GUIs to request this data during suspend-state.
          report_gcode_modes();
          break;   
        case 'C' : // Set check g-code mode [IDLE/CHECK]
          // Perform reset when toggling off. Check g-code mode should only work if Grbl
          // is idle and ready, regardless of alarm locks. This is mainly to keep things
          // simple and consistent.
          if ( sys.state == STATE_CHECK_MODE ) { 
            mc_reset(); 
            report_feedback_message(MESSAGE_DISABLED);
          } else {
            if (sys.state) { return(STATUS_IDLE_ERROR); } // Requires no alarm mode.
            sys.state = STATE_CHECK_MODE;
            report_feedback_message(MESSAGE_ENABLED);
          }
          break; 
        case 'X' : // Disable alarm lock [ALARM]
          if (sys.state == STATE_ALARM) { 
            report_feedback_message(MESSAGE_ALARM_UNLOCK);
            sys.state = STATE_IDLE;
            // Don't run startup script. Prevents stored moves in startup from causing accidents.
            if (system_check_safety_door_ajar()) { // Check safety door switch before returning.
              bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
              protocol_execute_realtime(); // Enter safety door mode.
            }
          } // Otherwise, no effect.
          break;                   
   
      }
      break;
    default : 
      // Block any system command that requires the state as IDLE/ALARM. (i.e. EEPROM, homing)
      if ( !(sys.state == STATE_IDLE || sys.state == STATE_ALARM) ) { return(STATUS_IDLE_ERROR); }
      switch( line[char_counter] ) {
        case '#' : // Print Grbl NGC parameters
          if ( line[++char_counter] != 0 ) { return(STATUS_INVALID_STATEMENT); }
          else { report_ngc_parameters(); }
          break;          
        case 'H' : // Perform homing cycle [IDLE/ALARM]
          if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { 
            sys.state = STATE_HOMING; // Set system state variable
            // Only perform homing if Grbl is idle or lost.
            
            // TODO: Likely not required.
            if (system_check_safety_door_ajar()) { // Check safety door switch before homing.
              bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
              protocol_execute_realtime(); // Enter safety door mode.
            }
            
            
            mc_homing_cycle(); 
            if (!sys.abort) {  // Execute startup scripts after successful homing.
              sys.state = STATE_IDLE; // Set to IDLE when complete.
              st_go_idle(); // Set steppers to the settings idle state before returning.
              system_execute_startup(line); 
            }
          } else { return(STATUS_SETTING_DISABLED); }
          break;
        case 'I' : // Print or store build info. [IDLE/ALARM]
          if ( line[++char_counter] == 0 ) { 
            settings_read_build_info(line);
            report_build_info(line);
          } else { // Store startup line [IDLE/ALARM]
            if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
            helper_var = char_counter; // Set helper variable as counter to start of user info line.
            do {
              line[char_counter-helper_var] = line[char_counter];
            } while (line[char_counter++] != 0);
            settings_store_build_info(line);
          }
          break; 
        case 'R' : // Restore defaults [IDLE/ALARM]
          if (line[++char_counter] != 'S') { return(STATUS_INVALID_STATEMENT); }
          if (line[++char_counter] != 'T') { return(STATUS_INVALID_STATEMENT); }
          if (line[++char_counter] != '=') { return(STATUS_INVALID_STATEMENT); }
          if (line[char_counter+2] != 0) { return(STATUS_INVALID_STATEMENT); }                        
          switch (line[++char_counter]) {
            case '$': settings_restore(SETTINGS_RESTORE_DEFAULTS); break;
            case '#': settings_restore(SETTINGS_RESTORE_PARAMETERS); break;
            case '*': settings_restore(SETTINGS_RESTORE_ALL); break;
            default: return(STATUS_INVALID_STATEMENT);
          }
          report_feedback_message(MESSAGE_RESTORE_DEFAULTS);
          mc_reset(); // Force reset to ensure settings are initialized correctly.
          break;
        case 'N' : // Startup lines. [IDLE/ALARM]
          if ( line[++char_counter] == 0 ) { // Print startup lines
            for (helper_var=0; helper_var < N_STARTUP_LINE; helper_var++) {
              if (!(settings_read_startup_line(helper_var, line))) {
                report_status_message(STATUS_SETTING_READ_FAIL);
              } else {
                report_startup_line(helper_var,line);
              }
            }
            break;
          } else { // Store startup line [IDLE Only] Prevents motion during ALARM.
            if (sys.state != STATE_IDLE) { return(STATUS_IDLE_ERROR); } // Store only when idle.
            helper_var = true;  // Set helper_var to flag storing method. 
            // No break. Continues into default: to read remaining command characters.
          }
        default :  // Storing setting methods [IDLE/ALARM]
          if(!read_float(line, &char_counter, &parameter)) { return(STATUS_BAD_NUMBER_FORMAT); }
          if(line[char_counter++] != '=') { return(STATUS_INVALID_STATEMENT); }
          if (helper_var) { // Store startup line
            // Prepare sending gcode block to gcode parser by shifting all characters
            helper_var = char_counter; // Set helper variable as counter to start of gcode block
            do {
              line[char_counter-helper_var] = line[char_counter];
            } while (line[char_counter++] != 0);
            // Execute gcode block to ensure block is valid.
            helper_var = gc_execute_line(line); // Set helper_var to returned status code.
            if (helper_var) { return(helper_var); }
            else { 
              helper_var = trunc(parameter); // Set helper_var to int value of parameter
              settings_store_startup_line(helper_var,line);
            }
          } else { // Store global setting.
            if(!read_float(line, &char_counter, &value)) { return(STATUS_BAD_NUMBER_FORMAT); }
            if((line[char_counter] != 0) || (parameter > 255)) { return(STATUS_INVALID_STATEMENT); }
            return(settings_store_global_setting((uint8_t)parameter, value));
          }
      }    
  }
  return(STATUS_OK); // If '$' command makes it to here, then everything's ok.
}


// Returns machine position of axis 'idx'. Must be sent a 'step' array.
// NOTE: If motor steps and machine position are not in the same coordinate frame, this function
//   serves as a central place to compute the transformation.
float system_convert_axis_steps_to_mpos(int32_t *steps, uint8_t idx)
{
  float pos;
  #ifdef COREXY
    if (idx==A_MOTOR) { 
      pos = 0.5*((steps[A_MOTOR] + steps[B_MOTOR])/settings.steps_per_mm[idx]);
    } else if (idx==B_MOTOR) {
      pos = 0.5*((steps[A_MOTOR] - steps[B_MOTOR])/settings.steps_per_mm[idx]);
    } else {
      pos = steps[idx]/settings.steps_per_mm[idx];
    }
  #else
    pos = steps[idx]/settings.steps_per_mm[idx];
  #endif
  return(pos);
}
  

void system_convert_array_steps_to_mpos(float *position, int32_t *steps)
{
  uint8_t idx;
  for (idx=0; idx<N_AXIS; idx++) {
    position[idx] = system_convert_axis_steps_to_mpos(steps, idx);
  }
  return;
}
