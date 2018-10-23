
#ifndef probe_h
#define probe_h 

// Values that define the probing state machine.  
#define PROBE_OFF     0 // Probing disabled or not in use. (Must be zero.)
#define PROBE_ACTIVE  1 // Actively watching the input pin.

// Probe pin initialization routine.
void probe_init();


void probe_configure_invert_mask(uint8_t is_probe_away);

// Returns probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
uint8_t probe_get_state();

// Monitors probe pin state and records the system position when detected. Called by the
// stepper ISR per ISR tick.
void probe_state_monitor();

#endif
