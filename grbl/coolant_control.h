

#ifndef coolant_control_h
#define coolant_control_h 


void coolant_init();
void coolant_stop();
void coolant_set_state(uint8_t mode);
void coolant_run(uint8_t mode);

#endif