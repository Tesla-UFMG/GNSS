#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

typedef enum {
    STATE_RECEIVE_UART = 0,
	STATE_SEND_UART,
	STATE_IDLE
} system_state_t;


void change_state(system_state_t* current_state, system_state_t new_state);


#endif /* INC_STATE_MACHINE_H_ */
