#include "../inc/state_machine.h"

void change_state(system_state_t* current_state, system_state_t new_state) {
    *current_state = new_state;
}

