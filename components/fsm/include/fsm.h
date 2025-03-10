#ifndef _FSM_H
#define _FSM_H

#include "screen.h"

typedef void (*state_handler_t)(screen_msg_t* msg);

typedef struct {
    state_handler_t state;
} fsm_t;


void fsm_dispatch(const fsm_t* fsm, screen_msg_t* msg);
void fsm_transition(fsm_t* fsm, state_handler_t target);

#endif