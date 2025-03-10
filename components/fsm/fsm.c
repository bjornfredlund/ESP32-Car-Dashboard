#include <stdio.h>
#include "fsm.h"

void fsm_dispatch(const fsm_t* fsm, screen_msg_t* msg)
{
    fsm->state(msg);
}

void fsm_transition(fsm_t* fsm, state_handler_t target)
{
    fsm->state = target;
}
