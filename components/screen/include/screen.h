#ifndef SCREEN_H
#define SCREEN_H

void screen_task(void *);


typedef enum
{
    SCREEN_INFO_NEXT,
    SCREEN_INFO_PREVIOUS,
    MSG_TEMP_MEASUREMENT,
    MSG_VEL_MEASUREMENT,
    MSG_ACC_MEASUREMENT
} screen_msg_type_t;

typedef struct
{
    screen_msg_type_t msg_type;
    union
    {
        float temp;
        float vel;
        float acc;
        uint16_t data;
    };
} screen_msg_t;
#endif