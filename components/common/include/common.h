
#ifndef COMMON_H
#define COMMON_H

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "driver/i2c_master.h"


typedef struct {
    i2c_master_bus_handle_t bus_handle;
    SemaphoreHandle_t i2c_mutex;
    QueueHandle_t queue_handle;
} task_params_t;
#endif

