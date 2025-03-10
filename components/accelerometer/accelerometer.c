#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "accelerometer.h"
#include "esp_log.h"
#include "screen.h"
#include "common.h"
#include "driver/i2c_master.h"

#define LIS2DH12TR_ADDR 0x19     // TMP117 I2C address
#define LIS2DH12TR_TEMP_REG 0x0F // Temperature register

#define LIS2DH12TR_CTRL_REG 0x14 // Ctrl register

const uint8_t FIFTY_HERTZ_MODE_BITS = 0x70;

#define LIS2DH12TR_X_AXIS_FIFO_OUT 0x41

static i2c_master_dev_handle_t LIS2DH12TR_dev_handle;
static i2c_master_bus_handle_t master_i2c_bus;
static i2c_device_config_t LIS2DH12TR_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = LIS2DH12TR_ADDR,
    .scl_speed_hz = 100000,
};

static SemaphoreHandle_t i2c_mutex;

static const char *TAG = "Accelerometer";

typedef enum
{
    X_AXIS,
    Y_AXIS,
    Z_AXIS
} axis_t;

#define conversion_factor (0.061) // for +-2g
float read_axis(axis_t axis)
{
    uint8_t data[2];
    uint8_t reg;

    esp_err_t ret = ESP_ERR_TIMEOUT;
    switch (axis)
    {
    case X_AXIS:
        reg = LIS2DH12TR_X_AXIS_FIFO_OUT;
        break;
    default:
        break;
    }
    reg = 0x2C;

    ret = i2c_master_transmit_receive(LIS2DH12TR_dev_handle, &reg, 1, data, 2, -1);
    if (ret != ESP_OK)
    {
        return ret;
    }
    // return data[0];
    uint16_t measurement_raw = (data[1] << 8) | data[0];
    // return measurement_raw;
    // ESP_LOGI(TAG, "raw: %dmG\n", measurement_raw);

    return (float)((measurement_raw * conversion_factor) * 0.001);
}

static void lis2dh12tr_setup()
{
    uint8_t data[] = {0x13, 0x08};
    uint8_t return_data[2];

    xSemaphoreTake(i2c_mutex, portMAX_DELAY);
    i2c_master_transmit_receive(LIS2DH12TR_dev_handle, data, 2, return_data, 1, -1);
    data[0] = 0x14;
    data[1] = 0xA0 ;
    i2c_master_transmit_receive(LIS2DH12TR_dev_handle, data, 2, return_data, 1, -1);
    

    xSemaphoreGive(i2c_mutex);
}

void accelerometer_task(void *pv)
{

    task_params_t *tsk_params = (task_params_t *)pv;

    master_i2c_bus = tsk_params->bus_handle;
    i2c_mutex = tsk_params->i2c_mutex;
    QueueHandle_t screen_queue = tsk_params->queue_handle;

    ESP_ERROR_CHECK(i2c_master_bus_add_device(master_i2c_bus, &LIS2DH12TR_dev_cfg, &LIS2DH12TR_dev_handle));
    lis2dh12tr_setup();

    screen_msg_t msg = {
        .msg_type = MSG_ACC_MEASUREMENT,
    };
    while (1)
    {
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        float x_axis = read_axis(X_AXIS);
        xSemaphoreGive(i2c_mutex);
        msg.acc = x_axis;
        xQueueSend(screen_queue, (void *)&msg, portMAX_DELAY);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}
