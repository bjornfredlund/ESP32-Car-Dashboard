#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "thermometer.h"
#include "esp_log.h"
#include "screen.h"
#include "common.h"
#include "driver/i2c_master.h"

#define TMP117_ADDR 0x48     // TMP117 I2C address
#define TMP117_TEMP_REG 0x00 // Temperature register

static i2c_master_dev_handle_t tmp117_dev_handle;
static i2c_master_bus_handle_t master_i2c_bus;
static i2c_device_config_t tmp117_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = TMP117_ADDR,
    .scl_speed_hz = 100000,
};

static SemaphoreHandle_t i2c_mutex;

static const char *TAG = "Thermometer";

static esp_err_t read_temperature(float *temperature)
{
    uint8_t data[2];
    esp_err_t ret = ESP_ERR_TIMEOUT;

    uint8_t temp_reg = TMP117_TEMP_REG;

    // Write to the temperature register
    ret = i2c_master_transmit_receive(tmp117_dev_handle, &temp_reg, 1, data, 2, -1);
    if (ret != ESP_OK)
    {
        return ret;
    }

    // Convert raw data to temperature (0.0078125°C per bit)
    int16_t raw_temp = (data[0] << 8) | data[1];
    *temperature = raw_temp * 0.0078125;
    return ESP_OK;
}

void thermo_task(void *pv)
{
    task_params_t *tsk_params = (task_params_t *)pv;

    master_i2c_bus = tsk_params->bus_handle;
    i2c_mutex = tsk_params->i2c_mutex;
    QueueHandle_t screen_queue = tsk_params->queue_handle;
    
    ESP_ERROR_CHECK(i2c_master_bus_add_device(master_i2c_bus, &tmp117_dev_cfg, &tmp117_dev_handle));

    
    screen_msg_t msg = {
        .msg_type = MSG_TEMP_MEASUREMENT,
    };

    while (xSemaphoreTake(i2c_mutex, portMAX_DELAY))
    {
        float temperature;
        if (read_temperature(&temperature) == ESP_OK)
        {
            msg.temp = temperature;
            xQueueSend(screen_queue, (void*)&msg ,portMAX_DELAY);
//            ESP_LOGI(TAG, "Temperature: %.2f°C", temperature);
        }
        else
        {
            ESP_LOGI(TAG, "Failed to read temperature");
        }
        xSemaphoreGive(i2c_mutex);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
