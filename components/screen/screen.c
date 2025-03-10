#include <stdio.h>
#include "screen.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "esp_log.h"

#include <stdio.h>
#include "i2c-lcd.h"
#include "esp_log.h"
#include "driver/i2c_master.h"
#include "common.h"
#include "freertos/FreeRTOS.h"
#include "fsm.h"
#include "freertos/task.h"

static const char *TAG = "Main screen";

#define I2C_MASTER_SCL_IO 17        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 16        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

static i2c_master_dev_handle_t screen_dev_handle;

static i2c_device_config_t screen_dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x27,
    .scl_speed_hz = 100000,
};

typedef struct
{
    fsm_t state;
} screen_fsm_t;

static screen_fsm_t screen_fsm;

static i2c_master_bus_handle_t master_bus_handle;
static SemaphoreHandle_t i2c_mutex;
static char outBuf[20];

static void screen_startup_view(screen_msg_t *msg);

static void lcd_send_cmd(char cmd)
{
    esp_err_t err;
    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (cmd & 0xf0);
    data_l = ((cmd << 4) & 0xf0);
    data_t[0] = data_u | 0x0C; // en=1, rs=0
    data_t[1] = data_u | 0x08; // en=0, rs=0
    data_t[2] = data_l | 0x0C; // en=1, rs=0
    data_t[3] = data_l | 0x08; // en=0, rs=0
    err = i2c_master_transmit(screen_dev_handle, data_t, 4, -1);
    if (err != 0)
        ESP_LOGI(TAG, "Error in sending command");
}

static void lcd_put_cur(int row, int col)
{
    switch (row)
    {
    case 0:
        col |= 0x80;
        break;
    case 1:
        col |= 0xC0;
        break;
    }

    lcd_send_cmd(col);
}

static void lcd_clear(void)
{
    lcd_send_cmd(0x01);
    vTaskDelay(100 / portTICK_PERIOD_MS);
}

static void lcd_send_data(char data)
{
    esp_err_t err;

    char data_u, data_l;
    uint8_t data_t[4];
    data_u = (data & 0xf0);
    data_l = ((data << 4) & 0xf0);
    data_t[0] = data_u | 0x0D; // en=1, rs=0
    data_t[1] = data_u | 0x09; // en=0, rs=0
    data_t[2] = data_l | 0x0D; // en=1, rs=0
    data_t[3] = data_l | 0x09; // en=0, rs=0
    err = i2c_master_transmit(screen_dev_handle, data_t, 4, 1000);
    if (err != 0)
        ESP_LOGE(TAG, "Error in sending data");
}
void lcd_send_string(char *str)
{
    while (*str)
        lcd_send_data(*str++);
}

static void init_screen()
{
    const uint8_t data_wr = 0x30;
    const uint8_t four_bit = 0x20;
    lcd_send_cmd(0x30);
    vTaskDelay(40 / portTICK_PERIOD_MS);
    lcd_send_cmd(0x30);
    vTaskDelay(4 / portTICK_PERIOD_MS);
    lcd_send_cmd(0x30);
    vTaskDelay(10 / portTICK_PERIOD_MS);
    lcd_send_cmd(0x20);

    vTaskDelay(1000 / portTICK_PERIOD_MS);

    const uint8_t initialization_data[] = {0x28, 0x08, 0x01, 0x06, 0x0C};
    // size_t initialization_data_len = sizeof(initialization_data) / sizeof(initialization_data[0]);
    for (int i = 0; i < 5; i++)
    {
        lcd_send_cmd(initialization_data[i]);
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
    // ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, initialization_data, 5, -1));

    ESP_LOGI(TAG, "Screen initilization done!");
}

static void screen_gps_temp_view(screen_msg_t *msg)
{
    int row = 0;
    int col = 0;
    bool should_display = false;
    switch (msg->msg_type)
    {
    case SCREEN_INFO_NEXT:
        fsm_transition((fsm_t *)&screen_fsm, screen_startup_view);
        break;
    case SCREEN_INFO_PREVIOUS:
        break;
    case MSG_TEMP_MEASUREMENT:
        should_display = true;
        row = 1;
        snprintf(outBuf, 20, "Temp: %.1fC", msg->temp);
        break;
    case MSG_VEL_MEASUREMENT:
        should_display = true;

        snprintf(outBuf, 20, "Vel: %.1fkm/h", msg->vel);
        break;
    default:
        break;
    }

    if (should_display)
    {
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        lcd_put_cur(row, col);
        lcd_send_string(outBuf);
        xSemaphoreGive(i2c_mutex);
    }
}

static void screen_startup_view(screen_msg_t *msg)
{
    int row = 0;
    int col = 0;
    bool should_display = false;
    switch (msg->msg_type)
    {
    case SCREEN_INFO_NEXT:
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        lcd_clear();
        xSemaphoreGive(i2c_mutex);
        fsm_transition((fsm_t *)&screen_fsm, screen_gps_temp_view);
        break;
    case SCREEN_INFO_PREVIOUS:
        break;
    case MSG_TEMP_MEASUREMENT:
        should_display = true;
        snprintf(outBuf, 20, "Temp: %.1fC", msg->temp);
        break;
    case MSG_ACC_MEASUREMENT:
        row = 1;
        should_display = true;
        snprintf(outBuf, 20, "Acc: %.2f", msg->acc);
        break;
    default:
        break;
    }

    if (should_display)
    {
        xSemaphoreTake(i2c_mutex, portMAX_DELAY);
        lcd_put_cur(row, col);
        lcd_send_string(outBuf);
        xSemaphoreGive(i2c_mutex);
    }
}

static void screen_fsm_init()
{
    fsm_transition((fsm_t *)&screen_fsm, screen_startup_view);
}

void screen_task(void *pv)
{
    task_params_t *task_params = (task_params_t *)pv;
    master_bus_handle = task_params->bus_handle;
    i2c_mutex = task_params->i2c_mutex;
    QueueHandle_t screen_queue = task_params->queue_handle;

    screen_fsm_init();
    ESP_ERROR_CHECK(i2c_master_bus_add_device(master_bus_handle, &screen_dev_cfg, &screen_dev_handle));
    xSemaphoreTake(i2c_mutex, portMAX_DELAY);

    init_screen();
    lcd_clear();
    lcd_put_cur(0, 0);
    xSemaphoreGive(i2c_mutex);

    screen_msg_t msg;
    while (xQueueReceive(screen_queue, &msg, portMAX_DELAY))
    {
        fsm_dispatch((fsm_t *)&screen_fsm, &msg);
    }
}
