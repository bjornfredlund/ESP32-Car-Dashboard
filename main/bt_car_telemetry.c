#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "gnss.h"
#include "screen.h"
#include "led.h"
#include "esp_log.h"
#include "thermometer.h"
#include "driver/gpio.h"
#include <inttypes.h>

#include "led_blink_task.h"
#include "driver/i2c_master.h"
#include "common.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "accelerometer.h"
#include "bt_a2dp.h"

#define I2C_MASTER_SCL_IO 17        /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO 16        /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM 0            /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000   /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0 /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000
#define TMP117_ADDR 0x48     // TMP117 I2C address
#define TMP117_TEMP_REG 0x00 // Temperature register

#define SCREEN_QUEUE_SIZE (10)
static i2c_master_dev_handle_t dev_handle;
const char *TAG = "Main";
i2c_master_bus_handle_t bus_handle;

#define GPIO_INPUT_PLUS 34
#define GPIO_INPUT_MINUS 35
#define GPIO_INPUT_PIN_SEL ((1ULL << GPIO_INPUT_MINUS) | (1ULL << GPIO_INPUT_PLUS))

static i2c_master_bus_config_t i2c_bus_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_MASTER_NUM,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,
    .flags.enable_internal_pullup = true,
};

static QueueHandle_t screen_queue_handle;
static task_params_t task_params;

static i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = 0x48,
    .scl_speed_hz = 100000,
};

static SemaphoreHandle_t i2c_mutex;

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
    int gpio_num = (int)arg;
    //  ESP_LOGI(TAG, "gpio num pressed %d\n", (int)gpio_num);
    screen_msg_t msg = {
        .msg_type = SCREEN_INFO_NEXT,
        .data = gpio_num,
    };
    xQueueSendFromISR(screen_queue_handle, &msg, NULL);
}

static void button_init()
{
    gpio_config_t io_conf = {};

    // interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    // bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    // enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);
    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_MINUS, gpio_isr_handler, (void *)GPIO_INPUT_MINUS);
    // hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_PLUS, gpio_isr_handler, (void *)GPIO_INPUT_PLUS);
}

void app_main(void)
{
    button_init();
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    screen_queue_handle = xQueueCreate(SCREEN_QUEUE_SIZE, sizeof(screen_msg_t));

    i2c_mutex = xSemaphoreCreateMutex();

    task_params.bus_handle = bus_handle;
    task_params.i2c_mutex = i2c_mutex;
    task_params.queue_handle = screen_queue_handle;

    TaskHandle_t screen_task_handle;
    xTaskCreate(screen_task, "Screen", 8168, &task_params, 1, &screen_task_handle);    
    TaskHandle_t gnss_task_handle;
    xTaskCreate(gnss_task, "Gnss", 8192, &task_params, tskIDLE_PRIORITY, &gnss_task_handle);
    TaskHandle_t led_blink_task_handle;
    xTaskCreate(led_blink_task, "Blink tas", 4096, NULL, tskIDLE_PRIORITY, &led_blink_task_handle);
    TaskHandle_t led_task_handle;
    xTaskCreate(led_task, "Led", 4096, NULL, tskIDLE_PRIORITY, &led_task_handle);

    TaskHandle_t termometer_task_handle;
    xTaskCreate(thermo_task, "Thermo", 4096, &task_params, tskIDLE_PRIORITY, &termometer_task_handle);

    TaskHandle_t accelerometer_task_handle;
    xTaskCreate(accelerometer_task, "Acc", 4096, &task_params, tskIDLE_PRIORITY, &accelerometer_task_handle);

    
    TaskHandle_t bt_task_handle;
    xTaskCreate(bt_a2dp_task, "Acc", 4096, &task_params, tskIDLE_PRIORITY, &bt_task_handle);



}
