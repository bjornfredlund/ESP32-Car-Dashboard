#include <stdio.h>
#include "gnss.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/uart.h"
#include "esp_log.h"
#include <string.h>
#include "minmea.h"
#include "common.h"
#include <sys/types.h>
#include "screen.h"

#define UART_RX_GPIO 27
#define UART_TX_GPIO 26


static const int RX_BUF_SIZE = 1024;

static char *readLine(uart_port_t uart)
{
    static char line[256];
    int size;
    char *ptr = line;
    while (1)
    {
        size = uart_read_bytes(uart, (unsigned char *)ptr, 1, portMAX_DELAY);
        if (size == 1)
        {
            if (*ptr == '\n')
            {
                ptr++;
                *ptr = 0;
                return line;
            }
            ptr++;
        } // End of read a character
    } // End of loop
} // End of readLine

static const char *TAG = "GPS";
static void init_gnss()
{
    ESP_LOGI(TAG, "Initializing...");
  //  ESP_LOGI(TAG, "CONFIG init");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };

    uart_param_config(UART_NUM_1, &uart_config);

    uart_set_pin(UART_NUM_1, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    /* uart init: No TX buffer, no uart event queue */
    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_LOGI(TAG, "CONFIG done");
}
void gnss_run(QueueHandle_t *screen_queue)
{
    screen_msg_t msg = {
        .msg_type = MSG_VEL_MEASUREMENT,
    };

    ESP_LOGI(TAG, "GNSS INIT");
    for (;;)
    {
        char *line = readLine(UART_NUM_1);
        switch (minmea_sentence_id(line, false))
        {
        case MINMEA_SENTENCE_RMC:
        {
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, line))
            {
                float s = minmea_tofloat(&frame.speed);
                msg.vel = s;
                ESP_LOGI(TAG, "Speed 2: %.3f km/h", s);
            }
            else
            {
            }

            break;
        }
        case MINMEA_SENTENCE_VTG:
        {
            struct minmea_sentence_vtg frame;
            if (minmea_parse_vtg(&frame, line))
            {

                float s = minmea_tofloat(&frame.speed_kph);
                ESP_LOGI(TAG, "Speed: %.3f km/h", s);
               
            }
            break;
        }
        default:
            break;
        }
        if(msg.vel != NAN)
        {
            xQueueSend(*screen_queue, (void*)&msg , portMAX_DELAY);
        }
    }
}

void gnss_task(void* pv)
{
    task_params_t *tsk_params = (task_params_t *)pv;

    
    QueueHandle_t screen_queue = tsk_params->queue_handle;

    init_gnss();
    gnss_run(&screen_queue);
    while (1)
    {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        ESP_LOGI(TAG, "hello");
    }
}
