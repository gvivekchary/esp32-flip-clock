/* SBB Flip Clockb based on ESP32 lwIP SNTP example 

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_attr.h"
#include "esp_sleep.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"
#include "esp_sntp.h"
#include "driver/uart.h"
#include "driver/gpio.h"

// Pin definitions: we only use DE and TX
#define DE_PIN       23
#define UART1_TX     22

// Addresses of the individual modules (see addr on sticker)
#define ADDR_MINUTE  56
#define ADDR_HOUR    7

const char * TAG = "SBB";

void time_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "Notification of a time synchronization event");
}

static void initialize_sntp(void)
{
    ESP_LOGI(TAG, "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(time_sync_notification_cb);
#ifdef CONFIG_SNTP_TIME_SYNC_METHOD_SMOOTH
    sntp_set_sync_mode(SNTP_SYNC_MODE_SMOOTH);
#endif
    sntp_init();
}

static void obtain_time(void)
{
    ESP_ERROR_CHECK( nvs_flash_init() );
    tcpip_adapter_init();
    ESP_ERROR_CHECK( esp_event_loop_create_default() );

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    initialize_sntp();

    // wait for time to be set
    time_t now = 0;
    struct tm timeinfo = { 0 };
    int retry = 0;
    const int retry_count = 10;
    while (sntp_get_sync_status() == SNTP_SYNC_STATUS_RESET && ++retry < retry_count) {
        ESP_LOGI(TAG, "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
    time(&now);
    localtime_r(&now, &timeinfo);

    // ESP_ERROR_CHECK( example_disconnect() );
}

// this depends on the individual module
int pos_for_hour(int hour){
  return hour+2;
}

int pos_for_minute(int minutes){
  if (minutes >= 31) return minutes-31; // 31 has pos 1
  return minutes+30; // 00 has pos 31
}

void set_pos_for_address(int address, int pos){
    uint8_t data[4];
    data[0] = 0xff;
    data[1] = 0xc0;
    data[2] = address;
    data[3] = pos;
    // Arduino sketch has break before data, sending it twice works as well
    uart_write_bytes_with_break(UART_NUM_1, (const char *) data, 4, 10);
    uart_write_bytes_with_break(UART_NUM_1, (const char *) data, 4, 10);
}

void setup_clock(void){
    // Set DE_PIN as output
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = 1 << DE_PIN;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    // set HIGH for RS485 TX
    gpio_set_level(DE_PIN, 1);

    // UART1 TEST
    static const int RX_BUF_SIZE = 1024;
    uart_config_t uart_config = {
        .baud_rate = 19200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, UART1_TX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, RX_BUF_SIZE * 2, 0, 0, NULL, 0);
}

void set_clock(uint8_t hour, uint8_t minute){
    set_pos_for_address(ADDR_HOUR,   pos_for_hour(hour));
    set_pos_for_address(ADDR_MINUTE, pos_for_minute(minute));
}

void app_main(void)
{
    uint8_t minute = 0xff;
    uint8_t hour   = 0xff;

    // setup clock
    setup_clock();

    // start SNTP
    obtain_time();

    // Set timezone to Zurich
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();

    while (1){
        time_t now;
        struct tm timeinfo;

        time(&now);
        char strftime_buf[64];
        localtime_r(&now, &timeinfo);
        strftime(strftime_buf, sizeof(strftime_buf), "%c", &timeinfo);
        ESP_LOGI(TAG, "The current date/time in Zurich is: %s", strftime_buf);

        // update clock on change
        if ((hour != timeinfo.tm_hour) || (minute != timeinfo.tm_min)) {
            hour   = timeinfo.tm_hour;
            minute = timeinfo.tm_min;
            set_clock(hour, minute);
        }

        // wait for time adjustment
        if (sntp_get_sync_mode() == SNTP_SYNC_MODE_SMOOTH) {
            struct timeval outdelta;
            while (sntp_get_sync_status() == SNTP_SYNC_STATUS_IN_PROGRESS) {
                adjtime(NULL, &outdelta);
                ESP_LOGI(TAG, "Waiting for adjusting time ... outdelta = %li sec: %li ms: %li us",
                            outdelta.tv_sec,
                            outdelta.tv_usec/1000,
                            outdelta.tv_usec%1000);
                vTaskDelay(2000 / portTICK_PERIOD_MS);
            }
        }

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}
