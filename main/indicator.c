
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#include "indicator.h"
#include "variables.h"
#include "parameters.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
static const char *TAG="Indicator";

#define LED0_GPIO CONFIG_LED0_GPIO
#define LED1_GPIO CONFIG_LED1_GPIO

void TaskIndicator(void *arg){
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = ((1ULL<<LED0_GPIO) | (1ULL<<LED1_GPIO));
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    /* 省略 */

    gpio_set_level(LED0_GPIO, 0);
    gpio_set_level(LED1_GPIO, 0);

    bool ledToggle = false;

    ESP_LOGI(TAG, "Complete initialization.");
    while(1){
        if(gObsBatteryIsLow == FALSE){
            // バッテリー電圧があれば、モードを表示
            gpio_set_level(LED1_GPIO, gIndicatorValue & 0x01);
            gpio_set_level(LED0_GPIO, (gIndicatorValue >> 1) & 0x01);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }else{
            // 電圧がなければ、点滅させる
            if(ledToggle){
                gpio_set_level(LED0_GPIO, 1);
                gpio_set_level(LED1_GPIO, 1);
            }else{
                gpio_set_level(LED0_GPIO, 0);
                gpio_set_level(LED1_GPIO, 0);
            }

            ledToggle = !ledToggle;
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

