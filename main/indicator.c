
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

#include "indicator.h"
#include "variables.h"
#include "parameters.h"


#define GPIO_LED0 22
#define GPIO_LED1 23

void TaskIndicator(void *arg){
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = ((1ULL<<GPIO_LED0) | (1ULL<<GPIO_LED1));
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    /* 省略 */

    gpio_set_level(GPIO_LED0, 0);
    gpio_set_level(GPIO_LED1, 0);

    bool ledToggle = false;
    while(1){
        if(gBatteryVoltage > LOW_BATTERY_VOLTAGE){
            // バッテリー電圧があれば、モードを表示
            gpio_set_level(GPIO_LED1, gIndicatorValue & 0x01);
            gpio_set_level(GPIO_LED0, (gIndicatorValue >> 1) & 0x01);
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }else{
            // 電圧がなければ、点滅させる
            if(ledToggle){
                gpio_set_level(GPIO_LED0, 1);
                gpio_set_level(GPIO_LED1, 1);
            }else{
                gpio_set_level(GPIO_LED0, 0);
                gpio_set_level(GPIO_LED1, 0);
            }

            ledToggle = !ledToggle;
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
    }
}

