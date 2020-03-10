#include <iostream>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "icm20648.h"

// ESP-IDFプログラミングガイド:
// Ref: https://docs.espressif.com/projects/esp-idf/en/v3.3.1/api-reference/peripherals/spi_master.html
// ICM-20648データシート：
// Ref: https://invensense.tdk.com/wp-content/uploads/2017/07/DS-000179-ICM-20648-v1.2-TYP.pdf

/* Inside .cpp file, app_main function must be declared with C linkage */
extern "C" void app_main(){

    while(1){
        std::cout<<"Hello ESP32 World"<<std::endl;
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
