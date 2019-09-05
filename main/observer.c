
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "observer.h"
#include "parameters.h"
#include "variables.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
static const char *TAG="Observer";

void TaskObservation(void *arg){
    // 観測データを加工するタスク


    ESP_LOGI(TAG, "Complete initialization.");
    while(1){
        // バッテリ電圧の低電圧監視
        if(gBatteryVoltage < pLOW_BATTERY_VOLTAGE){
            gObsBatteryIsLow = TRUE;
        }else{
            gObsBatteryIsLow = FALSE;
        }


        ESP_LOGI(TAG, "ObjVoltages L,FL,FR,R: %f,%f,%f,%f",
                gObjVoltages[OBJ_SENS_L],
                gObjVoltages[OBJ_SENS_FL],
                gObjVoltages[OBJ_SENS_FR],
                gObjVoltages[OBJ_SENS_R]);

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

}
