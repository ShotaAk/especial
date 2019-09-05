
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


void batteryObservation(void){
    // バッテリ電圧の計測
    if(gBatteryVoltage < pLOW_BATTERY_VOLTAGE){
        gObsBatteryIsLow = TRUE;
    }else{
        gObsBatteryIsLow = FALSE;
    }
}

void touchObservation(void){
    // Object Sensorをタッチセンサーとして使用する
    const float TOUCH_THRESH_VOLTAGE = 1.8; // Volts

    if(gObjVoltages[OBJ_SENS_L] > TOUCH_THRESH_VOLTAGE
            && gObjVoltages[OBJ_SENS_FL] > TOUCH_THRESH_VOLTAGE){
        gObsTouch[LEFT] = TRUE;
    }else{
        gObsTouch[LEFT] = FALSE;
    }

    if(gObjVoltages[OBJ_SENS_R] > TOUCH_THRESH_VOLTAGE
            && gObjVoltages[OBJ_SENS_FR] > TOUCH_THRESH_VOLTAGE){
        gObsTouch[RIGHT] = TRUE;
    }else{
        gObsTouch[RIGHT] = FALSE;
    }
}


void TaskObservation(void *arg){
    // 観測データを加工するタスク


    ESP_LOGI(TAG, "Complete initialization.");
    while(1){
        batteryObservation();
        touchObservation();

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

}
