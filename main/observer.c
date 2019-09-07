
#include <stdio.h>
#include <time.h>
#include <math.h>
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

float normalize(const float angle){
    float normalizedAngle = angle;

    while(normalizedAngle > M_PI){
        normalizedAngle -= 2.0*M_PI;
    }
    while(normalizedAngle < -M_PI){
        normalizedAngle += 2.0*M_PI;
    }

    return normalizedAngle;
}

void movingDistanceObservation(void){
    // エンコーダの値から走行距離を計算する
    static float prevLeft, prevRight;
    static float prevVelocity[SIDE_NUM];
    static double prevTime;

    float diffAngle[SIDE_NUM];
    float currentTime = (float)clock() / (float)CLOCKS_PER_SEC;

    diffAngle[LEFT] = normalize(gWheelAngle[LEFT] - prevLeft);
    diffAngle[RIGHT] = normalize(gWheelAngle[RIGHT] - prevRight);
    double diffTime = currentTime - prevTime;
    
    // エンコーダの取り付け向きの都合上、左側の符号を反転する
    diffAngle[LEFT] *= -1.0;

    // 走行距離を加算
    gObsMovingDistance += pTIRE_RADIUS * (diffAngle[LEFT]+ diffAngle[RIGHT]) / 2.0;

    // タイヤの角速度を計算
    float angularVelocity[SIDE_NUM];
    angularVelocity[LEFT] = diffAngle[LEFT] / diffTime;
    angularVelocity[RIGHT] = diffAngle[RIGHT] / diffTime;
    // 角速度をタイヤの周速度に変換
    float velocity[SIDE_NUM];
    velocity[LEFT] = angularVelocity[LEFT] * pTIRE_RADIUS;
    velocity[RIGHT] = angularVelocity[RIGHT] * pTIRE_RADIUS;
    // ローパスフィルタをかける
    velocity[LEFT] = velocity[LEFT] * 0.1 + prevVelocity[LEFT] * 0.9;
    velocity[RIGHT] = velocity[RIGHT] * 0.1 + prevVelocity[RIGHT] * 0.9;

    // 車体速度に変換
    gObsSpeed = (velocity[LEFT] + velocity[RIGHT]) / 2.0;

    prevLeft = gWheelAngle[LEFT];
    prevRight = gWheelAngle[RIGHT];
    prevTime = currentTime;
    prevVelocity[LEFT] = velocity[LEFT];
    prevVelocity[RIGHT] = velocity[RIGHT];
}


void TaskObservation(void *arg){
    // 観測データを加工するタスク


    ESP_LOGI(TAG, "Complete initialization.");
    while(1){
        batteryObservation();
        touchObservation();
        movingDistanceObservation();

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

}
