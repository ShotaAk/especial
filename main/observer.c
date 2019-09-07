
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

void wallObservation(void){
    // Object Sensorで壁を検出する
    const float THRESH_VOLTAGES[DIREC_NUM] = {
        [DIREC_FRONT] = 0.14, 
        [DIREC_LEFT]  = 0.15, 
        [DIREC_RIGHT] = 0.15, 
        [DIREC_BACK]  = 0.5}; // V


    if(gObjVoltages[OBJ_SENS_L] > THRESH_VOLTAGES[DIREC_LEFT]){
        gObsIsWall[DIREC_LEFT] = TRUE;
    }else{
        gObsIsWall[DIREC_LEFT] = FALSE;
    }

    if(gObjVoltages[OBJ_SENS_R] > THRESH_VOLTAGES[DIREC_RIGHT]){
        gObsIsWall[DIREC_RIGHT] = TRUE;
    }else{
        gObsIsWall[DIREC_RIGHT] = FALSE;
    }

    if( (gObjVoltages[OBJ_SENS_FL] + gObjVoltages[OBJ_SENS_FR]) * 0.5 
            > THRESH_VOLTAGES[DIREC_FRONT]){
        gObsIsWall[DIREC_FRONT] = TRUE;
    }else{
        gObsIsWall[DIREC_FRONT] = FALSE;
    }
}

void movingDistanceObservation(void){
    // エンコーダの値から走行距離と走行速度を計算する
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

void angleObservation(void){
    // ジャイロの値から現在角度を取得する
    static double prevTime;

    float currentTime = (float)clock() / (float)CLOCKS_PER_SEC;
    double diffTime = currentTime - prevTime;

    gObsAngle += diffTime * gGyro[AXIS_Z];

    prevTime = currentTime;
}


void dialObservation(void){
    // エンコーダの値を使って、ダイアルを表現する
    
    const float THRESH = M_PI * 0.5;
    const int DIAL_MAX = 9;
    const int DIAL_MIN = 0;
    
    static float prevTriggerdAngle;

    float currentAngle = gWheelAngle[RIGHT];
    float diffAngle = normalize(currentAngle - prevTriggerdAngle);

    ESP_LOGD(TAG, "DiffAngle: %f", diffAngle);
    if(diffAngle > THRESH){
        // インクリメント
        gObsDial++;
        prevTriggerdAngle = currentAngle;
    }else if(diffAngle < -THRESH){
        // デクリメント
        gObsDial--;
        prevTriggerdAngle = currentAngle;
    }

    // ダイアル操作はループさせる
    if(gObsDial > DIAL_MAX){
        gObsDial = DIAL_MIN;
    }else if(gObsDial < DIAL_MIN){
        gObsDial = DIAL_MAX;
    }
}

void TaskObservation(void *arg){
    // 観測データを加工するタスク


    ESP_LOGI(TAG, "Complete initialization.");
    while(1){
        batteryObservation();
        touchObservation();
        movingDistanceObservation();
        angleObservation();
        dialObservation();
        wallObservation();

        ESP_LOGD(TAG, "Dial: %d", gObsDial);

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

}
