
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

static float WALL_THRESHOLD[DIREC_NUM] = {
    [DIREC_FRONT] = 0.148355 - 0.03, 
    [DIREC_LEFT]  = 0.213455 - 0.03, 
    [DIREC_RIGHT] = 0.191652 - 0.03, 
    [DIREC_BACK]  = 0.5}; // volts


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
    const float TOUCH_THRESH_VOLTAGE = 1.0; // Volts

    if(gObjVoltages[OBJ_SENS_FL] > TOUCH_THRESH_VOLTAGE){
        gObsTouch[LEFT] = TRUE;
    }else{
        gObsTouch[LEFT] = FALSE;
    }

    if(gObjVoltages[OBJ_SENS_FR] > TOUCH_THRESH_VOLTAGE){
        gObsTouch[RIGHT] = TRUE;
    }else{
        gObsTouch[RIGHT] = FALSE;
    }
}

void updateWallThresholds(void){
    // 壁センサのしきい値を更新する
    const int DATA_NUM = 1000;
    const int WAIT_TIME = 5000;
    float sum=0;

    // 右壁
    // 準備時間
    gIndicatorValue = 4;
    vTaskDelay(WAIT_TIME / portTICK_PERIOD_MS);

    gIndicatorValue = 7;
    for(int i=0;i<DATA_NUM; i++){
        sum += gObjVoltages[OBJ_SENS_R];
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    WALL_THRESHOLD[DIREC_RIGHT] = sum / (float)DATA_NUM;
    gObsWallThresholds[DIREC_RIGHT] = WALL_THRESHOLD[DIREC_RIGHT];
    

    // 左壁
    sum=0;
    gIndicatorValue = 5;
    vTaskDelay(WAIT_TIME / portTICK_PERIOD_MS);

    gIndicatorValue = 8;
    for(int i=0;i<DATA_NUM; i++){
        sum += gObjVoltages[OBJ_SENS_L];
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    WALL_THRESHOLD[DIREC_LEFT] = sum / (float)DATA_NUM;
    gObsWallThresholds[DIREC_LEFT] = WALL_THRESHOLD[DIREC_LEFT];

    // 前両壁
    sum=0;
    gIndicatorValue = 6;
    vTaskDelay(WAIT_TIME / portTICK_PERIOD_MS);

    gIndicatorValue = 9;
    for(int i=0;i<DATA_NUM; i++){
        sum += (gObjVoltages[OBJ_SENS_FR] + gObjVoltages[OBJ_SENS_FL]);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    WALL_THRESHOLD[DIREC_FRONT] = sum / ((float)DATA_NUM * 2.0);
    gObsWallThresholds[DIREC_FRONT] = WALL_THRESHOLD[DIREC_FRONT];
    gIndicatorValue = 0;

}

void wallObservation(void){
    // Object Sensorで壁を検出する

    if(gObjVoltages[OBJ_SENS_L] > WALL_THRESHOLD[DIREC_LEFT]){
        gObsIsWall[DIREC_LEFT] = TRUE;
    }else{
        gObsIsWall[DIREC_LEFT] = FALSE;
    }

    if(gObjVoltages[OBJ_SENS_R] > WALL_THRESHOLD[DIREC_RIGHT]){
        gObsIsWall[DIREC_RIGHT] = TRUE;
    }else{
        gObsIsWall[DIREC_RIGHT] = FALSE;
    }

    if( (gObjVoltages[OBJ_SENS_FL] + gObjVoltages[OBJ_SENS_FR]) * 0.5 
            > WALL_THRESHOLD[DIREC_FRONT]){
        gObsIsWall[DIREC_FRONT] = TRUE;
    }else{
        gObsIsWall[DIREC_FRONT] = FALSE;
    }
}

void wallErrorObservation(void){
    // 壁制御用にエラー値を出力する
    gObsWallError[RIGHT] = gObjVoltages[OBJ_SENS_R] - WALL_THRESHOLD[DIREC_RIGHT];
    gObsWallError[LEFT] = gObjVoltages[OBJ_SENS_L] - WALL_THRESHOLD[DIREC_LEFT];
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
    
    const float THRESH = M_PI * 0.25;
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
        wallErrorObservation();

        ESP_LOGD(TAG, "Dial: %d", gObsDial);

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

}
