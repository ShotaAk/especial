
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "motion.h"
#include "variables.h"
#include "parameters.h"

static float TargetSpeed = 0;
static float MotorVoltage[SIDE_NUM] = {0};


void updateController(const float maxSpeed, const float accel_mps, 
        const int forceSpeedEnable, const float forceSpeed){

    const float Kp = 20.0;
    const float Kp_angle = 0.20;

    if(forceSpeedEnable){
        // 強制的に目標速度を設定する
        TargetSpeed = forceSpeed;
    }else{
        // 目標速度を更新する
        TargetSpeed += accel_mps * 0.001; // 1 msec周期なので、加速度を1/1000倍する

        if(TargetSpeed > maxSpeed){
            TargetSpeed = maxSpeed;
        }
    }

    // 目標速度をタイヤの速度に変換する

    // フィードバック制御
    MotorVoltage[RIGHT] = (TargetSpeed - gMeasuredSpeed) * Kp;
    MotorVoltage[LEFT] = (TargetSpeed - gMeasuredSpeed) * Kp;

    // 角速度値を入力
    MotorVoltage[RIGHT] += gGyro[AXIS_Z] * Kp_angle;
    MotorVoltage[LEFT]  += -gGyro[AXIS_Z] * Kp_angle;
    

    // 印加電圧リミット
    const float voltageLimit = 2.0; // voltage
    for(int side_i=0; side_i<SIDE_NUM; side_i++){
        if(MotorVoltage[side_i] > voltageLimit){
            MotorVoltage[side_i] = voltageLimit;
        }else if(MotorVoltage[side_i] < -voltageLimit){
            MotorVoltage[side_i] = -voltageLimit;
        }
    }

    // バッテリー電圧を元にデューティを計算
    gMotorDuty[RIGHT] = 100.0 * MotorVoltage[RIGHT] / gBatteryVoltage;
    gMotorDuty[LEFT] = 100.0 * MotorVoltage[LEFT] / gBatteryVoltage;
}

void goForward(void){
    const float targetDistance = 0.090; // meters
    const float maxSpeed = 0.3; // m/s
    const float minSpeed = 0.1; // m/s
    const float ACCEL = 1.0; // m/s^2
    const float DECEL = -1.0; // m/s^2

    float remainingDistance = 0; // 残距離

    // 移動距離を初期化
    gMovingDistance = 0;

    // 加速・低速
    float targetAccel = ACCEL;
    while(1){
        remainingDistance = targetDistance - gMovingDistance - 0.010;

        // 残距離が停止距離より短かったら加速をやめる
        if(remainingDistance <= TargetSpeed*TargetSpeed / (2.0*targetAccel)){
            break;
        }

        updateController(maxSpeed, targetAccel,0,0);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // 減速
    targetAccel = DECEL;
    float forceSpeed = 0;
    float forceSpeedEnable = 0;
    while(gMovingDistance < targetDistance - 0.001){

        
        // 一定速度まで減速したら、最低駆動トルクで走行
        if(TargetSpeed <= minSpeed){
            forceSpeed = minSpeed;
            forceSpeedEnable = 1;
        }

        updateController(maxSpeed, targetAccel, forceSpeedEnable, forceSpeed);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }


    // 速度が0以下になるまで制御を続ける
    while(gMeasuredSpeed >= 0.0){
        forceSpeedEnable = 1;
        forceSpeed = 0;

        updateController(0, 0, forceSpeedEnable, forceSpeed);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void TaskControlMotion(void *arg){

    while(1){
        if(gControlRequest == CONT_FORWARD){
            gMotorState = MOTOR_ON;
            goForward();
            gControlRequest = CONT_NONE;
        }else{
            gMotorState = MOTOR_OFF;
            gMotorDuty[RIGHT] = 0;
            gMotorDuty[LEFT] = 0;
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

}
