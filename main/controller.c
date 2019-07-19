
#include <stdio.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "motion.h"
#include "variables.h"
#include "parameters.h"

static float TargetSpeed = 0;
static float TargetOmega = 0;

typedef struct{
    float maxSpeed;
    float maxOmega;
    float accelSpeed;
    float accelOmega;
    int forceSpeedEnable;
    int forceOmegaEnable;
    float forceSpeed;
    float forceOmega;

}control_t;

typedef struct{
    float Kp;
    float Ki;
    float Kd;

}controlGain_t;

void updateController(control_t *control){
    const controlGain_t speedGain = {3.5, 0.0, 0.0};
    const controlGain_t omegaGain = {0.40, 0.0, 0.0};

    // 直進方向の速度更新
    if(control->forceSpeedEnable){
        // 強制的に目標速度を設定する
        TargetSpeed = control->forceSpeed;
    }else{
        // 目標速度を更新する
        TargetSpeed += control->accelSpeed * 0.001; // 1 msec周期なので、加速度を1/1000倍する

        if(TargetSpeed > control->maxSpeed){
            TargetSpeed = control->maxSpeed;
        }
    }
    // 回転方向の速度更新
    if(control->forceOmegaEnable){
        // 強制的に目標速度を設定する
        TargetOmega = control->forceOmega;
    }else{
        // 目標速度を更新する
        TargetOmega += control->accelOmega* 0.001; // 1 msec周期なので、加速度を1/1000倍する

        if(TargetOmega > control->maxOmega){
            TargetOmega = control->maxOmega;
        }
    }


    // 目標速度をタイヤの速度に変換する
    float MotorVoltage[SIDE_NUM] = {0};

    // フィードバック制御
    float speedError = TargetSpeed - gMeasuredSpeed;
    float omegaError = TargetOmega - gGyro[AXIS_Z];

    MotorVoltage[RIGHT] += speedError * speedGain.Kp;
    MotorVoltage[LEFT]  += speedError * speedGain.Kp;

    // 角速度値を入力
    MotorVoltage[RIGHT] -= omegaError * omegaGain.Kp;
    MotorVoltage[LEFT]  += omegaError * omegaGain.Kp;
    
    // printf("MotorVoltage, Error: %f, %f, %f\n", MotorVoltage[LEFT], MotorVoltage[RIGHT], omegaError);

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

void goForward(const float targetDistance){
    const float MAX_SPEED = 0.5; // m/s
    const float MIN_SPEED = 0.2; // m/s
    const float ACCEL = 1.0; // m/s^2
    const float DECEL = -1.0; // m/s^2

    float remainingDistance = 0; // 残距離
    control_t control;
    control.maxSpeed = MAX_SPEED;
    control.maxOmega = 0;
    control.accelOmega = 0;
    control.forceSpeedEnable = 0;
    control.forceOmegaEnable = 0; 

    // 移動距離を初期化
    gMovingDistance = 0;

    // 加速・低速
    control.accelSpeed = ACCEL;
    while(1){
        remainingDistance = targetDistance - gMovingDistance - 0.010;

        // 残距離が停止距離より短かったら加速をやめる
        if(remainingDistance <= TargetSpeed*TargetSpeed / (2.0*control.accelSpeed)){
            break;
        }


        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // 減速
    control.accelSpeed = DECEL;
    while(gMovingDistance < targetDistance - 0.001){

        
        // 一定速度まで減速したら、最低駆動トルクで走行
        if(TargetSpeed <= MIN_SPEED){
            control.forceSpeed = MIN_SPEED;
            control.forceSpeedEnable = 1;
        }

        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }


    // 速度が0以下になるまで制御を続ける
    while(gMeasuredSpeed >= 0.0){
        control.forceSpeedEnable = 1;
        control.forceSpeed = 0;

        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void doEnkai(void){
    // 宴会芸

    control_t control;
    control.maxSpeed = 0;
    control.maxOmega = 0;
    control.accelSpeed = 0;
    control.accelOmega = 0;
    control.forceSpeedEnable = 1;
    control.forceOmegaEnable = 1; 
    control.forceSpeed = 0;
    control.forceOmega = 0;


    while(1){
        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void TaskControlMotion(void *arg){

    while(1){
        if(gControlRequest == CONT_FORWARD){
            gMotorState = MOTOR_ON;
            goForward(0.090);
            gControlRequest = CONT_NONE;
        }else if(gControlRequest == CONT_ENKAI){
            gMotorState = MOTOR_ON;
            doEnkai();
            gControlRequest = CONT_NONE;
        }else{
            gMotorState = MOTOR_ON;
            gMotorDuty[RIGHT] = 0;
            gMotorDuty[LEFT] = 0;
        }

        // printf("07_TaskControlMotion\n");
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

}
