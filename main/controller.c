
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
    float invertOmega;
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
    const controlGain_t speedGain = {20.0, 0.0, 0.0};
    const controlGain_t omegaGain = {1.60, 0.0, 0.0};

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
        if(control->invertOmega){
            TargetOmega = -control->forceOmega;
        }else{
            TargetOmega = control->forceOmega;
        }
    }else{
        // 目標速度を更新する
        if(control->invertOmega){
            TargetOmega -= control->accelOmega* 0.001; // 1 msec周期なので、加速度を1/1000倍する
        }else{
            TargetOmega += control->accelOmega* 0.001; // 1 msec周期なので、加速度を1/1000倍する

        }

        if(fabs(TargetOmega) > control->maxOmega){
            TargetOmega = copysign(control->maxOmega, TargetOmega);
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
    control.invertOmega = 0;
    control.accelSpeed = 0;
    control.accelOmega = 0;
    control.forceSpeedEnable = 0;
    control.forceOmegaEnable = 0; 

    // 移動距離を初期化
    gMovingDistance = 0;

    // 加速・定速
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

void turn(const float targetAngle){
    const float MAX_OMEGA= M_PI; // rad/s
    const float MIN_OMEGA= M_PI*0.05; // rad/s
    const float ACCEL = M_PI*2; // rad/s^2
    const float DECEL = -M_PI; // rad/s^2

    float remainingAngle = 0; // 残角度

    control_t control;
    control.maxSpeed = 0;
    control.maxOmega = MAX_OMEGA;
    control.invertOmega = 0;
    control.accelSpeed = 0;
    control.accelOmega = 0;
    control.forceSpeedEnable = 0;
    control.forceOmegaEnable = 0; 

    float startAngle = gMeasuredAngle; // 制御開始前の角度取得

    if(targetAngle < 0){
        control.invertOmega = 1;
    }

    // 加速・定速
    control.accelOmega = ACCEL;
    while(1){
        remainingAngle = fabs(targetAngle - (gMeasuredAngle - startAngle));

        // 残角度が停止角度より短かったら加速をやめる
        if(remainingAngle <= TargetOmega*TargetOmega / (2.0*control.accelOmega)){
            break;
        }

        // printf("remainingAngle : %f\n", remainingAngle);
        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // 減速
    control.accelOmega = DECEL;
    while(fabs((gMeasuredAngle - startAngle)) < fabs(targetAngle)){
        // 一定速度まで減速したら、最低駆動トルクで走行
        if(fabs(TargetOmega) <= MIN_OMEGA){
            control.forceOmega = MIN_OMEGA;
            control.forceOmegaEnable = 1;
        }

        // printf("moveAngle %f\n", gMeasuredAngle - startAngle);
        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // 速度が0以下になるまで制御を続ける
    while(fabs(gGyro[AXIS_Z]) >= 0.05){
        control.forceOmega = 0;
        control.forceOmegaEnable = 1;

        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    // printf("finish\n");
}

void stop(void){
    // その場にとどまる

    control_t control;
    control.maxSpeed = 0;
    control.maxOmega = 0;
    control.accelSpeed = 0;
    control.accelOmega = 0;
    control.forceSpeedEnable = 1;
    control.forceOmegaEnable = 1; 
    control.forceSpeed = 0;
    control.forceOmega = 0;

    updateController(&control);
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
        switch(gControlRequest){
        case CONT_FORWARD:
            gControlRequest = CONT_NONE; // リクエスト受付開始
            gMotorState = MOTOR_ON;
            goForward(0.090);
            stop();
            break;

        case CONT_TURN_LEFT:
            gControlRequest = CONT_NONE; // リクエスト受付開始
            gMotorState = MOTOR_ON;
            turn(M_PI_2);
            stop();
            break;

        case CONT_TURN_RIGHT:
            gControlRequest = CONT_NONE; // リクエスト受付開始
            gMotorState = MOTOR_ON;
            turn(-M_PI_2);
            stop();
            break;

        case CONT_TURN_BACK:
            gControlRequest = CONT_NONE; // リクエスト受付開始
            gMotorState = MOTOR_ON;
            turn(M_PI);
            stop();
            break;

        case CONT_ENKAI:
            gControlRequest = CONT_NONE; // リクエスト受付開始
            gMotorState = MOTOR_ON;
            doEnkai();
            break;

        case CONT_NONE:
        case CONT_STOP:
            // リクエスト待機状態
            gMotorState = MOTOR_ON;
            stop();
            break;

        case CONT_FINISH:
        default:
            // 制御終了状態
            gMotorState = MOTOR_OFF;
            gMotorDuty[RIGHT] = 0;
            gMotorDuty[LEFT] = 0;
            break;

        }

        // printf("07_TaskControlMotion\n");
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

}
