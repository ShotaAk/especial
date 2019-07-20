
#include <stdio.h>
#include <math.h>
#include <time.h>
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
    float invertSpeed;
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
    const controlGain_t speedGain = {12.0, 0.0, 0.0}; // i= 0.1
    const controlGain_t omegaGain = {1.00, 0.01, 0.0}; // i = 0.01

    // 直進方向の速度更新
    if(control->forceSpeedEnable){
        // 強制的に目標速度を設定する
        if(control->invertSpeed){
            TargetSpeed = -control->forceSpeed;
        }else{
            TargetSpeed = control->forceSpeed;
        }
    }else{
        // 目標速度を更新する
        if(control->invertSpeed){
            TargetSpeed -= control->accelSpeed * 0.001; // 1 msec周期なので、加速度を1/1000倍する
        }else{
            TargetSpeed += control->accelSpeed * 0.001; // 1 msec周期なので、加速度を1/1000倍する
        }

        if(fabs(TargetSpeed) > control->maxSpeed){
            TargetSpeed = copysign(control->maxSpeed, TargetSpeed);
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

    const float ERROR_MAX = 1.0e+10;
    static float sumSpeedError = 0;
    static float sumOmegaError = 0;
    sumSpeedError += speedError;
    sumOmegaError += omegaError;
    // オーバフロー防止
    if(fabs(sumSpeedError) >= ERROR_MAX){
        sumSpeedError = copysign(ERROR_MAX, sumSpeedError);
    }
    if(fabs(sumOmegaError) >= ERROR_MAX){
        sumOmegaError = copysign(ERROR_MAX, sumOmegaError);
    }

    MotorVoltage[RIGHT] += (speedError * speedGain.Kp + sumSpeedError * speedGain.Ki);
    MotorVoltage[LEFT]  += (speedError * speedGain.Kp + sumSpeedError * speedGain.Ki);

    // 角速度値を入力
    MotorVoltage[RIGHT] -= (omegaError * omegaGain.Kp + sumOmegaError * omegaGain.Ki);
    MotorVoltage[LEFT]  += (omegaError * omegaGain.Kp + sumOmegaError * omegaGain.Ki);
    
    // printf("MotorVoltage, Error: %f, %f, %f\n", MotorVoltage[LEFT], MotorVoltage[RIGHT], omegaError);

    // 印加電圧リミット
    const float voltageLimit = 3.0; // voltage
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

int straight(const float targetDistance, const float timeout){
    const float MAX_SPEED = 0.5; // m/s
    const float MIN_SPEED = 0.2; // m/s
    const float ACCEL = 1.0; // m/s^2
    const float DECEL = -1.0; // m/s^2

    float remainingDistance = 0; // 残距離
    control_t control;
    control.maxSpeed = MAX_SPEED;
    control.maxOmega = 0;
    control.invertSpeed = 0;
    control.invertOmega = 0;
    control.accelSpeed = 0;
    control.accelOmega = 0;
    control.forceSpeedEnable = 0;
    control.forceOmegaEnable = 0; 

    if(targetDistance < 0){
        control.invertSpeed = 1;
    }

    // 移動距離を初期化
    gMovingDistance = 0;

    // 時間計測開始
    clock_t startTime = clock();

    // 加速・定速
    control.accelSpeed = ACCEL;
    while(1){
        remainingDistance = fabs(targetDistance - gMovingDistance) - 0.010;

        // 残距離が停止距離より短かったら加速をやめる
        if(remainingDistance <= TargetSpeed*TargetSpeed / (2.0*control.accelSpeed)){
            break;
        }

        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);

        // タイムアウトチェック
        if(timeout < (float)(clock() - startTime)/CLOCKS_PER_SEC){
            return 0;
        }
    }

    // 減速
    control.accelSpeed = DECEL;
    while(fabs(gMovingDistance) < fabs(targetDistance) - 0.001){
        // 一定速度まで減速したら、最低駆動トルクで走行
        if(fabs(TargetSpeed) <= MIN_SPEED){
            control.forceSpeed = MIN_SPEED;
            control.forceSpeedEnable = 1;
        }

        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);

        // タイムアウトチェック
        if(timeout < (float)(clock() - startTime)/CLOCKS_PER_SEC){
            return 0;
        }
    }


    // 速度が0以下になるまで制御を続ける
    while(fabs(gMeasuredSpeed) >= 0.01){
        control.forceSpeedEnable = 1;
        control.forceSpeed = 0;

        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);

        // タイムアウトチェック
        if(timeout < (float)(clock() - startTime)/CLOCKS_PER_SEC){
            return 0;
        }
    }

    return 1;
}

void turn(const float targetAngle){
    const float MAX_OMEGA= M_PI; // rad/s
    const float MIN_OMEGA= M_PI*0.02; // rad/s
    const float ACCEL = M_PI*2; // rad/s^2
    const float DECEL = -M_PI; // rad/s^2

    float remainingAngle = 0; // 残角度

    control_t control;
    control.maxSpeed = 0;
    control.maxOmega = MAX_OMEGA;
    control.invertSpeed = 0;
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
    while(fabs((gMeasuredAngle - startAngle)) < fabs(targetAngle) - M_PI*0.01){
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
    while(fabs(gGyro[AXIS_Z]) >= 0.01){
        control.forceOmega = 0;
        control.forceOmegaEnable = 1;

        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    // printf("finish\n");
}

void stop(const int times){
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

    for(int i=0;i<times;i++){
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
    const int waitTime = 500;
    const float TIME_OUT = 1.5; // sec

    while(1){
        switch(gControlRequest){
        case CONT_FORWARD:
            gMotorState = MOTOR_ON;
            straight(0.090, TIME_OUT);
            gControlRequest = CONT_NONE; // リクエスト受付開始
            stop(waitTime);
            break;

        case CONT_HALF_FORWARD:
            gMotorState = MOTOR_ON;
            straight(0.045, TIME_OUT);
            gControlRequest = CONT_NONE; // リクエスト受付開始
            stop(waitTime);
            break;

        case CONT_TURN_LEFT:
            gMotorState = MOTOR_ON;
            turn(M_PI_2);
            gControlRequest = CONT_NONE; // リクエスト受付開始
            stop(waitTime);
            break;

        case CONT_TURN_RIGHT:
            gMotorState = MOTOR_ON;
            turn(-M_PI_2);
            gControlRequest = CONT_NONE; // リクエスト受付開始
            stop(waitTime);
            break;

        case CONT_TURN_BACK:
            gMotorState = MOTOR_ON;
            turn(M_PI);
            gControlRequest = CONT_NONE; // リクエスト受付開始
            stop(waitTime);
            break;

        case CONT_KETSUATE:
            gMotorState = MOTOR_ON;
            straight(-0.030, TIME_OUT);
            stop(waitTime);
            straight(0.010, TIME_OUT);
            stop(waitTime);
            gControlRequest = CONT_NONE; // リクエスト受付開始
            stop(waitTime);
            break;

        case CONT_ENKAI:
            gMotorState = MOTOR_ON;
            doEnkai();
            gControlRequest = CONT_NONE; // リクエスト受付開始
            break;

        case CONT_NONE:
        case CONT_STOP:
            // リクエスト待機状態
            gMotorState = MOTOR_ON;
            stop(1);
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
