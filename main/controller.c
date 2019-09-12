
#include <stdio.h>
#include <math.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

#include "motion.h"
#include "variables.h"
#include "parameters.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
static const char *TAG="Controller";

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
    const controlGain_t omegaGain = {0.00, 0.000000, 0.0}; // i = 0.01

    // フィードフォワードパラメータ
    const float SPEED_FF_GAIN = 0;
    const float SPEED_ACCEL_FF_GAIN = 0;
    const float OMEGA_FF_GAIN = 0;
    const float OMEGA_ACCEL_FF_GAIN = 0;

    static float prevTargetSpeed = 0;
    static float prevTargetOmega = 0;

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

    /* -------------------フィードバック項の計算------------------------------*/
    float speedError = TargetSpeed - gObsSpeed;
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
    /* -----------------------------------------------------------------------*/

    // 直進速度のフィードフォワード
    float voltageSpeedFF = TargetSpeed * SPEED_FF_GAIN 
        + (TargetSpeed - prevTargetSpeed) * SPEED_ACCEL_FF_GAIN;
    MotorVoltage[RIGHT] += voltageSpeedFF;
    MotorVoltage[LEFT]  += voltageSpeedFF;
    // 角速度のフィードフォワード
    float voltageOmegaFF = TargetOmega * OMEGA_FF_GAIN 
        + (TargetOmega - prevTargetOmega) * OMEGA_ACCEL_FF_GAIN;;
    MotorVoltage[RIGHT] -= voltageOmegaFF;
    MotorVoltage[LEFT]  += voltageOmegaFF;

    // 直進速度のフィードバック
    float voltageSpeedFB = speedError * speedGain.Kp
        + sumSpeedError * speedGain.Ki;
    MotorVoltage[RIGHT] += voltageSpeedFB;
    MotorVoltage[LEFT]  += voltageSpeedFB;
    // 角速度のフィードバック
    float voltageOmegaFB = omegaError * omegaGain.Kp
        + sumOmegaError * omegaGain.Ki;
    MotorVoltage[RIGHT] -= voltageOmegaFB;
    MotorVoltage[LEFT]  += voltageOmegaFB;

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

    // 目標速度を保存（デバッグ用）
    gTargetSpeed = TargetSpeed;
    // 目標速度を保存（フィードフォワード用）
    prevTargetSpeed = TargetSpeed;
    prevTargetSpeed = TargetOmega;
}

int straight(const float targetDistance, const float endSpeed, const float timeout){
    // 到達地点で速度がendSpeedになる直線走行
    // 台形制御
    const float MAX_SPEED = 0.5; // m/s
    const float MIN_SPEED = 0.2; // m/s
    const float ACCEL = 1.0; // m/s^2
    const float DECEL = -1.0; // m/s^2

    control_t control;
    control.maxSpeed = MAX_SPEED;
    control.maxOmega = 0;
    control.invertSpeed = 0;
    control.invertOmega = 0;
    control.accelSpeed = 0;
    control.accelOmega = 0;
    control.forceSpeedEnable = 0;
    control.forceOmegaEnable = 0; 

    // TODO:逆走機能を設ける
    if(targetDistance < 0 || endSpeed < 0 || timeout < 0){
        // control.invertSpeed = 1;
        ESP_LOGE(TAG, "Invalid arguments, targetDistance:%f, endSpeed:%f, timeout:%f", 
                targetDistance, endSpeed, timeout);
        return FALSE;
    }

    // 移動距離を初期化
    gObsMovingDistance = 0;

    // 時間計測開始
    clock_t startTime = clock();

    // 加速・定速
    control.accelSpeed = ACCEL;
    while(1){
        // 目標位置までの残り移動距離
        // TODO:逆走機能を設ける
        float remainingDistance = targetDistance - gObsMovingDistance;
        if(remainingDistance < 0){
            ESP_LOGE(TAG, "targetDistance < gObsMovingDistance");
            return FALSE;
        }
        // 目標最終速度までの残り速度
        float remainingSpeed = TargetSpeed - endSpeed;
        // 減速に必要な距離
        float brakingDistance = 
            remainingSpeed //高さ
            * (remainingSpeed / control.accelSpeed) // 底辺
            * 0.5; // 三角形の面積

        // 残距離が減速距離より短かったら加速ループを抜ける
        if(remainingDistance <= brakingDistance){
            break;
        }

        // 制御器の更新
        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);

        // タイムアウトチェック
        if(timeout < (float)(clock() - startTime)/CLOCKS_PER_SEC){
            ESP_LOGE(TAG, "Timeout at accelereation");
            return FALSE;
        }
    }

    // 減速
    // TODO:逆走機能を設ける
    float stopDistance = 0.0;
    int stopControlEnable = FALSE;
    if(endSpeed < MIN_SPEED){
        // 終端速度が最低駆動トルクの速度より小さい場合は、停止用の距離を設ける
        stopDistance = 0.001;
        stopControlEnable = TRUE;
    }
    control.accelSpeed = DECEL;
    while(gObsMovingDistance < (targetDistance - stopDistance)){
        // 制御速度が終端速度よりも小さくなったら、加速度(減速度)を0にする
        if(TargetSpeed <= endSpeed){
            control.accelSpeed = 0;
        }
        // 制御速度が最低駆動トルクの速度より小さくなったら、目標速度を固定する
        if(TargetSpeed < MIN_SPEED){
            control.forceSpeedEnable = 1;
            control.forceSpeed = MIN_SPEED;
        }

        // 制御器の更新
        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);

        // タイムアウトチェック
        if(timeout < (float)(clock() - startTime)/CLOCKS_PER_SEC){
            ESP_LOGE(TAG, "Timeout at deceleration");
            return FALSE;
        }
    }

    // 停止
    // TODO:逆走機能を設ける
    const float SPEED_MARGIN = 0.01; // m/s ピッタリ速度を合わせるのは難しいので
    if(stopControlEnable){
        // 終端速度に達するまで制御を続ける
        while(fabs(gObsSpeed - endSpeed) >= SPEED_MARGIN){
            // 強制的に速度を0 m/sにする
            control.forceSpeedEnable = 1;
            control.forceSpeed = 0;

            // 制御器の更新
            updateController(&control);
            vTaskDelay(1 / portTICK_PERIOD_MS);

            // タイムアウトチェック
            if(timeout < (float)(clock() - startTime)/CLOCKS_PER_SEC){
                ESP_LOGE(TAG, "Timeout at stop");
                return FALSE;
            }
        }
    }

    return TRUE;
}

int straightAndStop(const float targetDistance, const float timeout){
    // 到達地点で速度が0になる直線走行
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
    gObsMovingDistance = 0;

    // 時間計測開始
    clock_t startTime = clock();

    // 加速・定速
    control.accelSpeed = ACCEL;
    while(1){
        remainingDistance = fabs(targetDistance - gObsMovingDistance) - 0.010;

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
    while(fabs(gObsMovingDistance) < fabs(targetDistance) - 0.001){
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
    while(fabs(gObsSpeed) >= 0.01){
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

    float startAngle = gObsAngle; // 制御開始前の角度取得

    if(targetAngle < 0){
        control.invertOmega = 1;
    }

    // 加速・定速
    control.accelOmega = ACCEL;
    while(1){
        remainingAngle = fabs(targetAngle - (gObsAngle - startAngle));

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
    while(fabs((gObsAngle - startAngle)) < fabs(targetAngle) - M_PI*0.01){
        // 一定速度まで減速したら、最低駆動トルクで走行
        if(fabs(TargetOmega) <= MIN_OMEGA){
            control.forceOmega = MIN_OMEGA;
            control.forceOmegaEnable = 1;
        }

        // printf("moveAngle %f\n", gObsAngle - startAngle);
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
    const float TIME_OUT = 0.5; // sec

    while(1){
        switch(gControlRequest){
        case CONT_FORWARD:
            gMotorState = MOTOR_ON;
            // straightAndStop(0.090, TIME_OUT);
            // gControlRequest = CONT_NONE; // リクエスト受付開始
            // stop(waitTime);

            straight(0.090, 0.2, TIME_OUT);
            gControlRequest = CONT_NONE; // リクエスト受付開始
            break;

        case CONT_HALF_FORWARD:
            gMotorState = MOTOR_ON;
            // straightAndStop(0.045, TIME_OUT);
            // gControlRequest = CONT_NONE; // リクエスト受付開始
            // stop(waitTime);
            
            straight(0.040, 0.2, TIME_OUT);
            gControlRequest = CONT_NONE; // リクエスト受付開始
            
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
            straightAndStop(-0.045, TIME_OUT);
            stop(waitTime);
            straightAndStop(0.005, TIME_OUT);
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
