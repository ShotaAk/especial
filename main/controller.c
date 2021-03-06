
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
    float enableWallControl;
    int initializeSumOfError;

}control_t;

typedef struct{
    float Kp;
    float Ki;
    float Kd;

}controlGain_t;

void updateController(control_t *control){
    const controlGain_t speedGain = {6.0, 0.0, 0.0}; // i= 0.1
    const controlGain_t omegaGain = {0.50, 0.00002, 0.0}; // p = 0.5

    // フィードフォワードパラメータ
    const float SPEED_FF_GAIN = 1.5;
    const float SPEED_ACCEL_FF_GAIN = 0.0;
    const float OMEGA_FF_GAIN = 0.02;
    const float OMEGA_ACCEL_FF_GAIN = 0;
    const float OMEGA_WALL_GAIN = 0.5;

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

    /* --------フィードフォワード＆フィードバック項の計算---------------------*/
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

    // 差分の初期化
    if(control->initializeSumOfError){
        sumSpeedError = 0;
        sumOmegaError = 0;
    }

    // 直進速度のフィードフォワード
    float voltageSpeedFF = TargetSpeed * SPEED_FF_GAIN 
        + (TargetSpeed - prevTargetSpeed) * SPEED_ACCEL_FF_GAIN;
    MotorVoltage[RIGHT] += voltageSpeedFF;
    MotorVoltage[LEFT]  += voltageSpeedFF;
    // 角速度のフィードフォワード
    float voltageOmegaFF = TargetOmega * OMEGA_FF_GAIN 
        + (TargetOmega - prevTargetOmega) * OMEGA_ACCEL_FF_GAIN;;
    MotorVoltage[RIGHT] += voltageOmegaFF;
    MotorVoltage[LEFT]  -= voltageOmegaFF;

    // 直進速度のフィードバック
    float voltageSpeedFB = speedError * speedGain.Kp
        + sumSpeedError * speedGain.Ki;
    MotorVoltage[RIGHT] += voltageSpeedFB;
    MotorVoltage[LEFT]  += voltageSpeedFB;
    // 角速度のフィードバック
    float voltageOmegaFB = omegaError * omegaGain.Kp
        + sumOmegaError * omegaGain.Ki;
    MotorVoltage[RIGHT] += voltageOmegaFB;
    MotorVoltage[LEFT]  -= voltageOmegaFB;
    // 壁制御のフィードバック
    if(control->enableWallControl){
        // 左右に壁があるときのみ、壁制御を実施する
        float wallError = 0;
        if(gObsIsWall[DIREC_RIGHT]){
            wallError += gObsWallError[RIGHT];
        }
        if(gObsIsWall[DIREC_LEFT]){
            wallError -= gObsWallError[LEFT];
        }
        // }
        // if(gObsIsWall[DIREC_RIGHT] && gObsIsWall[DIREC_LEFT]){
        //     float wallError = gObsWallError[RIGHT] - gObsWallError[LEFT];
        //     float voltageWallFB = wallError * OMEGA_WALL_GAIN;
        //     MotorVoltage[RIGHT] += voltageWallFB;
        //     MotorVoltage[LEFT]  -= voltageWallFB;
        // }
        float voltageWallFB = wallError * OMEGA_WALL_GAIN;
        MotorVoltage[RIGHT] += voltageWallFB;
        MotorVoltage[LEFT]  -= voltageWallFB;
    }

    /* -----------------------------------------------------------------------*/

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
    gTargetOmega = TargetOmega;
    // 目標速度を保存（フィードフォワード用）
    prevTargetSpeed = TargetSpeed;
    prevTargetSpeed = TargetOmega;
}

int straight(const float targetDistance, const float endSpeed, const float timeout,
        const float maxSpeed, const float accel){
    // 到達地点で速度がendSpeedになる直線走行
    // 台形制御
    // const float MAX_SPEED = 0.5; // m/s
    const float MIN_SPEED = 0.2; // m/s
    // const float ACCEL = 1.0; // m/s^2
    // const float DECEL = -1.0; // m/s^2

    float MAX_SPEED = maxSpeed;
    float ACCEL = accel;
    float DECEL = -accel;

    control_t control;
    control.maxSpeed = MAX_SPEED;
    control.maxOmega = 0;
    control.invertSpeed = 0;
    control.invertOmega = 0;
    control.accelSpeed = 0;
    control.accelOmega = 0;
    control.forceSpeedEnable = 0;
    control.forceOmegaEnable = 0; 
    control.enableWallControl = 1;

    // TODO:逆走機能を設ける
    if(targetDistance < 0 || endSpeed < 0 || timeout < 0){
        // control.invertSpeed = 1;
        ESP_LOGE(TAG, "Invalid arguments, targetDistance:%f, endSpeed:%f, timeout:%f", 
                targetDistance, endSpeed, timeout);
        return FALSE;
    }

    // 移動距離を初期化
    // gObsMovingDistance = 0;

    // 時間計測開始
    clock_t startTime = clock();

    // 加速・定速
    control.accelSpeed = ACCEL;
    while(1){
        // 目標位置までの残り移動距離
        // endSpeedとTargetSpeedが等しい場合は、この条件分岐で関数を抜ける
        // TODO:逆走機能を設ける
        float remainingDistance = targetDistance - gObsMovingDistance;
        if(remainingDistance < 0){
            // 移動距離を初期化
            // 関数の終了時に初期化することで、
            // 関数外の処理中に進んだ距離を計測できる
            gObsMovingDistance = 0;
            return TRUE;
        }
        // 目標最終速度までの残り速度
        float remainingSpeed = TargetSpeed - endSpeed;
        // 減速にかかる時間
        float brakingTime = remainingSpeed / fabs(DECEL); // 減速度の大きさだけ取る
        // 減速に必要な距離
        float brakingDistance = 
            remainingSpeed * brakingTime * 0.5 // 三角形の面積
            + endSpeed * brakingTime; // 四角形の面積

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
        stopDistance = 0.005;
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
    const float SPEED_MARGIN = 0.01; // 0 m/s ピッタリ速度を合わせるのは難しいので
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

    // 移動距離を初期化
    // 関数の終了時に初期化することで、
    // 関数外の処理中に進んだ距離を計測できる
    gObsMovingDistance = 0;

    return TRUE;
}


int turn(const float targetAngle, const float timeout){
    // 到達角度で速度が0になる超信地旋回
    // 台形制御
    const float MAX_OMEGA= 8; // rad/s
    const float MIN_OMEGA= M_PI*0.3; // rad/s
    const float ACCEL = 100; // rad/s^2
    const float DECEL = -100; // rad/s^2

    const float END_OMEGA = 0; // 終端角速度を0 rad/s固定にする

    control_t control;
    control.maxSpeed = 0;
    control.maxOmega = MAX_OMEGA;
    control.invertSpeed = 0;
    control.invertOmega = 0;
    control.accelSpeed = 0;
    control.accelOmega = 0;
    control.forceSpeedEnable = 0;
    control.forceOmegaEnable = 0; 
    control.enableWallControl = 0;

    float startAngle = gObsAngle; // 制御開始前の角度取得

    // 時間計測開始
    clock_t startTime = clock();

    if(targetAngle < 0){
        control.invertOmega = 1;
    }

    // 加速・定速
    control.accelOmega = ACCEL;
    while(1){
        // 目標角度までの残り回転角度
        float remainingAngle = fabs(targetAngle - (gObsAngle - startAngle));

        // 目標最終角速度(0 rad/s)までの残り速度
        float remainingOmega = TargetOmega - END_OMEGA;
        // 減速にかかる時間
        float brakingTime = remainingOmega / fabs(DECEL); // 減速度の大きさだけ取る
        // 減速に必要な回転角度
        float brakingAngle = 
            remainingOmega * brakingTime * 0.5 // 三角系の面積
            + END_OMEGA * brakingTime; // 四角形の面積

        // 残角度が停止角度より短かったら加速ループを抜ける
        if(remainingAngle <= brakingAngle){
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
    control.accelOmega = DECEL;
    const float STOP_ANGLE = M_PI*0.01; // 停止用の角度
    while(fabs((gObsAngle - startAngle)) < fabs(targetAngle) - STOP_ANGLE){
        // 一定速度まで減速したら、最低駆動トルクで走行
        if(fabs(TargetOmega) <= MIN_OMEGA){
            control.forceOmega = MIN_OMEGA;
            control.forceOmegaEnable = 1;
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

    // 速度が0以下になるまで制御を続ける
    const float OMEGA_MARGIN = 0.01; // 0 rad/s ピッタリ速度を合わせるのは難しいので
    while(fabs(gGyro[AXIS_Z]) >= OMEGA_MARGIN){
        control.forceOmega = 0;
        control.forceOmegaEnable = 1;

        // 制御器の更新
        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);

        // タイムアウトチェック
        if(timeout < (float)(clock() - startTime)/CLOCKS_PER_SEC){
            ESP_LOGE(TAG, "Timeout at stop");
            return FALSE;
        }
    }

    // 移動距離を初期化
    // 関数の終了時に初期化することで、
    // 関数外の処理中に進んだ距離を計測できる
    gObsMovingDistance = 0;

    return TRUE;
}

int slalomBase(const int isTurnRight, const float endSpeed, const float timeout,
        const float MAX_OMEGA, const float ACCEL_DECEL,
        const float START_OFFSET_DISTANCE, const float STOP_OFFSET_DISTANCE,
        const float ACCEL_DECEL_ANGLE, const float KEEP_OMEGA_ANGLE){

    // スラロームやりたい
    control_t control;
    // 直進速度は一定速
    control.maxSpeed = endSpeed;
    control.invertSpeed = 0;
    control.accelSpeed = 0;
    control.forceSpeedEnable = 1;
    control.forceSpeed = endSpeed;
    // 角速度を変化させる
    control.maxOmega = MAX_OMEGA;
    control.invertOmega = 0;
    control.accelOmega = 0;
    control.forceOmegaEnable = 0; 
    // 壁制御はしない
    control.enableWallControl = 0;

    if(isTurnRight){
        control.invertOmega = 1;
    }

    float startAngle = gObsAngle; // 制御開始前の角度取得
    // 時間計測開始
    clock_t startTime = clock();

    // オフセット距離を走行
    // gObsMovingDistance = 0; // 移動距離を初期化
    while(1){
        // 走行距離がオフセット距離を超えたらループを抜ける
        if(gObsMovingDistance > START_OFFSET_DISTANCE){
            break;
        }

        // 制御器の更新
        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);

        // タイムアウトチェック
        if(timeout < (float)(clock() - startTime)/CLOCKS_PER_SEC){
            ESP_LOGE(TAG, "Timeout at offset");
            return FALSE;
        }
    }

    // 一定角度に達するまで加速
    control.accelOmega = ACCEL_DECEL;
    while(1){
        // 回転角度が一定角度を超えたらループを抜ける
        if(fabs(gObsAngle - startAngle) > ACCEL_DECEL_ANGLE){
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

    // 一定角度に達するまで低速
    control.accelOmega = 0;
    while(1){
        // 回転角度が一定角度を超えたらループを抜ける
        if(fabs(gObsAngle - startAngle) > ACCEL_DECEL_ANGLE+KEEP_OMEGA_ANGLE){
            break;
        }

        // 制御器の更新
        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);

        // タイムアウトチェック
        if(timeout < (float)(clock() - startTime)/CLOCKS_PER_SEC){
            ESP_LOGE(TAG, "Timeout at keeping omega");
            return FALSE;
        }
    }

    // 角速度が０以下になるまで減速
    control.accelOmega = -ACCEL_DECEL;
    while(1){
        // 回転角度が一定角度を超えたらループを抜ける
        if(fabs(gObsAngle - startAngle) > 2.0*ACCEL_DECEL_ANGLE+KEEP_OMEGA_ANGLE){
            break;
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

    // オフセット距離を走行
    // 角速度を0にする
    control.accelOmega = 0;
    control.forceOmegaEnable = 1;
    control.forceOmega = 0;
    gObsMovingDistance = 0; // 移動距離を初期化
    while(1){
        // 走行距離がオフセット距離を超えたらループを抜ける
        if(gObsMovingDistance > STOP_OFFSET_DISTANCE){
            break;
        }

        // 制御器の更新
        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);

        // タイムアウトチェック
        if(timeout < (float)(clock() - startTime)/CLOCKS_PER_SEC){
            ESP_LOGE(TAG, "Timeout at offset");
            return FALSE;
        }
    }

    // 移動距離を初期化
    // 関数の終了時に初期化することで、
    // 関数外の処理中に進んだ距離を計測できる
    gObsMovingDistance = 0;

    return TRUE;
}


int slalom(const int isTurnRight, const float endSpeed, const float timeout){
    // 探索走行用のスラローム
    const float MAX_OMEGA= 8; // 最大角速度 rad/s
    const float ACCEL_DECEL = 100; // 50 加減速度 rad/s^2
    const float START_OFFSET_DISTANCE = 0.010; // オフセット直線走行距離 meter
    const float STOP_OFFSET_DISTANCE = 0.018; // オフセット直線走行距離 meter
    const float ACCEL_DECEL_ANGLE = 15.0 * M_PI / 180.0; // 25.0 ->  0.436332313; // 加減速角度 rad
    const float KEEP_OMEGA_ANGLE  = 56.0 * M_PI / 180.0; // 40.0 -> 0.6981317008; // 低速角度 rad

    return slalomBase(isTurnRight, endSpeed, timeout, 
            MAX_OMEGA, ACCEL_DECEL, 
            START_OFFSET_DISTANCE, STOP_OFFSET_DISTANCE,
            ACCEL_DECEL_ANGLE, KEEP_OMEGA_ANGLE);
}


int fastSlalom(const int isTurnRight, const float endSpeed, const float timeout){
    // 最短走行用のスラローム
    const float MAX_OMEGA= 15; // 最大角速度 rad/s
    const float ACCEL_DECEL = 200; // 50 加減速度 rad/s^2
    const float START_OFFSET_DISTANCE = 0.003; // オフセット直線走行距離 meter
    const float STOP_OFFSET_DISTANCE = 0.003; // オフセット直線走行距離 meter
    const float ACCEL_DECEL_ANGLE = 30.0 * M_PI / 180.0; // 25.0 ->  0.436332313; // 加減速角度 rad
    const float KEEP_OMEGA_ANGLE  = 30.0 * M_PI / 180.0; // 40.0 -> 0.6981317008; // 低速角度 rad

    return slalomBase(isTurnRight, endSpeed, timeout, 
            MAX_OMEGA, ACCEL_DECEL, 
            START_OFFSET_DISTANCE, STOP_OFFSET_DISTANCE,
            ACCEL_DECEL_ANGLE, KEEP_OMEGA_ANGLE);
}

int straightBack(const float timeout){
    // けつあてようの逆走行
    const float MIN_SPEED = 0.2; // m/s

    control_t control;
    control.maxSpeed = MIN_SPEED;
    control.maxOmega = 0;
    control.invertSpeed = 1;
    control.invertOmega = 0;
    control.accelSpeed = 0;
    control.accelOmega = 0;
    control.forceSpeedEnable = 0;
    control.forceOmegaEnable = 0; 
    control.enableWallControl = 0;

    // 移動距離を初期化
    // gObsMovingDistance = 0;

    // 時間計測開始
    clock_t startTime = clock();

    // 加速・定速
    control.accelSpeed = 1.0;
    // 強制的に速度を与える
    control.forceSpeedEnable = 1;
    control.forceSpeed = MIN_SPEED;
    while(1){
        // 制御器の更新
        updateController(&control);
        vTaskDelay(1 / portTICK_PERIOD_MS);

        // タイムアウトチェック
        if(timeout < (float)(clock() - startTime)/CLOCKS_PER_SEC){
            ESP_LOGE(TAG, "Timeout at accelereation");
            break;
        }
    }

    // 強制的に速度を0にする
    control.forceSpeedEnable = 1;
    control.forceSpeed = 0;
    // 差分を初期化する
    control.initializeSumOfError = 1;
    // 制御器の更新
    updateController(&control);
    vTaskDelay(1 / portTICK_PERIOD_MS);

    // 移動距離を初期化
    // 関数の終了時に初期化することで、
    // 関数外の処理中に進んだ距離を計測できる
    gObsMovingDistance = 0;

    return TRUE;
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


int searchStraight(const float distance, const float endSpeed){
    // 予めパラメータをセットした探索走行関数
    // コードを綺麗にするために作成した
    return straight(distance, endSpeed, pSEARCH_TIMEOUT, 
            pSEARCH_MAX_SPEED, pSEARCH_ACCEL);
}

int fastStraight(const float distance, const float endSpeed){
    // 予めパラメータをセットした最短走行関数
    // コードを綺麗にするために作成した
    return straight(distance, endSpeed, pFAST_TIMEOUT, 
            pFAST_MAX_SPEED, pFAST_ACCEL);
}

int ketsuate(const float endSpeed){
    // 予めパラメータをセットしたけつあて関数
    // コードを綺麗にするために作成した
    int result;

    result = straightBack(pKETSU_TIMEOUT);
    // 振動を防ぐためモータをOFF
    gMotorState = MOTOR_OFF;
    vTaskDelay(100 / portTICK_PERIOD_MS);
    // ジャイロのバイアスリセット
    gGyroBiasResetRequest = 1;
    while(gGyroBiasResetRequest){
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    gMotorState = MOTOR_ON;
    result = straight(pKETSU_DISTANCE, endSpeed, pSEARCH_TIMEOUT, 
            pSEARCH_MAX_SPEED, pSEARCH_ACCEL);
    return result;
}


