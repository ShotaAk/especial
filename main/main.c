/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "variables.h"
#include "parameters.h"
#include "battery.h"
#include "encoder.h"
#include "indicator.h"
#include "motion.h"
#include "motor.h"
#include "object_sensor.h"
#include "controller.h"
#include "observer.h"
#include "logger.h"
#include "maze.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"

static void motorTest(void){
    static const char *TAG="motorTest";
    const int delayTime = 500; // ms

    ESP_LOGI(TAG, "Motor Test Start");
    gMotorState = MOTOR_ON;
    // 正転
    for(int duty_i=0; duty_i<=10; duty_i++){
        float duty = 10.0 * duty_i;
        gMotorDuty[RIGHT] = duty;
        gMotorDuty[LEFT] = duty;

        ESP_LOGI(TAG, "Duty L:R -> %f:%f",gMotorDuty[LEFT], gMotorDuty[RIGHT]);
        vTaskDelay(delayTime / portTICK_PERIOD_MS);
    }

    // 正転から逆転
    for(int duty_i=10; duty_i>=-10; duty_i--){
        float duty = 10.0 * duty_i;
        gMotorDuty[RIGHT] = duty;
        gMotorDuty[LEFT] = duty;

        ESP_LOGI(TAG, "Duty L:R -> %f:%f",gMotorDuty[LEFT], gMotorDuty[RIGHT]);
        vTaskDelay(delayTime / portTICK_PERIOD_MS);
    }

    // 逆転から0
    for(int duty_i=-10; duty_i<=0; duty_i++){
        float duty = 10.0 * duty_i;
        gMotorDuty[RIGHT] = duty;
        gMotorDuty[LEFT] = duty;

        ESP_LOGI(TAG, "Duty L:R -> %f:%f",gMotorDuty[LEFT], gMotorDuty[RIGHT]);
        vTaskDelay(delayTime / portTICK_PERIOD_MS);
    }

    gMotorState = MOTOR_OFF;
    ESP_LOGI(TAG, "Motor Test Finish");
}

void searchLefthand(void){
    float endSpeed = pSEARCH_MAX_SPEED;

    gIndicatorValue = 6;
    vTaskDelay(3000 / portTICK_PERIOD_MS);
    gIndicatorValue = 9;

    // ジャイロのバイアスリセット
    gGyroBiasResetRequest = 1;
    while(gGyroBiasResetRequest){
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    gIndicatorValue = 0;

    gMotorState = MOTOR_ON;
    //半区画進む
    straight(pHALF_CELL_DISTANCE, endSpeed, pSEARCH_TIMEOUT, 
            pSEARCH_MAX_SPEED, pSEARCH_ACCEL);

    int doHipAdjust = 0; // けつあて補正
    while(1){
        // 左手法なので、条件文の順番が重要である
        // 順番を変更してはいけない
        if(gObsIsWall[DIREC_LEFT] != 1){
            // 左に壁がなければ左に進む
            if(gObsIsWall[DIREC_RIGHT] == 1){
                // 右に壁があればけつあて
                doHipAdjust = 1;
            }

            gIndicatorValue = 2;
            straight(pHALF_CELL_DISTANCE, 0.0, pSEARCH_TIMEOUT, 
                    pSEARCH_MAX_SPEED, pSEARCH_ACCEL);
            turn(M_PI_2, pSEARCH_TIMEOUT);

            if(doHipAdjust){
                // けつあて
                straightBack(pKETSU_TIMEOUT);
                // ジャイロのバイアスリセット
                gGyroBiasResetRequest = 1;
                while(gGyroBiasResetRequest){
                    vTaskDelay(1 / portTICK_PERIOD_MS);
                }
                straight(pKETSU_DISTANCE, endSpeed, pSEARCH_TIMEOUT, 
                        pSEARCH_MAX_SPEED, pSEARCH_ACCEL);
                doHipAdjust = 0;
            }
            straight(pHALF_CELL_DISTANCE, endSpeed, pSEARCH_TIMEOUT, 
                    pSEARCH_MAX_SPEED, pSEARCH_ACCEL);

        }else if(gObsIsWall[DIREC_FRONT] != 1){
            // 前に壁がなければ前に進む
            gIndicatorValue = 3;
            straight(pCELL_DISTANCE, endSpeed, pSEARCH_TIMEOUT, 
                    pSEARCH_MAX_SPEED, pSEARCH_ACCEL);

        }else if(gObsIsWall[DIREC_RIGHT] != 1){
            // 右に壁がなければ右に進む
            if(gObsIsWall[DIREC_LEFT] == 1){
                // 左に壁があればけつあて
                doHipAdjust = 1;
            }
            gIndicatorValue = 1;
            straight(pHALF_CELL_DISTANCE, 0.0, pSEARCH_TIMEOUT, 
                    pSEARCH_MAX_SPEED, pSEARCH_ACCEL);
            turn(-M_PI_2, pSEARCH_TIMEOUT);

            if(doHipAdjust){
                // けつあて
                straightBack(pKETSU_TIMEOUT);
                // ジャイロのバイアスリセット
                gGyroBiasResetRequest = 1;
                while(gGyroBiasResetRequest){
                    vTaskDelay(1 / portTICK_PERIOD_MS);
                }
                straight(pKETSU_DISTANCE, endSpeed, pSEARCH_TIMEOUT, 
                        pSEARCH_MAX_SPEED, pSEARCH_ACCEL);
                doHipAdjust = 0;
            }
            straight(pHALF_CELL_DISTANCE, endSpeed, pSEARCH_TIMEOUT, 
                    pSEARCH_MAX_SPEED, pSEARCH_ACCEL);

        }else{
            // それ以外の場合は後ろに進む
            gIndicatorValue = 6;
            straight(pHALF_CELL_DISTANCE, 0.0, pSEARCH_TIMEOUT, 
                    pSEARCH_MAX_SPEED, pSEARCH_ACCEL);
            turn(M_PI, pSEARCH_TIMEOUT);
            // けつあて
            straightBack(pKETSU_TIMEOUT);
            // ジャイロのバイアスリセット
            gGyroBiasResetRequest = 1;
            while(gGyroBiasResetRequest){
                vTaskDelay(1 / portTICK_PERIOD_MS);
            }
            straight(pKETSU_DISTANCE, endSpeed, pSEARCH_TIMEOUT, 
                    pSEARCH_MAX_SPEED, pSEARCH_ACCEL);

            straight(pHALF_CELL_DISTANCE, endSpeed, pSEARCH_TIMEOUT, 
                    pSEARCH_MAX_SPEED, pSEARCH_ACCEL);
        }
    }

}


void indicateWall(void){
    // 壁センサの反応に合わせてLEDを点灯させる

    while(1){
        gIndicatorValue = 0;
        if(gObsIsWall[DIREC_RIGHT]){
            gIndicatorValue = 1;
        }
        if(gObsIsWall[DIREC_LEFT]){
            gIndicatorValue = 2;
        }
        if(gObsIsWall[DIREC_FRONT]){
            gIndicatorValue = 3;
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

}

void loggingTest(void){
    static const char *TAG="LoggingTest";

    vTaskDelay(2000 / portTICK_PERIOD_MS);
    if(gObsTouch[LEFT] && gObsTouch[RIGHT]){
        loggingLoadPrint();
        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }

    ESP_LOGI(TAG, "Complete initialization.");
    while(1){

        if(gObsTouch[LEFT] && gObsTouch[RIGHT]){
            // loggingSave(); // SPIFFSに保存
            loggingPrint();
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            loggingReset();
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }else if(gObsTouch[LEFT]){
            loggingStop();
            vTaskDelay(2000 / portTICK_PERIOD_MS);
        }else if(gObsTouch[RIGHT]){
            if(loggingIsInitialized() == FALSE){
                loggingInitialize(1, 3000,
                        "gObsAngle", &gObsAngle,
                        "", NULL,
                        "", NULL);

            }else{
                loggingStart();
                vTaskDelay(2000 / portTICK_PERIOD_MS);
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void Debug(void){
    static const char *TAG="Debug";
    static const int LOGGING_ENABLE = 0; // ロギングしたいときはここを１にする

    gIndicatorValue = 9;
    vTaskDelay(3000 / portTICK_PERIOD_MS);

    // ジャイロのバイアスリセット
    gGyroBiasResetRequest = 1;
    while(gGyroBiasResetRequest){
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
    gIndicatorValue = 0;

    // --------------ロガーの設定-------------
    if(LOGGING_ENABLE){
        loggingInitialize(1, 3000,
                "gObjVoltagesFR", &gObjVoltages[OBJ_SENS_FR],
                "gObjVoltagesFL", &gObjVoltages[OBJ_SENS_FL],
                // "gObsMovingDistance", &gObsMovingDistance,
                // "gTargetSpeed", &gTargetSpeed,
                // "gObsSpeed", &gObsSpeed);
                // "gGyroZ", &gGyro[AXIS_Z],
                // "gObsAngle", &gObsAngle,
                "gTargetOmega", &gTargetOmega);
        while(loggingIsInitialized() == FALSE){
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        loggingStart();
        while(loggingIsStarted() == FALSE){
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }
    // --------------------------------------

    gMotorState = MOTOR_ON;
    int result;

    while(1){
        ESP_LOGI(TAG, "volt L,R,FL,FR:\t%f,\t%f,\t%f\t%f",
                gObjVoltages[OBJ_SENS_L],
                gObjVoltages[OBJ_SENS_R],
                gObjVoltages[OBJ_SENS_FL],
                gObjVoltages[OBJ_SENS_FR]);
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // -----------時間経過を待つ------
    // vTaskDelay(3000 / portTICK_PERIOD_MS);

    // ------------16区画直進---------
    {
        // float endSpeed = pSEARCH_MAX_SPEED;
        // // 移動距離を初期化
        // gObsMovingDistance = 0;
        // searchStraight(pHALF_CELL_DISTANCE, endSpeed);
        // for(int i=0; i<4; i++){
        //     searchStraight(pCELL_DISTANCE, endSpeed);
        // }
        // searchStraight(pHALF_CELL_DISTANCE, 0.0);
    }

    // -------16区画直進を１つの関数で--------
    {
        // float endSpeed = 0.0;
        // float timeout = 10.0;
        // straight(pCELL_DISTANCE*3.0, endSpeed, timeout, 
        //         pSEARCH_MAX_SPEED, pSEARCH_ACCEL);
    }

    // 外周をグルって回るやつ
    {
        // float endSpeed = pSEARCH_MAX_SPEED;
        // // 移動距離を初期化
        // gObsMovingDistance = 0;
        // ketsuate(endSpeed);
        // searchStraight(pHALF_CELL_DISTANCE, endSpeed);
        // for(int i=0; i<8; i++){
        //     // スラローム
        //     searchStraight(pCELL_DISTANCE, endSpeed);
        //     slalom(FALSE, endSpeed, pSEARCH_TIMEOUT);
        //     // 超信地旋回
        //     // searchStraight(pHALF_CELL_DISTANCE, 0.0);
        //     // turn(-M_PI_2, pSEARCH_TIMEOUT);
        //     // searchStraight(pHALF_CELL_DISTANCE, endSpeed);
        // }
        // searchStraight(pHALF_CELL_DISTANCE, 0.0);
    }


    // ---------------けつあてとスラロームで３区画すすむ---------
    {
        // float endSpeed = pSEARCH_MAX_SPEED;
        // // けつあて
        // result = straightBack(pKETSU_TIMEOUT);
        // gMotorState = MOTOR_OFF;
        // vTaskDelay(500 / portTICK_PERIOD_MS);
        // // ジャイロのバイアスリセット
        // gGyroBiasResetRequest = 1;
        // while(gGyroBiasResetRequest){
        //     vTaskDelay(1 / portTICK_PERIOD_MS);
        // }
        // gMotorState = MOTOR_ON;
        //
        // straight(pKETSU_DISTANCE, endSpeed, pSEARCH_TIMEOUT, 
        //         pSEARCH_MAX_SPEED, pSEARCH_ACCEL);
        // straight(pHALF_CELL_DISTANCE, endSpeed, pSEARCH_TIMEOUT, 
        //         pSEARCH_MAX_SPEED, pSEARCH_ACCEL);
        //
        // // スラローム
        // gIndicatorValue = 3;
        // slalom(0, endSpeed, pSEARCH_TIMEOUT);
        // gIndicatorValue = 0;
        // straight(pHALF_CELL_DISTANCE, 0.0, pSEARCH_TIMEOUT, 
        //         pSEARCH_MAX_SPEED, pSEARCH_ACCEL);
    }
    

    gMotorState = MOTOR_OFF;
    // --------------ロガーの設定-------------
    if(LOGGING_ENABLE){
        gIndicatorValue = 9;
        loggingStop();
        while(loggingIsStarted() == TRUE){
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        loggingSave();
        gIndicatorValue = 0;
    }
    // --------------------------------------


    // ダイアルを初期化
    gObsDial = 0;
}

static void TaskMain(void *arg){
    static const char *TAG="Main";

    // 迷路初期化
    initMaze();

    ESP_LOGI(TAG, "Complete initialization.");
    while(1){
        if(gCurrentMode == MODE_SELECT){
            // モード選択
            int mode = gObsDial;
            int goalX = 1;
            int goalY = 1;
            // タッチセンサでモードを確定する
            if(gObsTouch[RIGHT] && gObsTouch[LEFT]){
                switch(mode){
                case MODE0_SEARCH:
                {
                    int slalomEnable = TRUE;
                    int goHomeEanble = TRUE;
                    ESP_LOGI(TAG, "SEARCH_SLALOM");
                    search(goalX, goalY, slalomEnable, goHomeEanble);
                    break;
                }
                case MODE1_FAST_RUN:
                {
                    int slalomEnable = FALSE;
                    int goHomeEanble = FALSE;
                    ESP_LOGI(TAG, "FAST RUN");
                    run(goalX, goalY, slalomEnable, goHomeEanble);
                    break;
                }
                case MODE2_CONFIG:
                    ESP_LOGI(TAG, "CONFIG");
                    updateWallThresholds();
                    break;
                case MODE3_DEBUG:
                    ESP_LOGI(TAG, "DEBUG");
                    Debug();
                    break;
                case MODE4_DUMMY:
                    ESP_LOGI(TAG, "LOG PRINT");
                    loggingLoadPrint();
                    vTaskDelay(2000 / portTICK_PERIOD_MS);
                    break;
                case MODE5_DUMMY:
                    ESP_LOGI(TAG, "CONFIG_PRINT");
                    indicateWall();
                    break;
                default:
                    ESP_LOGI(TAG, "ELSE");
                    break;
                }
            }
            gIndicatorValue = mode;
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    static const char *TAG="Startup";
    ESP_LOGI(TAG, "Especial Power On.");
    xTaskCreate(TaskCheckBatteryVoltage, "TaskCheckBatteryVoltage", 4096, NULL, 5, NULL);
    xTaskCreate(TaskIndicator, "TaskIndicator", 4096, NULL, 5, NULL);
    xTaskCreate(TaskObservation, "TaskObservation", 4096, NULL, 5, NULL);
    
    // バッテリ電圧が安定するまでのwait
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    if(gObsBatteryIsLow){
        ESP_LOGE(TAG, "Low battery voltage at startup. %f volts", gBatteryVoltage);
    }else{
        ESP_LOGI(TAG, "Battery voltage is %f volts", gBatteryVoltage);
        ESP_LOGI(TAG, "Start Especial Main Program");
        xTaskCreate(TaskObjectSensing, "TaskObjectSensing", 4096, NULL, 5, NULL);

        // センサーをタッチするまでwait
        vTaskDelay(1000 / portTICK_PERIOD_MS);

        if(gObsTouch[LEFT] && gObsTouch[RIGHT]){
            ESP_LOGI(TAG, "Start HTTP Server");
            gIndicatorValue = 1; // LED点灯
        }else{
            ESP_LOGI(TAG, "Start Especial Mouse");
            xTaskCreate(TaskLogging, "TaskLogging", 4096, NULL, 5, NULL);
            xTaskCreate(TaskReadEncoders, "TaskReadEncoders", 4096, NULL, 5, NULL);
            xTaskCreate(TaskReadMotion, "TaskReadMotion", 4096, NULL, 5, NULL);
            xTaskCreate(TaskMotorDrive, "TaskMotorDrive", 4096, NULL, 5, NULL);
            // xTaskCreate(TaskControlMotion, "TaskControlMotion", 4096, NULL, 5, NULL);
            gIndicatorValue = 9; // LED点灯

            vTaskDelay(3000 / portTICK_PERIOD_MS);
            xTaskCreate(TaskMain, "TaskMain", 4096, NULL, 5, NULL);

            // ダイアルを初期化
            gObsDial = 0;
        }
    }


    ESP_LOGI(TAG, "Finish startup.");
}

