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
#include "search.h"

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
    const float TIME_OUT = 4.0; // sec
    const float MAX_SPEED = 0.3; // m/s
    const float ACCEL = 1.0; // m/ss
    const float HALF_DISTANCE = 0.045;
    const float DISTANCE = 0.090;
    const float KETSU_DISTANCE = 0.003;
    const float KETSU_TIME_OUT = 0.5; // sec
    int result = TRUE;

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
    result = straight(HALF_DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);

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
            result = straight(HALF_DISTANCE, 0.0, TIME_OUT, MAX_SPEED, ACCEL);
            result = turn(M_PI_2, TIME_OUT);

            if(doHipAdjust){
                // けつあて
                result = straightBack(KETSU_TIME_OUT);
                // ジャイロのバイアスリセット
                gGyroBiasResetRequest = 1;
                while(gGyroBiasResetRequest){
                    vTaskDelay(1 / portTICK_PERIOD_MS);
                }
                result = straight(KETSU_DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
                doHipAdjust = 0;
            }
            result = straight(HALF_DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
        }else if(gObsIsWall[DIREC_FRONT] != 1){
            // 前に壁がなければ前に進む
            gIndicatorValue = 3;
            result = straight(DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);

        }else if(gObsIsWall[DIREC_RIGHT] != 1){
            // 右に壁がなければ右に進む
            if(gObsIsWall[DIREC_LEFT] == 1){
                // 左に壁があればけつあて
                doHipAdjust = 1;
            }
            gIndicatorValue = 1;
            result = straight(HALF_DISTANCE, 0.0, TIME_OUT, MAX_SPEED, ACCEL);
            result = turn(-M_PI_2, TIME_OUT);

            if(doHipAdjust){
                // けつあて
                result = straightBack(KETSU_TIME_OUT);
                // ジャイロのバイアスリセット
                gGyroBiasResetRequest = 1;
                while(gGyroBiasResetRequest){
                    vTaskDelay(1 / portTICK_PERIOD_MS);
                }
                result = straight(KETSU_DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
                doHipAdjust = 0;
            }
            result = straight(HALF_DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);

        }else{
            // それ以外の場合は後ろに進む
            gIndicatorValue = 6;
            result = straight(HALF_DISTANCE, 0.0, TIME_OUT, MAX_SPEED, ACCEL);
            result = turn(M_PI, TIME_OUT);
            // けつあて
            result = straightBack(KETSU_TIME_OUT);
            // ジャイロのバイアスリセット
            gGyroBiasResetRequest = 1;
            while(gGyroBiasResetRequest){
                vTaskDelay(1 / portTICK_PERIOD_MS);
            }
            result = straight(KETSU_DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);

            result = straight(HALF_DISTANCE, MAX_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
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
    static const int LOGGING_ENABLE = 0;

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
                "gObsMovingDistance", &gObsMovingDistance,
                "gTargetSpeed", &gTargetSpeed,
                "gObsSpeed", &gObsSpeed);
        while(loggingIsInitialized() == FALSE){
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
        loggingStart();
        while(loggingIsStarted() == FALSE){
            vTaskDelay(1 / portTICK_PERIOD_MS);
        }
    }
    // --------------------------------------

    const float DISTANCE = 0.090;
    const float HALF_DISTANCE = 0.045;
    const float END_SPEED = 0.3; // m/s
    const float TIME_OUT = 2.0; // sec
    const float MAX_SPEED = 0.3; // m/s
    const float ACCEL = 1.0; // m/ss

    gMotorState = MOTOR_ON;
    int result;

    // // 外周をグルって回るやつ
    // straight(HALF_DISTANCE, END_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
    // for(int i=0; i<4; i++){
    //     straight(DISTANCE, END_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
    //     straight(DISTANCE, END_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
    //     straight(HALF_DISTANCE, 0.0, TIME_OUT, MAX_SPEED, ACCEL);
    //     turn(-M_PI_2, TIME_OUT);
    //     straight(HALF_DISTANCE, END_SPEED, TIME_OUT, MAX_SPEED, ACCEL);
    // }

    turn(-M_PI_2, TIME_OUT);
    turn(-M_PI_2, TIME_OUT);
    turn(-M_PI_2, TIME_OUT);
    turn(-M_PI_2, TIME_OUT);

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

    ESP_LOGI(TAG, "Complete initialization.");
    while(1){
        if(gCurrentMode == MODE_SELECT){
            // モード選択
            int mode = gObsDial;
            // タッチセンサでモードを確定する
            if(gObsTouch[RIGHT] && gObsTouch[LEFT]){
                switch(mode){
                case MODE0_SEARCH:
                    ESP_LOGI(TAG, "SEARCH");
                    // searchLefthand();
                    searchAdachi(3,0);
                    // search_adachi(0,3);
                    break;
                case MODE1_FAST_RUN:
                    ESP_LOGI(TAG, "FAST_RUN");
                    break;
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

            vTaskDelay(500 / portTICK_PERIOD_MS);
            xTaskCreate(TaskMain, "TaskMain", 4096, NULL, 5, NULL);

            // ダイアルを初期化
            gObsDial = 0;
        }
    }


    ESP_LOGI(TAG, "Finish startup.");
}

