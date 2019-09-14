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

static void controlTest(void){

    // enum CONTROL_REQUEST requests[19] = {
    //     CONT_FORWARD,
    //     CONT_FORWARD,
    //     CONT_FORWARD,
    //     CONT_TURN_LEFT,
    //     CONT_TURN_LEFT,
    //     CONT_FORWARD,
    //     CONT_FORWARD,
    //     CONT_FORWARD,
    //     CONT_TURN_RIGHT,
    //     CONT_TURN_RIGHT,
    //     CONT_FORWARD,
    //     CONT_FORWARD,
    //     CONT_FORWARD,
    //     CONT_TURN_BACK,
    //     CONT_FORWARD,
    //     CONT_FORWARD,
    //     CONT_FORWARD,
    //     CONT_TURN_BACK,
    //     CONT_FINISH
    // };

    enum CONTROL_REQUEST requests[19] = {
        CONT_HALF_FORWARD,
        CONT_HALF_FORWARD,
        CONT_TURN_RIGHT,
        CONT_FORWARD,
        CONT_TURN_RIGHT,
        CONT_FORWARD,
        CONT_TURN_LEFT,
        CONT_FORWARD,
        CONT_TURN_BACK,
        CONT_FINISH
    };



    gControlRequest = CONT_NONE;
    int step=0;
    while(1){
        if(gControlRequest == CONT_NONE){
            printf("REQUEST!!!! %d\n",requests[step]);
            gControlRequest = requests[step];
            if(requests[step] == CONT_FINISH){
                printf("FINISH!!!! \n");
                break;
            }
            step++;
        }

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void requestAndWait(const enum CONTROL_REQUEST request){
    // controllerへリクエストを送り、動作完了を待つ

    gControlRequest = request;
    while(gControlRequest != CONT_NONE){
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
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

/*
static void TaskMain(void *arg){
    gIndicatorValue = 0x01;

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // gMotorState = MOTOR_ON;
    // gMotorDuty[RIGHT] = 20;
    // gMotorDuty[LEFT] = 20;

    //　起動時のデバッグ
    const float thresh = 1.0;
    if(gObjVoltages[OBJ_SENS_L] > thresh && gObjVoltages[OBJ_SENS_R] > thresh){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // ダブルチェック
        if(gObjVoltages[OBJ_SENS_L] > thresh && gObjVoltages[OBJ_SENS_R] > thresh){
            for(int step=0; step<5; step++){
                vTaskDelay(300 / portTICK_PERIOD_MS);
                gIndicatorValue = 3;
                vTaskDelay(300 / portTICK_PERIOD_MS);
                gIndicatorValue = 0;
            }
            printf("ENKAI!!!! \n");
            gControlRequest = CONT_ENKAI;
        }
    }

    float prevAngle = gWheelAngle[RIGHT];
    while(1){
        float diffAngle = gWheelAngle[RIGHT] - prevAngle;
        if(fabs(diffAngle) > 1.0){
            prevAngle = gWheelAngle[RIGHT];

            gIndicatorValue += 1;
            if(gIndicatorValue > 3){
                gIndicatorValue = 1;
            }
        }


        if(gIndicatorValue == 3){
            // モード選択
            if(gObjVoltages[OBJ_SENS_L] > thresh && gObjVoltages[OBJ_SENS_R] > thresh){
                vTaskDelay(1000 / portTICK_PERIOD_MS);

                // ダブルチェック
                if(gObjVoltages[OBJ_SENS_L] > thresh && gObjVoltages[OBJ_SENS_R] > thresh){
                    gIndicatorValue = 0;
                    vTaskDelay(300 / portTICK_PERIOD_MS);
                    gIndicatorValue = 3;
                    vTaskDelay(300 / portTICK_PERIOD_MS);
                    gIndicatorValue = 0;
                    vTaskDelay(300 / portTICK_PERIOD_MS);
                    gIndicatorValue = 3;
                    vTaskDelay(300 / portTICK_PERIOD_MS);
                    // goForward(10);
                    // turnBack(10);
                    // Maze();
                    // motorTest();
                    // controlTest();
                    searchLefthand();
                    // if(gControlRequest == CONT_FINISH || gControlRequest == CONT_NONE){
                    //     // gControlRequest = CONT_FORWARD;
                    //     gControlRequest = CONT_KETSUATE;
                    // }
                }
            }

        }
        // printf("ax, ay az: %f, %f, %f\n",gAccel[AXIS_X], gAccel[AXIS_Y], gAccel[AXIS_Z]);
        // printf("gx, gy gz: %f, %f, %f\n",gGyro[AXIS_X], gGyro[AXIS_Y], gGyro[AXIS_Z]);
        printf("L, FL, FR, R: %f, %f, %f, %f\n", 
                gObjVoltages[OBJ_SENS_L], 
                gObjVoltages[OBJ_SENS_FL],
                gObjVoltages[OBJ_SENS_FR],
                gObjVoltages[OBJ_SENS_R]);


        // printf("encoder L:R %f:%f\n",gWheelAngle[LEFT], gWheelAngle[RIGHT]);

        // printf("battery Voltage:%d\n", gBatteryVoltage);

        // printf("%f\n", gMeasuredAngle);

        // printf("Distance: %f m, Speed: %f m/s\n", gMovingDistance, gMeasuredSpeed);
        
        // printf("Duty: %f, %f\n",gMotorDuty[LEFT], gMotorDuty[RIGHT]);

        // motorTest();

        

        // printf("00_TaskMain\n");
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}
*/

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

    gIndicatorValue = 9;
    // ロガーの起動
    if(loggingIsInitialized() == FALSE){
        loggingInitialize(1, 3000,
                "gObsMovingDistance", &gObsMovingDistance,
                "gTargetSpeed", &gTargetSpeed,
                "gObsSpeed", &gObsSpeed

                // "gObsAngle", &gObsAngle,
                // "gTargetOmega", &gTargetOmega,
                // "gGyroZ", &gGyro[AXIS_Z]

                // "gObjVoltagesR", &gObjVoltages[OBJ_SENS_R],
                // "gObjVoltagesL", &gObjVoltages[OBJ_SENS_L]
                // "gObsIsWallR", &gObsIsWall[DIREC_RIGHT],
                // "gObsIsWallL", &gObsIsWall[DIREC_LEFT]
                );
    }
    // ロガーの初期化が終わるまで待機
    while(loggingIsInitialized() == FALSE){
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    gIndicatorValue = 0;

    // ジャイロのバイアスリセット
    gIndicatorValue = 6;
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    gGyroBiasResetRequest = 1;
    while(gGyroBiasResetRequest){
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // ロガーが起動するまで待機
    loggingStart();
    while(loggingIsStarted() == FALSE){
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
    gIndicatorValue = 0;

    int result = TRUE;

    // --------------------------------------------

    const float TIME_OUT = 2.0; // sec
    const float MAX_SPEED = 0.3; // m/s
    const float ACCEL = 1.0; // m/ss

    gMotorState = MOTOR_ON;

    result = straight(0.045, 0.3, TIME_OUT, MAX_SPEED, ACCEL);
    result = straight(0.045, 0.0, TIME_OUT, MAX_SPEED, ACCEL);
    // result = turn(-M_PI_2, TIME_OUT);

    // KETSUATE
    // result = straight(-0.020, 0.0, 0.5, MAX_SPEED, ACCEL);
    // result = straight(0.010, 0.3, TIME_OUT, MAX_SPEED, ACCEL);

    // result = straight(0.045, 0.3, TIME_OUT, MAX_SPEED, ACCEL);
    // result = straight(0.045, 0.0, TIME_OUT, MAX_SPEED, ACCEL);

    ESP_LOGI(TAG, "Result is %d",result);


    // --------------------------------------------
    // 制御終了状態
    gMotorDuty[RIGHT] = 0;
    gMotorDuty[LEFT] = 0;
    loggingStop();

    // 結果の表示
    if(result){
        gIndicatorValue = 3;
    }else{
        gIndicatorValue = 6;
    }
    vTaskDelay(2000 / portTICK_PERIOD_MS);
    gMotorState = MOTOR_OFF;

    // ログの保存
    gIndicatorValue = 9;
    loggingSave();
    loggingReset();
    gIndicatorValue = 0;

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
                    searchLefthand();
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

