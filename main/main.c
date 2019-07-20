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
#include "battery.h"
#include "encoder.h"
#include "indicator.h"
#include "motion.h"
#include "motor.h"
#include "wall_detector.h"
#include "controller.h"

static void motorTest(void){
    const int delayTime = 500; // ms

    printf("Motor Test Start\n");
    gMotorState = MOTOR_ON;
    // 正転
    for(int duty_i=0; duty_i<=10; duty_i++){
        float duty = 10.0 * duty_i;
        gMotorDuty[RIGHT] = duty;
        gMotorDuty[LEFT] = duty;

        printf("Duty L:R -> %f:%f\n",gMotorDuty[LEFT], gMotorDuty[RIGHT]);
        vTaskDelay(delayTime / portTICK_PERIOD_MS);
    }

    // 正転から逆転
    for(int duty_i=10; duty_i>=-10; duty_i--){
        float duty = 10.0 * duty_i;
        gMotorDuty[RIGHT] = duty;
        gMotorDuty[LEFT] = duty;

        printf("Duty L:R -> %f:%f\n",gMotorDuty[LEFT], gMotorDuty[RIGHT]);
        vTaskDelay(delayTime / portTICK_PERIOD_MS);
    }

    // 逆転から0
    for(int duty_i=-10; duty_i<=0; duty_i++){
        float duty = 10.0 * duty_i;
        gMotorDuty[RIGHT] = duty;
        gMotorDuty[LEFT] = duty;

        printf("Duty L:R -> %f:%f\n",gMotorDuty[LEFT], gMotorDuty[RIGHT]);
        vTaskDelay(delayTime / portTICK_PERIOD_MS);
    }

    gMotorState = MOTOR_OFF;
    printf("Motor Test Finish\n");
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
    printf("search left hand\n");

    // ジャイロのバイアスリセット
    gGyroBiasResetRequest = 1;
    while(gGyroBiasResetRequest){
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }

    // ここからモータに電源が入る
    gControlRequest = CONT_NONE; // 待機状態

    //半区画進む
    requestAndWait(CONT_HALF_FORWARD);

    int doHipAdjust = 0; // けつあて補正
    while(1){
        // 左手法なので、条件文の順番が重要である
        // 順番を変更してはいけない
        if(gIsWall[DIREC_LEFT] != 1){
            printf("TURN LEFT\n");
            // 左に壁がなければ左に進む
            if(gIsWall[DIREC_LEFT] == 1){
                // 右に壁があればけつあて
                doHipAdjust = 1;
            }
            requestAndWait(CONT_HALF_FORWARD);
            requestAndWait(CONT_TURN_LEFT);
            if(doHipAdjust){
                // けつあて
                requestAndWait(CONT_KETSUATE);
                doHipAdjust = 0;
            }
            requestAndWait(CONT_HALF_FORWARD);
        }else if(gIsWall[DIREC_FRONT] != 1){
            printf("FORWARD\n");
            // 前に壁がなければ前に進む
            requestAndWait(CONT_FORWARD);

        }else if(gIsWall[DIREC_RIGHT] != 1){
            printf("TURN RIGHT\n");
            // 右に壁がなければ右に進む
            if(gIsWall[DIREC_LEFT] == 1){
                // 左に壁があればけつあて
                doHipAdjust = 1;
            }
            requestAndWait(CONT_HALF_FORWARD);
            requestAndWait(CONT_TURN_RIGHT);
            if(doHipAdjust){
                // けつあて
                requestAndWait(CONT_KETSUATE);
                doHipAdjust = 0;
            }
            requestAndWait(CONT_HALF_FORWARD);

        }else{
            printf("TURN BACK\n");
            // それ以外の場合は後ろに進む
            requestAndWait(CONT_HALF_FORWARD);
            requestAndWait(CONT_TURN_BACK);
            // けつあて
            requestAndWait(CONT_KETSUATE);
            requestAndWait(CONT_HALF_FORWARD);

        }
    }

}

static void TaskMain(void *arg){
    gIndicatorValue = 0x01;

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // gMotorState = MOTOR_ON;
    // gMotorDuty[RIGHT] = 20;
    // gMotorDuty[LEFT] = 20;

    //　起動時のデバッグ
    const float thresh = 1.0;
    if(gWallVoltage[WALL_SENS_L] > thresh && gWallVoltage[WALL_SENS_R] > thresh){
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        // ダブルチェック
        if(gWallVoltage[WALL_SENS_L] > thresh && gWallVoltage[WALL_SENS_R] > thresh){
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
            if(gWallVoltage[WALL_SENS_L] > thresh && gWallVoltage[WALL_SENS_R] > thresh){
                vTaskDelay(1000 / portTICK_PERIOD_MS);

                // ダブルチェック
                if(gWallVoltage[WALL_SENS_L] > thresh && gWallVoltage[WALL_SENS_R] > thresh){
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
        // printf("L, FL, FR, R: %f, %f, %f, %f\n", 
        //         gWallVoltage[WALL_SENS_L], 
        //         gWallVoltage[WALL_SENS_FL],
        //         gWallVoltage[WALL_SENS_FR],
        //         gWallVoltage[WALL_SENS_R]);

        // printf("isWall:F,B,L,R : %d, %d, %d, %d\n", 
        //         gIsWall[DIREC_FRONT], 
        //         gIsWall[DIREC_BACK],
        //         gIsWall[DIREC_LEFT],
        //         gIsWall[DIREC_RIGHT]);

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

void app_main()
{
    xTaskCreate(TaskCheckBatteryVoltage, "TaskCheckBatteryVoltage", 4096, NULL, 5, NULL);
    xTaskCreate(TaskReadEncoders, "TaskReadEncoders", 4096, NULL, 5, NULL);
    xTaskCreate(TaskIndicator, "TaskIndicator", 4096, NULL, 4, NULL);
    xTaskCreate(TaskReadMotion, "TaskReadMotion", 4096, NULL, 5, NULL);
    xTaskCreate(TaskMotorDrive, "TaskMotorDrive", 4096, NULL, 5, NULL);
    xTaskCreate(TaskDetectWall, "TaskDetectWall", 4096, NULL, 5, NULL);
    xTaskCreate(TaskControlMotion, "TaskControlMotion", 4096, NULL, 5, NULL);
    xTaskCreate(TaskMain, "TaskMain", 4096, NULL, 6, NULL);
}

