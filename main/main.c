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


static void TaskMain(void *arg){
    gIndicatorValue = 0x01;

    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // gMotorState = MOTOR_ON;
    // gMotorDuty[RIGHT] = 10;
    // gMotorDuty[LEFT] = 10;

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
            const float thresh = 1.0;
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
                    if(gControlRequest == CONT_NONE){
                        printf("REQUEST!!!! \n");
                        gControlRequest = CONT_FORWARD;
                    }
                }
            }

        }
        // printf("ax, ay az: %f, %f, %f\n",gAccel[AXIS_X], gAccel[AXIS_Y], gAccel[AXIS_Z]);
        printf("gx, gy gz: %f, %f, %f\n",gGyro[AXIS_X], gGyro[AXIS_Y], gGyro[AXIS_Z]);
        // printf("L, FL, FR, R: %f, %f, %f, %f\n", 
        //         gWallVoltage[WALL_SENS_L], 
        //         gWallVoltage[WALL_SENS_FL],
        //         gWallVoltage[WALL_SENS_FR],
        //         gWallVoltage[WALL_SENS_R]);

        // printf("encoder L:R %f:%f\n",gWheelAngle[LEFT], gWheelAngle[RIGHT]);

        // printf("battery Voltage:%d\n", gBatteryVoltage);

        // printf("%f\n", gMovingDistance);

        // printf("Distance: %f m, Speed: %f m/s\n", gMovingDistance, gMeasuredSpeed);
        
        // printf("Duty: %f, %f\n",gMotorDuty[LEFT], gMotorDuty[RIGHT]);

        // motorTest();

        

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    xTaskCreate(TaskCheckBatteryVoltage, "TaskCheckBatteryVoltage", 4096, NULL, 5, NULL);
    xTaskCreate(TaskReadEncoders, "TaskReadEncoders", 4096, NULL, 6, NULL);
    xTaskCreate(TaskIndicator, "TaskIndicator", 4096, NULL, 5, NULL);
    xTaskCreate(TaskReadMotion, "TaskReadMotion", 4096, NULL, 5, NULL);
    xTaskCreate(TaskMotorDrive, "TaskMotorDrive", 4096, NULL, 5, NULL);
    xTaskCreate(TaskDetectWall, "TaskDetectWall", 4096, NULL, 5, NULL);
    xTaskCreate(TaskControlMotion, "TaskControlMotion", 4096, NULL, 5, NULL);
    xTaskCreate(TaskMain, "TaskMain", 4096, NULL, 2, NULL);
}

