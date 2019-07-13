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

static void motorTest(float duty){
    gMotorDriveMode = MOTOR_SLEEP;
    gMotorDuty[RIGHT] = 0;
    gMotorDuty[LEFT] = 0;
    printf("MOTOR SLEEP\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    gMotorDriveMode = MOTOR_STOP;
    gMotorDuty[RIGHT] = 0;
    gMotorDuty[LEFT] = 0;
    printf("MOTOR STOP\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    gMotorDriveMode = MOTOR_FORWARD;
    gMotorDuty[RIGHT] = duty;
    gMotorDuty[LEFT] = duty;
    printf("MOTOR FORWARD\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    gMotorDriveMode = MOTOR_BACKWARD;
    gMotorDuty[RIGHT] = duty;
    gMotorDuty[LEFT] = duty;
    printf("MOTOR BACKWARD\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    gMotorDriveMode = MOTOR_ROTATE_CW;
    gMotorDuty[RIGHT] = duty;
    gMotorDuty[LEFT] = duty;
    printf("MOTOR ROTATE CW\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    gMotorDriveMode = MOTOR_ROTATE_CCW;
    gMotorDuty[RIGHT] = duty;
    gMotorDuty[LEFT] = duty;
    printf("MOTOR ROTATE CCW\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    gMotorDriveMode = MOTOR_SLEEP;
    gMotorDuty[RIGHT] = 0;
    gMotorDuty[LEFT] = 0;
    printf("MOTOR SLEEP\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);
}

void turnBack(float duty){
    gMotorDriveMode = MOTOR_ROTATE_CW;
    gMotorDuty[RIGHT] = duty;
    gMotorDuty[LEFT] = duty;
    vTaskDelay(350 / portTICK_PERIOD_MS);
    gMotorDriveMode = MOTOR_STOP;
}

void goForward(float duty){
    gMotorDriveMode = MOTOR_FORWARD;
    gMotorDuty[RIGHT] = duty;
    gMotorDuty[LEFT] = duty;
}

static void Maze(void){
    const int thresh = 1000;

    while(1){
        // とりあえず前進
        goForward(10);
        if(gWallVoltage[WALL_SENS_L] > thresh && gWallVoltage[WALL_SENS_R] > thresh){
            // 壁があったら停止
            gMotorDriveMode = MOTOR_STOP;
            vTaskDelay(500 / portTICK_PERIOD_MS);
            turnBack(10);
        }
    }
}

static void TaskMain(void *arg){
    gIndicatorValue = 0x01;

    vTaskDelay(2000 / portTICK_PERIOD_MS);
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
            const int thresh = 1000;
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
                    motorTest(30);
                    // goForward(10);
                    // turnBack(10);
                    // Maze();
                }
            }

        }
        // printf("ax, ay az: %f, %f, %f\n",gAccel[AXIS_X], gAccel[AXIS_Y], gAccel[AXIS_Z]);
        // printf("gx, gy gz: %f, %f, %f\n",gGyro[AXIS_X], gGyro[AXIS_Y], gGyro[AXIS_Z]);
        printf("L, FL, FR, R: %d, %d, %d, %d\n", 
                gWallVoltage[WALL_SENS_L], 
                gWallVoltage[WALL_SENS_FL],
                gWallVoltage[WALL_SENS_FR],
                gWallVoltage[WALL_SENS_R]);

        // printf("encoder L:R %f:%f\n",gWheelAngle[LEFT], gWheelAngle[RIGHT]);

        // printf("battery Voltage:%d\n", gBatteryVoltage);
        
        // motorTest(10);

        

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

void app_main()
{
    xTaskCreate(TaskCheckBatteryVoltage, "TaskCheckBatteryVoltage", 4096, NULL, 5, NULL);
    xTaskCreate(TaskReadEncoders, "TaskReadEncoders", 4096, NULL, 5, NULL);
    xTaskCreate(TaskIndicator, "TaskIndicator", 4096, NULL, 5, NULL);
    xTaskCreate(TaskReadMotion, "TaskReadMotion", 4096, NULL, 5, NULL);
    xTaskCreate(TaskMotorDrive, "TaskMotorDrive", 4096, NULL, 6, NULL);
    xTaskCreate(TaskDetectWall, "TaskDetectWall", 4096, NULL, 5, NULL);
    xTaskCreate(TaskMain, "TaskMain", 4096, NULL, 5, NULL);
}

