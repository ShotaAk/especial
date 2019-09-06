
#include <stdio.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "logger.h"
#include "variables.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
static const char *TAG="Logger";

static const int INIT_PERIOD_MSEC = 1000;
static const int INIT_TIMEOUT_MSEC = 1000;
static char *pLogName1;
static float *pLogData1;
static int logPeriod_msec;
static int logTimeout_msec;
static int logInitialized;
static int logStarted;
static int logIndex;
static int logStartTime;


void loggingInitialize(const int period_msec, const int timeout_msec,
        char *name1, float *data1){
    ESP_LOGI(TAG, "Initialize: Period %d msec, Timeout %d msec", period_msec, timeout_msec); 
    ESP_LOGI(TAG, "Initialize: Log data is %s, current value is %f", name1, *data1); 
    logPeriod_msec = period_msec;
    logTimeout_msec = timeout_msec;
    pLogName1 = name1;
    pLogData1 = data1;
    logIndex = 0;

    int logSize = timeout_msec / period_msec;
    if(logSize > LOG_NUM){
        ESP_LOGW(TAG, "Log Size %d grater than LOG_NUM: %d", logSize, LOG_NUM);
    }else{
        ESP_LOGI(TAG, "Log Size %d smaller than LOG_NUM: %d", logSize, LOG_NUM);
    }
    logInitialized = TRUE;
    logStartTime = (uint32_t) (clock() * 1000 / CLOCKS_PER_SEC);
}

void loggingStart(void){
    if(logInitialized){
        ESP_LOGI(TAG, "Start logging");
        logStarted = TRUE;
    }
}

void loggingStop(void){
    ESP_LOGI(TAG, "Stop logging");
    logStarted = FALSE;
}

void loggingReset(void){
    ESP_LOGI(TAG, "Reset logging");
    logStarted = FALSE;
    logInitialized = FALSE;
    logPeriod_msec = INIT_PERIOD_MSEC;
    logTimeout_msec = INIT_TIMEOUT_MSEC;
    logIndex = 0;
}

int loggingIsInitialized(void){
    return logInitialized;
}

int loggingIsStarted(void){
    return logStarted;
}

void loggingPrint(void){
    printf("Index,Timestamp,%s\n", pLogName1);
    for(int log_i=0; log_i<logIndex; log_i++){
        printf("%d,%d,%f\n",log_i, gLogTime[log_i], gLogData[log_i]);
    }
}

void loggingSave(void){

}

void TaskLogging(void *arg){
    logPeriod_msec = INIT_PERIOD_MSEC;
    logTimeout_msec = INIT_TIMEOUT_MSEC;
    logInitialized = FALSE;
    logStarted = FALSE;
    logIndex = 0;

    uint32_t elapsed_time;
    ESP_LOGI(TAG, "Complete initialization.");
    while(1){
        if(logStarted){
            if(logIndex < LOG_NUM){
                // 時間取得
                elapsed_time = (uint32_t) (clock() * 1000 / CLOCKS_PER_SEC) - logStartTime;
                ESP_LOGD(TAG, "Logging: %d, %s, %f", elapsed_time, pLogName1, *pLogData1);
                gLogTime[logIndex] = elapsed_time;
                gLogData[logIndex] = *pLogData1;
                logIndex++;
            }else{
                logStarted = FALSE;
            }

        }else{
            ESP_LOGD(TAG, "Stanby");
        }

        vTaskDelay(logPeriod_msec / portTICK_PERIOD_MS);
    }

}
