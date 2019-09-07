
#include <stdio.h>
#include <time.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#include "logger.h"
#include "variables.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
static const char *TAG="Logger";

#define SPIFFS_OK 1
#define SPIFFS_NG 0

static const char *FILE_PATH = "/spiffs/log.csv";
static int spiffsInitialized = SPIFFS_NG;

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


void SPIFFSinit(void){

    ESP_LOGI(TAG, "Initializing SPIFFS");
    
    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 5,
      .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }
    
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
        return;
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
        spiffsInitialized = SPIFFS_OK;
    }
}


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
    printf("Index,TimeElapsed,%s\n", pLogName1);
    for(int log_i=0; log_i<logIndex; log_i++){
        printf("%d,%d,%f\n",log_i, gLogTime[log_i], gLogData[log_i]);
    }
}

void loggingSave(void){
    if(spiffsInitialized != SPIFFS_OK){
        return;
    }

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");

    FILE* f = fopen(FILE_PATH, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }

    fprintf(f, "Index,TimeElapsed,%s\n", pLogName1);
    for(int log_i=0; log_i<logIndex; log_i++){
        fprintf(f, "%d,%d,%f\n", log_i,gLogTime[log_i],gLogData[log_i]);
    }
    fclose(f);

    ESP_LOGI(TAG, "File written");

}

void loggingLoadPrint(void){
    if(spiffsInitialized != SPIFFS_OK){
        return;
    }

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");

    FILE* f = fopen(FILE_PATH, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for loading");
        return;
    }

    char stringBuf[3][20];

    // １行目は文字列
    fscanf(f, "%[^,],%[^,],%s", stringBuf[0], stringBuf[1], stringBuf[2]);
    printf("%s,%s,%s\n",stringBuf[0], stringBuf[1], stringBuf[2]);


    int ret;
    int index, timeElapsed;
    float data;
    while( (ret=fscanf(f, "%d,%d,%f", &index, &timeElapsed, &data)) != EOF){
        printf("%d,%d,%f\n",index,timeElapsed,data);
    }
    fclose(f);

    ESP_LOGI(TAG, "File loaded");

}

void TaskLogging(void *arg){

    // Initialization for SPIFFS
    SPIFFSinit();
    vTaskDelay(2000 / portTICK_PERIOD_MS);

    // Initialization for Logger
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
                // データ容量が満杯になったので終了
                ESP_LOGI(TAG, "Finished.");
                logPeriod_msec = INIT_PERIOD_MSEC;
                logTimeout_msec = INIT_TIMEOUT_MSEC;
                logStarted = FALSE;
            }

        }else{
            ESP_LOGD(TAG, "Stanby");
        }

        vTaskDelay(logPeriod_msec / portTICK_PERIOD_MS);
    }

}
