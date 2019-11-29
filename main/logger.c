
#include <stdio.h>
#include <time.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#include "logger.h"
#include "parameters.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
static const char *TAG="Logger";

#define SPIFFS_OK 1
#define SPIFFS_NG 0

static const char *FILE_PATH = "/spiffs/log.csv";
static const char *WALL_FILE_PATH = "/spiffs/wall.csv";
static int spiffsInitialized = SPIFFS_NG;

static const int INIT_PERIOD_MSEC = 1000;
static const int INIT_TIMEOUT_MSEC = 1000;
static float DUMMY_DATA = 0.0;
static char *pLogName1, *pLogName2, *pLogName3;
static float *pLogData1, *pLogData2, *pLogData3;
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
        char *name1, float *data1,
        char *name2, float *data2,
        char *name3, float *data3){
    ESP_LOGI(TAG, "Initialize: Period %d msec, Timeout %d msec", period_msec, timeout_msec); 
    logPeriod_msec = period_msec;
    logTimeout_msec = timeout_msec;

    // TODO:ここを簡潔に書く
    if(data1){
        ESP_LOGI(TAG, "Initialize: Log data1 is %s, current value is %f", name1, *data1); 
        pLogName1 = name1;
        pLogData1 = data1;
    }else{
        ESP_LOGI(TAG, "Initialize: No log data1."); 
        pLogName1 = "DUMNY_DATA";
        pLogData1 = &DUMMY_DATA;
    }
    if(data2){
        ESP_LOGI(TAG, "Initialize: Log data2 is %s, current value is %f", name2, *data2); 
        pLogName2 = name2;
        pLogData2 = data2;
    }else{
        ESP_LOGI(TAG, "Initialize: No log data2."); 
        pLogName2 = "DUMNY_DATA";
        pLogData2 = &DUMMY_DATA;
    }
    if(data3){
        ESP_LOGI(TAG, "Initialize: Log data3 is %s, current value is %f", name3, *data3); 
        pLogName3 = name3;
        pLogData3 = data3;
    }else{
        ESP_LOGI(TAG, "Initialize: No log data3."); 
        pLogName3 = "DUMNY_DATA";
        pLogData3 = &DUMMY_DATA;
    }
    logIndex = 0;

    int logSize = timeout_msec / period_msec;
    if(logSize > LOG_INDEX_NUM){
        ESP_LOGW(TAG, "Log Size %d grater than LOG_INDEX_NUM: %d", logSize, LOG_INDEX_NUM);
    }else{
        ESP_LOGI(TAG, "Log Size %d smaller than LOG_INDEX_NUM: %d", logSize, LOG_INDEX_NUM);
    }
    logInitialized = TRUE;
}

void loggingStart(void){
    if(logInitialized){
        ESP_LOGI(TAG, "Start logging");
        logStarted = TRUE;
        logStartTime = (uint32_t) (clock() * 1000 / CLOCKS_PER_SEC);
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
    printf("Index,TimeElapsed,%s,%s,%s\n", pLogName1,pLogName2,pLogName3);
    for(int log_i=0; log_i<logIndex; log_i++){
        printf("%d,%d,%f,%f,%f\n",log_i, gLogTime[log_i], 
                gLogData[0][log_i],
                gLogData[1][log_i],
                gLogData[2][log_i]
                );
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

    fprintf(f, "Index,TimeElapsed,%s,%s,%s\n", pLogName1,pLogName2,pLogName3);
    for(int log_i=0; log_i<logIndex; log_i++){
        fprintf(f, "%d,%d,%f,%f,%f\n", log_i,gLogTime[log_i],
                gLogData[0][log_i],
                gLogData[1][log_i],
                gLogData[2][log_i]
                );
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

    char stringBuf[5][20];

    // １行目は文字列
    // Ref:https://qiita.com/oh-thevenin/items/6cbdd5081a4b31e41127
    fscanf(f, "%[^,],%[^,],%[^,],%[^,],%s", stringBuf[0], stringBuf[1], 
            stringBuf[2], stringBuf[3], stringBuf[4]);
    printf("%s,%s,%s,%s,%s\n",stringBuf[0], stringBuf[1], 
            stringBuf[2], stringBuf[3], stringBuf[4]);


    int ret;
    int index, timeElapsed;
    float data[LOG_DATA_NUM];
    while( (ret=fscanf(f, "%d,%d,%f,%f,%f", &index, &timeElapsed, 
                    &data[0], &data[1], &data[2])) != EOF){
        printf("%d,%d,%f,%f,%f\n",index,timeElapsed,
                data[0], data[1], data[2]);
    }
    fclose(f);

    ESP_LOGI(TAG, "File loaded");

}

char encodeWall(const t_wall wall){
    // 壁情報を1つのchar型変数にまとめる
    //
    // Data order:
    // north:east:south:west
    char data=0;
    data |= (wall.north && 0x03);
    data <<= 2;
    data |= (wall.east && 0x03);
    data <<= 2;
    data |= (wall.south && 0x03);
    data <<= 2;
    data |= (wall.west && 0x03);

    return data;
}

void decodeWall(char* data, t_wall* wall){
    wall->north = (*data) && 0x03;
    wall->east  = ((*data)>>2) && 0x03;
    wall->south = ((*data)>>4) && 0x03;
    wall->west  = ((*data)>>6) && 0x03;
}


void loggingSaveWall(const int mazesize_x, const int mazesize_y, t_wall wallMap[mazesize_x][mazesize_y]){
    // 壁情報をフラッシュメモリに保存する
    
    if(spiffsInitialized != SPIFFS_OK){
        return;
    }

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");

    FILE* f = fopen(WALL_FILE_PATH, "w");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        return;
    }
    for(int x_i=0; x_i<mazesize_x; x_i++){
        char text[128] = {};
        char data = encodeWall(wallMap[x_i][0]);
        strcat(text, &data);
        for(int y_i=1; y_i<mazesize_y; y_i++){
            data = encodeWall(wallMap[x_i][y_i]);
            strcat(text, ",");
            strcat(text, &data);
        }
        ESP_LOGE(TAG, "%s", text);
        fprintf(f, "%s\n",text);
    }

    fclose(f);

    ESP_LOGI(TAG, "File written");
}

void loggingLoadPrintWall(void){
    // フラッシュメモリの壁情報を読み取り、シリアルでプリントする

    if(spiffsInitialized != SPIFFS_OK){
        return;
    }

    // Use POSIX and C standard library functions to work with files.
    // First create a file.
    ESP_LOGI(TAG, "Opening file");

    FILE* f = fopen(WALL_FILE_PATH, "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for loading");
        return;
    }

    char buf[128] = {};

    int ret;
    int count=0;
    // 1行ずつ読み込む
    while( (ret=fscanf(f, "%s", buf)) != EOF){
        printf("%d:", count);

        t_wall wall;

        char* tp;
        tp = strtok(buf, ",");
        decodeWall(tp, &wall);
        printf("%d", wall.north);
        while((tp = strtok(NULL, ",")) ){
            decodeWall(tp, &wall);
            printf(" %d", wall.north);
        }
        printf("\n");
        count++;
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
            if(logIndex < LOG_INDEX_NUM){
                // 時間取得
                elapsed_time = (uint32_t) (clock() * 1000 / CLOCKS_PER_SEC) - logStartTime;
                ESP_LOGD(TAG, "Logging1: %d, %s, %f", elapsed_time, pLogName1, *pLogData1);
                ESP_LOGD(TAG, "Logging2: %d, %s, %f", elapsed_time, pLogName2, *pLogData2);
                ESP_LOGD(TAG, "Logging3: %d, %s, %f", elapsed_time, pLogName3, *pLogData3);
                gLogTime[logIndex] = elapsed_time;
                gLogData[0][logIndex] = *pLogData1;
                gLogData[1][logIndex] = *pLogData2;
                gLogData[2][logIndex] = *pLogData3;
                logIndex++;

                if(elapsed_time > logTimeout_msec){
                    ESP_LOGI(TAG, "Timeout.");
                    logPeriod_msec = INIT_PERIOD_MSEC;
                    logTimeout_msec = INIT_TIMEOUT_MSEC;
                    logStarted = FALSE;
                }
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
