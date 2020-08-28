
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <fstream>
#include <iostream>
#include <string>

#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "icm20648.h"

static int led_pattern_ = 0;
static bool save_led_pattern_ = false;

static void TaskLED(void *arg){
    const gpio_num_t GPIO_LED_0 = GPIO_NUM_22;
    const gpio_num_t GPIO_LED_1 = GPIO_NUM_23;

    gpio_pad_select_gpio(GPIO_LED_0);  // パッドをGPIOとして扱う
    gpio_pad_select_gpio(GPIO_LED_1);  // パッドをGPIOとして扱う
    gpio_set_direction(GPIO_LED_0, GPIO_MODE_OUTPUT);  // GPIOを出力に設定
    gpio_set_direction(GPIO_LED_1, GPIO_MODE_OUTPUT);  // GPIOを出力に設定

    while(1){
        double delay_time_ms = 100 * (led_pattern_ + 1);

        gpio_set_level(GPIO_LED_0, 1);
        gpio_set_level(GPIO_LED_1, 1);
        vTaskDelay(delay_time_ms / portTICK_PERIOD_MS);

        gpio_set_level(GPIO_LED_0, 0);
        gpio_set_level(GPIO_LED_1, 0);
        vTaskDelay(delay_time_ms / portTICK_PERIOD_MS);
    }
}

static void TaskIMU(void *arg){
    const gpio_num_t GPIO_MOSI = GPIO_NUM_19;
    const gpio_num_t GPIO_MISO = GPIO_NUM_21;
    const gpio_num_t GPIO_SCLK = GPIO_NUM_18;
    const gpio_num_t GPIO_CS = GPIO_NUM_5;
    const unsigned int ACCEL_FSSEL = 1;  // 0:2g, 1:4g, 2:8g, 3:16g
    const unsigned int GYRO_FSSEL = 2;  // 0:250dps, 1:500dps, 2:1000dps, 3:2000dps

    // LED点灯用のパラメータ
    const int MAX_LED_PATTERN = 3;
    const int MIN_LED_PATTERN = 0;
    const double GYRO_THRESHOLD = 800;  // degrees per sec

    icm20648 imu_driver(GPIO_MOSI, GPIO_MISO, GPIO_SCLK, GPIO_CS,
            ACCEL_FSSEL, GYRO_FSSEL);

    while(1){
        double gyro_x = imu_driver.getGyroX();

        // ジャイロの値がしきい値を超えたら、LEDパターンを更新する
        if(std::abs(gyro_x) > GYRO_THRESHOLD){
            if(gyro_x > GYRO_THRESHOLD){
                led_pattern_++;
            }else{
                led_pattern_--;
            }

            // オーバーフロー、アンダーフローの処理
            if(led_pattern_ > MAX_LED_PATTERN){
                led_pattern_ = MIN_LED_PATTERN;
            }else if(led_pattern_ < MIN_LED_PATTERN){
                led_pattern_ = MAX_LED_PATTERN;
            }

            // LEDパターンを保存するためにフラグを立てる
            save_led_pattern_ = true;
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}


/* Inside .cpp file, app_main function must be declared with C linkage */
extern "C" void app_main(){
    const char *TAG = "logger_example";
    const char *PARTITION_LABEL = "my_storage";
    const std::string BASE_PATH = "/my_log";
    const std::string FILE_NAME = BASE_PATH + "/LED点灯パターン.txt";
    const esp_vfs_fat_mount_config_t MOUNT_CONFIG = {
            .format_if_mount_failed = true,
            .max_files = 4,
            .allocation_unit_size = CONFIG_WL_SECTOR_SIZE
    };

    wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

    // FATファイルシステムを初期化してバーチャルファイルシステムに登録する
    esp_err_t err = esp_vfs_fat_spiflash_mount(
            BASE_PATH.c_str(), PARTITION_LABEL, &MOUNT_CONFIG, &s_wl_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount FATFS (%s)", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "LED点灯パターンを読み出します");
    std::ifstream ifs(FILE_NAME, std::ios::in | std::ios::binary);
    if(ifs.fail()){
        ESP_LOGE(TAG, "Failed to open %s for reading.", FILE_NAME.c_str());
    }else{
        std::string line;
        getline(ifs, line);
        led_pattern_ = std::stoi(line);
        ESP_LOGI(TAG, "LED点灯パターン[%d]を読み出しました", led_pattern_);
    }
    ifs.close();

    // タスクの生成
    xTaskCreate(TaskLED, "TaskLED", 4096, NULL, 5, NULL);
    xTaskCreate(TaskIMU, "TaskIMU", 4096, NULL, 5, NULL);

    while(1){
        if(save_led_pattern_){
            ESP_LOGI(TAG, "LED点灯パターン[%d]を保存します", led_pattern_);
            std::ofstream ofs(FILE_NAME, std::ios::out | std::ios::binary);
            if(ofs.fail()){
                ESP_LOGE(TAG, "Failed to open %s for writing.", FILE_NAME.c_str());
            }else{
                ofs << led_pattern_ << std::endl;
                ofs.close();
                ESP_LOGI(TAG, "保存しました");
            }

            save_led_pattern_ = false;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }

    // FATファイルシステムのアンマウント
    esp_vfs_fat_spiflash_unmount(BASE_PATH.c_str(), s_wl_handle);
}

