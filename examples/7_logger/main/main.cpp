#include <iostream>
#include <iomanip>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <chrono>
#include <thread>

#include "driver/gpio.h"
#include "icm20648.h"

const static gpio_num_t GPIO_MOSI = GPIO_NUM_19;
const static gpio_num_t GPIO_MISO = GPIO_NUM_21;
const static gpio_num_t GPIO_SCLK = GPIO_NUM_18;
const static gpio_num_t GPIO_CS = GPIO_NUM_5;

const static gpio_num_t GPIO_LED_0 = GPIO_NUM_22;
const static gpio_num_t GPIO_LED_1 = GPIO_NUM_23;

const static int MAX_LED_PATTERN = 3;
const static int MIN_LED_PATTERN = 0;

static int led_pattern_ = 0;
static bool save_led_pattern_ = false;


static void TaskLED(void *arg){
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
    unsigned int accel_fssel = 1;  // 0:2g, 1:4g, 2:8g, 3:16g
    unsigned int gyro_fssel = 2;  // 0:250dps, 1:500dps, 2:1000dps, 3:2000dps
    icm20648 imu_driver(GPIO_MOSI, GPIO_MISO, GPIO_SCLK, GPIO_CS,
            accel_fssel, gyro_fssel);

    const double GYRO_THRESHOLD = 800;  // degrees per sec
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
    std::cout << "LEDパターンを読み出します" << std::endl;
    std::cout << "LEDパターン:" << led_pattern_ << "を読み出しました" << std::endl;
    // タスクの生成
    xTaskCreate(TaskLED, "TaskLED", 4096, NULL, 5, NULL);
    xTaskCreate(TaskIMU, "TaskIMU", 4096, NULL, 5, NULL);
    while(1){
        if(save_led_pattern_){
            std::cout << "LEDパターン:" << led_pattern_ << "を保存します" << std::endl;

            std::cout << "保存しました" << std::endl;
            save_led_pattern_ = false;
        }

        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

