#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
 
#define GPIO_OUTPUT_IO_0    22
#define GPIO_OUTPUT_IO_1    23
 
void app_main(void)
{
    gpio_config_t io_conf;
    // 割り込みをしない
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    // 出力モード
    io_conf.mode = GPIO_MODE_OUTPUT;
    // 設定したいピンのビットマスク
    // 1ULLはunsigned long long(64bit)で1を表現する
    // 22ピンと23ピンを設定するので、ビットマスクは下記のようになる
    // 上位32ビット：MSB|0000 0000|0000 0000|0000 0000|0000 0000|
    // 下位32ビット：    0000 0000|1100 0000|0000 0000|0000 0000|LSB
    io_conf.pin_bit_mask = ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1));
    // 内部プルダウンしない
    io_conf.pull_down_en = 0;
    // 内部プルアップしない
    io_conf.pull_up_en = 0;
    // 設定をセットする
    gpio_config(&io_conf);
 
 
    int cnt = 0;
    while(1) {
        // カウンタcntをインクリメント
        printf("cnt: %d\n", cnt++);
        // 1秒停止
        vTaskDelay(1000 / portTICK_RATE_MS);
        // gpioの出力をセットする
        // 0: Lo, 1: Hi
        // cntが奇数のとき、cnt % 2が1になるので、出力はHi
        // cntが偶数のとき、cnt % 2が0になるので、出力はHi
        gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
        gpio_set_level(GPIO_OUTPUT_IO_1, cnt % 2);
    }
}
