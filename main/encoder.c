
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/timer.h"

#include "parameters.h"
#include "variables.h"

#define GPIO_HSPI_MISO 12
#define GPIO_HSPI_MOSI 13
#define GPIO_HSPI_CLK  14
#define GPIO_HSPI_CS0  15
#define GPIO_HSPI_CS1  27

#define TIMER_DIVIDER  16  //  Hardware timer clock divider
#define TIMER_SCALE    (TIMER_BASE_CLK / TIMER_DIVIDER)  // convert counter value to seconds
#define TIMER_GROUP    TIMER_GROUP_0
#define TIMER_ID       TIMER_0 


const uint16_t RESOLUTION = 4096;

uint16_t read2Byte(spi_device_handle_t spi){
    // 2バイト読み込み
    // 送信データはダミー

    esp_err_t ret;
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));       // Zero out the transaction
    t.length = 16;                  // SPI_ADDRESS(8bit) + SPI_DATA(8bit)
    t.flags = SPI_TRANS_USE_RXDATA;


    uint16_t tx_data = 0x00;
    tx_data = SPI_SWAP_DATA_TX(tx_data , 16);
    t.tx_buffer = &tx_data;

    ret=spi_device_polling_transmit(spi, &t);  // Transmit!
    assert(ret==ESP_OK);            // Should have had no issues.

    uint16_t data = SPI_SWAP_DATA_RX(*(uint16_t*)t.rx_data, 16);

    return data;
}

float getAngle(spi_device_handle_t spi){
    uint16_t rawData = read2Byte(spi);

    rawData >>= 4; // LSBから4bitは常に0なので、シフトする

    float angle =  2.0*M_PI * (float)rawData / RESOLUTION;

    return angle;
}

float normalize(const float angle){
    float normalizedAngle = angle;

    while(normalizedAngle > M_PI){
        normalizedAngle -= 2.0*M_PI;
    }
    while(normalizedAngle < -M_PI){
        normalizedAngle += 2.0*M_PI;
    }

    return normalizedAngle;
}

void updateMeasurement(const float angleLeft, const float angleRight, const double currentTime){
    // 移動距離を計算する
    static float prevLeft, prevRight;
    static float prevVelocity[SIDE_NUM];
    static double prevTime;

    float diffAngle[SIDE_NUM];

    diffAngle[LEFT] = normalize(angleLeft - prevLeft);
    diffAngle[RIGHT] = normalize(angleRight - prevRight);
    double diffTime = currentTime - prevTime;
    
    // エンコーダの取り付け向きの都合上、左側の符号を反転する
    diffAngle[LEFT] *= -1.0;

    // 走行距離を加算
    gMovingDistance += TIRE_RADIUS * (diffAngle[LEFT]+ diffAngle[RIGHT]) / 2.0;

    // タイヤの角速度を計算
    float angularVelocity[SIDE_NUM];
    angularVelocity[LEFT] = diffAngle[LEFT] / diffTime;
    angularVelocity[RIGHT] = diffAngle[RIGHT] / diffTime;
    // 角速度をタイヤの周速度に変換
    float velocity[SIDE_NUM];
    velocity[LEFT] = angularVelocity[LEFT] * TIRE_RADIUS;
    velocity[RIGHT] = angularVelocity[RIGHT] * TIRE_RADIUS;
    // ローパスフィルタをかける
    velocity[LEFT] = velocity[LEFT] * 0.1 + prevVelocity[LEFT] * 0.9;
    velocity[RIGHT] = velocity[RIGHT] * 0.1 + prevVelocity[RIGHT] * 0.9;

    // 車体速度に変換
    gMeasuredSpeed = (velocity[LEFT] + velocity[RIGHT]) / 2.0;

    prevLeft = angleLeft;
    prevRight = angleRight;
    prevTime = currentTime;
    prevVelocity[LEFT] = velocity[LEFT];
    prevVelocity[RIGHT] = velocity[RIGHT];
}

void TaskReadEncoders(void *arg){
    printf("TaskReadEncoders start\n");

    esp_err_t ret;
    spi_device_handle_t spi_l, spi_r;
    spi_bus_config_t buscfg={
        .miso_io_num=GPIO_HSPI_MISO,
        .mosi_io_num=GPIO_HSPI_MOSI,
        .sclk_io_num=GPIO_HSPI_CLK,
        .quadwp_io_num=-1, // unused
        .quadhd_io_num=-1, // unused
        .max_transfer_sz=4 // bytes
    };
    spi_device_interface_config_t devcfg={
        .clock_speed_hz=1*1000*1000,   //Clock out at 1 MHz
        .mode=3,                    //SPI mode 3
        .spics_io_num=GPIO_HSPI_CS0,   //CS pin
        .queue_size=7,              //We want to be able to queue 7 transactions at a time
    };
    // Initialize the SPI bus
    ret=spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi_l);
    ESP_ERROR_CHECK(ret);

    devcfg.spics_io_num=GPIO_HSPI_CS1;
    ret=spi_bus_add_device(HSPI_HOST, &devcfg, &spi_r);
    ESP_ERROR_CHECK(ret);

    // Initialize Timer
    timer_config_t config;
    config.divider = TIMER_DIVIDER;
    config.counter_dir = TIMER_COUNT_UP;
    config.counter_en = TIMER_PAUSE;
    config.alarm_en = TIMER_ALARM_DIS;
    config.intr_type = TIMER_INTR_LEVEL;
    config.auto_reload = 1;
    timer_init(TIMER_GROUP, TIMER_ID, &config);
    /* Timer's counter will initially start from value below.
       Also, if auto_reload is set, this value will be automatically reload on alarm */
    timer_set_counter_value(TIMER_GROUP, TIMER_ID, 0x00000000ULL);
    timer_start(TIMER_GROUP, TIMER_ID);

    printf("TaskReadEncoders initialized\n");
    double currentTime;

    while(1){
        gWheelAngle[LEFT] = getAngle(spi_l);
        gWheelAngle[RIGHT] = getAngle(spi_r);

        // 時間取得
        timer_get_counter_time_sec(TIMER_GROUP, TIMER_ID, &currentTime);

        updateMeasurement(gWheelAngle[LEFT], gWheelAngle[RIGHT], currentTime);

        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

