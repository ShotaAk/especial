#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"

// 公式マニュアル:
// Ref: https://docs.espressif.com/projects/esp-idf/en/v3.3.1/api-reference/peripherals/spi_master.html

enum SIDE{
    LEFT = 0,
    RIGHT,
    SIDE_SIZE
};


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

    ret=spi_device_polling_transmit(spi, &t); // Transmit!
    assert(ret==ESP_OK); // Should have had no issues.

    return SPI_SWAP_DATA_RX(*(uint16_t*)t.rx_data, 16);
}


float getAngle(spi_device_handle_t device){
    const uint16_t RESOLUTION = 4096;

    uint16_t rawData = read2Byte(device);
    rawData >>= 4; // LSBから4bitは常に0なので、シフトする
    return 2.0*M_PI * (float)rawData / RESOLUTION;
}


void app_main()
{
    static const gpio_num_t GPIO_HSPI_CS[SIDE_SIZE] 
        = {GPIO_NUM_15, GPIO_NUM_27};
    static const gpio_num_t GPIO_HSPI_SCLK = GPIO_NUM_14;
    static const gpio_num_t GPIO_HSPI_MOSI = GPIO_NUM_13;
    static const gpio_num_t GPIO_HSPI_MISO = GPIO_NUM_12;

    esp_err_t ret;
    // SPIバスの設定
    spi_bus_config_t buscfg = {
        .miso_io_num = GPIO_HSPI_MISO,
        .mosi_io_num = GPIO_HSPI_MOSI,
        .sclk_io_num = GPIO_HSPI_SCLK,
        .quadwp_io_num = -1, // unused
        .quadhd_io_num = -1, // unused
        .max_transfer_sz = 4 // bytes
    };
    ret = spi_bus_initialize(HSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);

    // SPIデバイスの設定
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1*1000*1000,    //Clock out at 1 MHz
        .mode = 3, //SPI mode 3
        .queue_size = 7, // We want to be able to queue 7 transactions at a time
    };

    spi_device_handle_t spidev[SIDE_SIZE];

    devcfg.spics_io_num = GPIO_HSPI_CS[LEFT];
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spidev[LEFT]);
    ESP_ERROR_CHECK(ret);

    devcfg.spics_io_num = GPIO_HSPI_CS[RIGHT];
    ret = spi_bus_add_device(HSPI_HOST, &devcfg, &spidev[RIGHT]);
    ESP_ERROR_CHECK(ret);

    float wheelAngle[SIDE_SIZE];
    while(1){
        wheelAngle[LEFT] = getAngle(spidev[LEFT]);
        wheelAngle[RIGHT] = getAngle(spidev[RIGHT]);

        // radian to degree
        wheelAngle[LEFT] *= 180.0/M_PI;
        wheelAngle[RIGHT] *= 180.0/M_PI;
        printf("wheelAngle: %f, %f\n",wheelAngle[LEFT], wheelAngle[RIGHT]);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
