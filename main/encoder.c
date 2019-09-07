
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"

#include "parameters.h"
#include "variables.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
static const char *TAG="Encoder";

#define GPIO_HSPI_MISO 12
#define GPIO_HSPI_MOSI 13
#define GPIO_HSPI_CLK  14
#define GPIO_HSPI_CS0  15
#define GPIO_HSPI_CS1  27


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


void TaskReadEncoders(void *arg){
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


    ESP_LOGI(TAG, "Complete initialization.");
    while(1){
        gWheelAngle[LEFT] = getAngle(spi_l);
        gWheelAngle[RIGHT] = getAngle(spi_r);

        // printf("02_TaskEncoder\n");
        vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

