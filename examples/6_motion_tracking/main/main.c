#include <stdio.h>
#include <math.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/spi_master.h"

// ESP-IDFプログラミングガイド:
// Ref: https://docs.espressif.com/projects/esp-idf/en/v3.3.1/api-reference/peripherals/spi_master.html
// ICM-20648データシート：
// Ref: https://invensense.tdk.com/wp-content/uploads/2017/07/DS-000179-ICM-20648-v1.2-TYP.pdf

const uint8_t ADDR_WHO_AM_I = 0x00;

const size_t DATA_LENGTH = 8;

uint8_t transaction(const spi_device_handle_t spidev, 
    const uint8_t cmd, const uint8_t addr, const uint8_t data){
    // ICM-20648と通信するデータ読み込み、書き込み兼用関数
    
    uint8_t recv_data;
    spi_transaction_t trans;
    memset(&trans, 0, sizeof(trans)); // 構造体をゼロで初期化
    // flags: SPI_TRANS_ではじまるフラグを設定できる
    // trans.flags = SPI_TRANS_USE_RXDATA;
    trans.cmd = cmd;
    trans.addr = addr;
    trans.length = DATA_LENGTH; // データ長 bit
    // trans.rxlength = 16; // デフォルトでlengthと同じになるので設定不要　
    // trans.user = NULL; // ユーザ定義の変数、コールバックを使うときに役立つ
    trans.tx_buffer = &data; // 送信バッファのポインタ
    // trans.tx_data; // SPI_TRANS_USE_TXDATAフラグを立てれば使用可能
    trans.rx_buffer = &recv_data; // 受信バッファのポインタ
    // trans.rx_data; // SPI_TRANS_USE_RXDATAのフラグを立てれば使用可能

    // 通信開始
    esp_err_t ret;
    ret=spi_device_polling_transmit(spidev, &trans);
    assert(ret==ESP_OK);

    return recv_data;
}

uint8_t readRegister(const spi_device_handle_t spidev, const uint8_t address){
    // レジスタデータを読み取る
    const uint8_t READ_COMMAND = 1;

    uint8_t raw_data = transaction(spidev, READ_COMMAND, address, 0x00);

    return raw_data;
}

void app_main(){
    static const gpio_num_t GPIO_VSPI_CS = GPIO_NUM_5;
    static const gpio_num_t GPIO_VSPI_SCLK = GPIO_NUM_18;
    static const gpio_num_t GPIO_VSPI_MOSI = GPIO_NUM_19;
    static const gpio_num_t GPIO_VSPI_MISO = GPIO_NUM_21;

    esp_err_t ret;
    // SPIバスの設定
    spi_bus_config_t buscfg = {
        .mosi_io_num = GPIO_VSPI_MOSI, // Master Out Slave Inのピン
        .miso_io_num = GPIO_VSPI_MISO, // Master In Slave Outのピン
        .sclk_io_num = GPIO_VSPI_SCLK, // SPI Clockのピン
        .quadwp_io_num = -1, // Quad SPIのWPピン。使わないので-1をセット。
        .quadhd_io_num = -1, // Quad SPIのHDピン。使わないので-1をセット。
        .max_transfer_sz = 2, // 最大送信バイト数。
        // flags: SPICOMMON_BUSFLAG_で始まるフラグをセットできる
        .flags = SPICOMMON_BUSFLAG_MASTER,
        // .intr_flags = NULL, // 割り込みの優先順位フラグ
    };
    ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);

    // SPIデバイスの設定
    spi_device_interface_config_t devcfg = {
        // MA702では角度取得時にコマンドアドレスを使用しないので、
        //   command_bitsとaddress_bitsはセットしない
        .command_bits = 1, // コマンドフェーズのビット長
        .address_bits = 7, // アドレスフェーズのビット長
        // .dummy_bits = 8, // アドレスフェーズとデータフェーズ間のビット長
        .mode = 3, // SPIのモード
        // .duty_cycle_pos 128, // クロックのデューティ比。デフォルト50%
        // .cs_ena_posttrans = 0, // 半二重通信で送信処理後CSをアクティブにし続けるサイクル数
        .clock_speed_hz = 7*1000*1000, // クロックスピードを7MHzに設定
        // input_delay_ns: SCLKとMISOの間にある、
        //   スレーブのデータが有効になるまでの最大遅延時間。
        //   CSをアクティブにして、MISOが送信されるまでに、追加で遅延を設ける。
        //   8MHz以上のクロックスピードを使うときに必要だけど、
        //   正確な値が分からなければ0を設定してね。
        .input_delay_ns = 0, 
        // .spics_io_num // CSピン。後ほど設定する
        // .flags = NULL, // SPI_DEVICE_で始まるフラグを設定できる
        .queue_size = 1, // transactionのキュー数。1以上の値を入れておく。
        // .pre_cb // transactionが始まる前に呼ばれる関数をセットできる
        // .post_cn // transactionが完了した後に呼ばれる関数をセットできる
    };

    // SPIデバイスハンドラーを使って通信する
    spi_device_handle_t spidev;

    // デバイス設定のCSピンだけ書き換える
    devcfg.spics_io_num = GPIO_VSPI_CS;
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spidev);
    ESP_ERROR_CHECK(ret);

    while(1){
        uint8_t raw_data = readRegister(spidev, ADDR_WHO_AM_I);
        printf("who am i: %x\n", raw_data);
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}
