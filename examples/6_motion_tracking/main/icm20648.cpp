// ESP-IDFプログラミングガイド:
// Ref: https://docs.espressif.com/projects/esp-idf/en/v3.3.1/api-reference/peripherals/spi_master.html
// ICM-20648データシート：
// Ref: https://invensense.tdk.com/wp-content/uploads/2017/07/DS-000179-ICM-20648-v1.2-TYP.pdf

#include "icm20648.h"
#include <driver/spi_master.h>
#include <cstring>

static spi_device_handle_t spidev_;

uint8_t transaction(const uint8_t cmd, const uint8_t addr, const uint8_t data=0x00){
    // ICM-20648と通信するデータ読み込み、書き込み兼用関数
    const size_t DATA_LENGTH = 8;
    uint8_t recv_data=0;

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
    ret=spi_device_polling_transmit(spidev_, &trans);
    assert(ret==ESP_OK);
    
    return recv_data;
}

uint8_t readRegister(const uint8_t address){
    // レジスタデータを読み取る
    const uint8_t READ_COMMAND = 1;
    uint8_t raw_data = transaction(READ_COMMAND, address, 0x00);

    return raw_data;
}

uint8_t writeRegister(const uint8_t address, const uint8_t data){
    // レジスタにデータを書き込む
    const uint8_t WRITE_COMMAND = 0;
    uint8_t raw_data = transaction(WRITE_COMMAND, address, data);

    return raw_data;
}


void spidev_init(const int mosi_io_num, const int miso_io_num, 
        const int sclk_io_num, const int cs_io_num){
    esp_err_t ret;
    // SPIバスの設定
    spi_bus_config_t buscfg;
    buscfg.mosi_io_num = mosi_io_num; // Master Out Slave Inのピン
    buscfg.miso_io_num = miso_io_num; // Master In Slave Outのピン
    buscfg.sclk_io_num = sclk_io_num; // MasterSPI Clockのピン
    buscfg.quadwp_io_num = -1; // Quad SPIのWPピン。使わないので-1をセット。
    buscfg.quadhd_io_num = -1; // Quad SPIのHDピン。使わないので-1をセット。
    buscfg.max_transfer_sz = 2; // 最大送信バイト数。
    // flags: SPICOMMON_BUSFLAG_で始まるフラグをセットできる
    buscfg.flags = SPICOMMON_BUSFLAG_MASTER;
    buscfg.intr_flags = 0; // 割り込みをしない
    ret = spi_bus_initialize(VSPI_HOST, &buscfg, 1);
    ESP_ERROR_CHECK(ret);
    
    // SPIデバイスの設定
    spi_device_interface_config_t devcfg;
    devcfg.command_bits = 1; // コマンドフェーズのビット長
    devcfg.address_bits = 7; // アドレスフェーズのビット長
    devcfg.dummy_bits = 0, // アドレスフェーズとデータフェーズ間のビット長
    devcfg.mode = 3; // SPIのモード
    devcfg.duty_cycle_pos = 0, // クロックのデューティ比。0で、デフォルトの50%がセットされる。
    devcfg.cs_ena_pretrans = 0; // 送信処理前にCSをアクティブにし続けるサイクル数
    devcfg.cs_ena_posttrans = 0; // 送信処理後にCSをアクティブにし続けるサイクル数
    devcfg.clock_speed_hz = 7*1000*1000; // クロックスピードを7MHzに設定
    // input_delay_ns: SCLKとMISOの間にある、
    //   スレーブのデータが有効になるまでの最大遅延時間。
    //   CSをアクティブにして、MISOが送信されるまでに、追加で遅延を設ける。
    //   8MHz以上のクロックスピードを使うときに必要だけど、
    //   正確な値が分からなければ0を設定してね。
    devcfg.input_delay_ns = 0;
    // devcfg.spics_io_num = NULL, // CSピン。後ほど設定する
    devcfg.flags = 0; // SPI_DEVICE_で始まるフラグを設定できる
    devcfg.queue_size = 1; // transactionのキュー数。1以上の値を入れておく。
    devcfg.pre_cb = NULL; // transactionが始まる前に呼ばれる関数をセットできる
    devcfg.post_cb = NULL;// transactionが完了した後に呼ばれる関数をセットできる
    
    // デバイス設定のCSピンだけ書き換える
    devcfg.spics_io_num = cs_io_num;
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &spidev_);
    ESP_ERROR_CHECK(ret);
}

void icm20648_init(void){
    // ICM20648の初期設定
}

void ICM20648::init(const int mosi_io_num, const int miso_io_num, 
        const int sclk_io_num, const int cs_io_num){
    // SPIデバイスの初期設定
    spidev_init(mosi_io_num, miso_io_num, sclk_io_num, cs_io_num);
    // icm20648_init();
}

int ICM20648::read_who_am_i(void){
    const uint8_t ADDR_WHO_AM_I = 0x00;
    return readRegister(ADDR_WHO_AM_I);
}

int ICM20648::read_accel_x(void){
    const uint8_t ADDR_ACCEL_X_H = 0x2d;
    const uint8_t ADDR_ACCEL_X_L = 0x2e;

    uint8_t accel_h = readRegister(ADDR_ACCEL_X_H);
    uint8_t accel_l = readRegister(ADDR_ACCEL_X_L);

    return accel_h << 8 | accel_l;
}
