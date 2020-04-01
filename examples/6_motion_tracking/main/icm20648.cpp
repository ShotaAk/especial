// ESP-IDFプログラミングガイド:
// Ref: https://docs.espressif.com/projects/esp-idf/en/v3.3.1/api-reference/peripherals/spi_master.html
// ICM-20648データシート：
// Ref: https://invensense.tdk.com/wp-content/uploads/2017/07/DS-000179-ICM-20648-v1.2-TYP.pdf

#include "icm20648.h"
#include <cstring>
#include <freertos/task.h>

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
static const char *TAG="Motion";

const int icm20648::FS_SEL_SIZE = 4;
const float icm20648::GYRO_SENSITIVITY[icm20648::FS_SEL_SIZE] = {
    131, 65.5, 32.8, 16.4
};
const float icm20648::ACCEL_SENSITIVITY[icm20648::FS_SEL_SIZE] = {
    16384, 8192, 4096, 2048
};

uint8_t icm20648::transaction(const uint8_t cmd, const uint8_t addr,
        const uint8_t data = 0x00) {
    // ICM-20648と通信するデータ読み込み、書き込み兼用関数
    const size_t DATA_LENGTH = 8;
    uint8_t recv_data = 0;

    spi_transaction_t trans;
    memset(&trans, 0, sizeof(trans));  // 構造体をゼロで初期化
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
    ret=spi_device_polling_transmit(mHandler, &trans);
    assert(ret==ESP_OK);
    
    return recv_data;
}

uint8_t icm20648::readRegister(const uint8_t address){
    // レジスタデータを読み取る
    const uint8_t READ_COMMAND = 1;
    uint8_t raw_data = transaction(READ_COMMAND, address, 0x00);

    return raw_data;
}

uint16_t icm20648::readRegister2Byte(const uint8_t addrHigh,
    const uint8_t addrLow){

    uint8_t highData = readRegister(addrHigh);
    uint8_t lowData  = readRegister(addrLow);

    return highData << 8 | lowData;
}

uint8_t icm20648::writeRegister(const uint8_t address, const uint8_t data){
    // レジスタにデータを書き込む
    const uint8_t WRITE_COMMAND = 0;

    ESP_LOGD(TAG, "Write Register addr: %x, data :%x\n", int(address), int(data));
    vTaskDelay(10 / portTICK_PERIOD_MS);  // 書き込み前にディレイを入れないと動かない
    uint8_t raw_data = transaction(WRITE_COMMAND, address, data);

    return raw_data;
}

bool icm20648::changeUserBank(const uint8_t bank){
    // レジスタのユーザバンクを切り替える
    const uint8_t ADDR_REG_BANK_SEL = 0x7f;
    const uint8_t BANK_MAX = 3;

    if(bank > BANK_MAX){
        return false;
    }
    writeRegister(ADDR_REG_BANK_SEL, bank);

    return true;
}

void icm20648::spidevInit(const int mosi_io_num, const int miso_io_num, 
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
    ret = spi_bus_add_device(VSPI_HOST, &devcfg, &mHandler);
    ESP_ERROR_CHECK(ret);
}

bool icm20648::writeGyroConfig(const uint8_t fssel,
    const bool enableLPF=false, const uint8_t configLPF=0){

    const uint8_t ADDR_GYRO_CONFIG_1 = 0x01;
    const uint8_t LPFCFG_MAX = 7;

    if(fssel >= FS_SEL_SIZE || configLPF > LPFCFG_MAX){
        return false;
    }

    uint8_t data=0;
    data |= configLPF << 3;
    data |= fssel << 1; 
    data |= enableLPF;

    changeUserBank(2);
    writeRegister(ADDR_GYRO_CONFIG_1, data);
    changeUserBank(0);

    return true;
}

float icm20648::getGyro(const AXIS axis){
    const uint8_t ADDR_GYRO_OUT_H[AXIS_SIZE] = {0x33, 0x35, 0x37};
    const uint8_t ADDR_GYRO_OUT_L[AXIS_SIZE] = {0x34, 0x36, 0x38};

    uint16_t rawData = readRegister2Byte(
        ADDR_GYRO_OUT_H[axis], 
        ADDR_GYRO_OUT_L[axis]
        );

    return float(rawData) / GYRO_SENSITIVITY[mGyroFSSel];
}

icm20648::icm20648(const int mosi_io_num, const int miso_io_num, 
    const int sclk_io_num, const int cs_io_num){

    spidevInit(mosi_io_num, miso_io_num, sclk_io_num, cs_io_num);

    pwr_mgmt_t mgmt;
    mgmt.reset_device = true;
    mgmt.enable_sleep_mode = false;
    mgmt.enable_low_power = false;
    mgmt.disable_temp_sensor = false;
    mgmt.clock_source = 1;  // auto select
    mgmt.disable_accel = 0b111;  // disable all
    mgmt.disable_gyro = 0b111;  // disable all
    writePwrMgmt(&mgmt);

    // writeGyroConfig(3, false, 0);


    mgmt.reset_device =false;
    mgmt.disable_accel = 0b000;  // enable all
    mgmt.disable_gyro = 0b000;  // enable all
    writePwrMgmt(&mgmt);
}

int icm20648::readWhoAmI(void){
    const uint8_t ADDR_WHO_AM_I = 0x00;
    return readRegister(ADDR_WHO_AM_I);
}

void icm20648::writePwrMgmt(pwr_mgmt_t *mgmt){
    const uint8_t ADDR_PWR_MGMT_1 = 0x06;
    const uint8_t ADDR_PWR_MGMT_2 = 0x07;
    const uint8_t CLOCK_SOURCE_MAX = 7;
    const uint8_t DISABLE_ACCEL_GYRO_MAX = 7;

    uint8_t data = 0;
    data |= mgmt->reset_device << 7;
    data |= mgmt->enable_sleep_mode << 6;
    data |= mgmt->enable_low_power << 5;
    data |= mgmt->disable_temp_sensor << 3;

    if(mgmt->clock_source <= CLOCK_SOURCE_MAX){
        data |= uint8_t(mgmt->clock_source);
    }
    writeRegister(ADDR_PWR_MGMT_1, data);

    data = 0;

    if(mgmt->disable_accel <= DISABLE_ACCEL_GYRO_MAX){
        data |= uint8_t(mgmt->disable_accel) << 3;
    }
    if(mgmt->disable_gyro <= DISABLE_ACCEL_GYRO_MAX){
        data |= uint8_t(mgmt->disable_gyro);
    }
    writeRegister(ADDR_PWR_MGMT_2, data);
}

float icm20648::getGyroX(void){
    return getGyro(AXIS_X);
}

float icm20648::getGyroY(void){
    return getGyro(AXIS_Y);
}

float icm20648::getGyroZ(void){
    return getGyro(AXIS_Z);
}