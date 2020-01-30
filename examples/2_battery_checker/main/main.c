#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#define DEFAULT_VREF 1100 // eFuseメモリのVrefを使うため、このデフォルト値は使用されない

// 公式マニュアル:
// https://docs.espressif.com/projects/esp-idf/en/stable/api-reference/peripherals/adc.html

void app_main()
{
    // ADC1_CH0はSENSOR_VPピンの機能
    static const adc_unit_t         unit = ADC_UNIT_1;
    static const adc_channel_t      channel = ADC_CHANNEL_0;
    // 11dB減衰を設定。フルスケールレンジは3.9V
    // SENSOR_VPピンにはバッテリ電圧の1/2 (約1.85V）が印加される
    // マニュアルより、
    //   6dBで正確な値が取れる電圧範囲は0.15 ~ 1.75V
    //   11dBで正確な値が取れる電圧範囲は0.15 ~ 2.45V
    // よって、11dB減衰を設定する
    static const adc_atten_t        atten = ADC_ATTEN_DB_11;
    // ADCの分解能を12bit (0~4095)に設定
    static const adc_bits_width_t   width = ADC_WIDTH_BIT_12;

    // ADCのユニットとチャンネルの設定
    adc1_config_width(width);
    adc1_config_channel_atten(channel, atten);

    // ADCの特性を設定
    esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, adc_chars);

    while (1) {
        uint32_t adc_reading = 0;
        static const int NO_OF_SAMPLES = 64; // 平均値を求めるためのサンプル数

        // ADCの変換結果のばらつきを抑えるため、
        // 結果を複数回取得して、平均値を求める
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
        }
        adc_reading /= NO_OF_SAMPLES;

        // ADCの変換結果（0 ~ 4095）を電圧値 mV に変換
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        // ピンにはバッテリ電圧の1/2が印加されるので、voltageを2倍する
        uint32_t battery_voltage = voltage * 2;
        printf("Raw: %d\tVoltage: %dmV\tBatteryVoltage: %dmV\n",
                adc_reading, voltage, battery_voltage);

        // 1秒待機
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}


