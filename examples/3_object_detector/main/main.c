#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/gpio.h"


// 公式マニュアル:
// https://docs.espressif.com/projects/esp-idf/en/stable/api-reference/peripherals/gpio.html
// https://docs.espressif.com/projects/esp-idf/en/stable/api-reference/peripherals/adc.html

void app_main()
{
    // ----- GPIOの設定 -----
    static const gpio_num_t GPIO_L_FR = GPIO_NUM_2;
    static const gpio_num_t GPIO_R_FL = GPIO_NUM_4;

    gpio_config_t io_conf;
    // 割り込みをしない
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    // 出力モード
    io_conf.mode = GPIO_MODE_OUTPUT;
    // 設定したいピンのビットマスク
    io_conf.pin_bit_mask = ((1ULL<<GPIO_L_FR) | (1ULL<<GPIO_R_FL));
    // 内部プルダウンしない
    io_conf.pull_down_en = 0;
    // 内部プルアップしない
    io_conf.pull_up_en = 0;
    // 設定をセットする
    gpio_config(&io_conf);

    // ----- ADCの設定 -----
    enum AN_IR{
        AN_IR_FL = 0,
        AN_IR_L,
        AN_IR_R,
        AN_IR_FR,
        AN_IR_SIZE
    };

    static const adc_unit_t         UNIT = ADC_UNIT_1;
    static const adc_channel_t      CHANNELS[AN_IR_SIZE] = {
        ADC_CHANNEL_3, ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_4};
    // 11dB減衰を設定。フルスケールレンジは3.9V
    static const adc_atten_t        ATTEN = ADC_ATTEN_DB_11;
    // ADCの分解能を12bit (0~4095)に設定
    static const adc_bits_width_t   WIDTH = ADC_WIDTH_BIT_12;
    // eFuseメモリのVrefを使うため、このデフォルト値は使用されない
    static const uint32_t DEFAULT_VREF = 1100;

    adc1_config_width(WIDTH);
    // 各チャンネルの減衰量を設定
    for(int ir_i=0; ir_i<AN_IR_SIZE; ir_i++){
        adc1_config_channel_atten(CHANNELS[ir_i], ATTEN);
    }
    // ADCの特性を設定
    esp_adc_cal_characteristics_t *adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_characterize(UNIT, ATTEN, WIDTH, DEFAULT_VREF, adc_chars);

    uint32_t adc_readings[AN_IR_SIZE] = {0};
    uint32_t adc_offsets[AN_IR_SIZE] = {0};
    uint32_t sense_millivolts[AN_IR_SIZE] = {0};
    while (1) {
        // 反射する前のセンサ値を取得(オフセット)
        adc_offsets[AN_IR_L] = adc1_get_raw((adc1_channel_t)CHANNELS[AN_IR_L]);
        adc_offsets[AN_IR_FR] = adc1_get_raw((adc1_channel_t)CHANNELS[AN_IR_FR]);
        // LED ON
        gpio_set_level(GPIO_L_FR, 1);
        // 壁に反射するまで待つ
        vTaskDelay(1 / portTICK_RATE_MS);
        // ADC
        adc_readings[AN_IR_L] = adc1_get_raw((adc1_channel_t)CHANNELS[AN_IR_L]);
        adc_readings[AN_IR_FR] = adc1_get_raw((adc1_channel_t)CHANNELS[AN_IR_FR]);
        // LED OFF
        gpio_set_level(GPIO_L_FR, 0);


        // 反射する前のセンサ値を取得(オフセット)
        adc_offsets[AN_IR_R] = adc1_get_raw((adc1_channel_t)CHANNELS[AN_IR_R]);
        adc_offsets[AN_IR_FL] = adc1_get_raw((adc1_channel_t)CHANNELS[AN_IR_FL]);
        // LED ON
        gpio_set_level(GPIO_R_FL, 1);
        // 壁に反射するまで待つ
        vTaskDelay(1 / portTICK_RATE_MS);
        // ADC
        adc_readings[AN_IR_R] = adc1_get_raw((adc1_channel_t)CHANNELS[AN_IR_R]);
        adc_readings[AN_IR_FL] = adc1_get_raw((adc1_channel_t)CHANNELS[AN_IR_FL]);
        // LED OFF
        gpio_set_level(GPIO_R_FL, 0);

        
        // オフセットを処理して、ADCの変換結果を電圧値に変換する
        for(int ir_i=0; ir_i<AN_IR_SIZE; ir_i++){
           // センサ値のオフセットを引く
           if(adc_readings[ir_i] < adc_offsets[ir_i]){
               // アンダーフローを防ぐ
               adc_readings[ir_i] = 0;
           }else{
               adc_readings[ir_i] -= adc_offsets[ir_i];
           }

           sense_millivolts[ir_i] =
               esp_adc_cal_raw_to_voltage(adc_readings[ir_i], adc_chars);
        }
        
        printf("FL, L, R, FR(RAW): \t %d\t, %d\t, %d\t, %d\n",
                adc_readings[AN_IR_FL], adc_readings[AN_IR_L],
                adc_readings[AN_IR_R], adc_readings[AN_IR_FR]);
        printf("FL, L, R, FR( mV): \t %d\t, %d\t, %d\t, %d\n",
                sense_millivolts[AN_IR_FL], sense_millivolts[AN_IR_L],
                sense_millivolts[AN_IR_R], sense_millivolts[AN_IR_FR]);

        vTaskDelay(100 / portTICK_RATE_MS);
    }
}


