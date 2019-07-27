
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

#include "wall_detector.h"
#include "variables.h"


#define GPIO_RFLED_0    2
#define GPIO_RFLED_1    4
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_RFLED_0) | (1ULL<<GPIO_RFLED_1))

const uint32_t WALL_DEFAULT_VREF = 1128;        //Use adc2_vref_to_gpio() to obtain a better estimate

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channels[WALL_SENS_NUM] = {ADC_CHANNEL_3, ADC_CHANNEL_6, ADC_CHANNEL_7, ADC_CHANNEL_4};
static const adc_atten_t atten = ADC_ATTEN_DB_6;
static const adc_unit_t unit = ADC_UNIT_1;
   

static void check_efuse()
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}

void updateWallExistence(const float *wallVoltages){
    // 壁のありなし判定をする
    const float THRESH_VOLTAGES[DIREC_NUM] = {
        [DIREC_FRONT] = 0.12, 
        [DIREC_LEFT]  = 0.15, 
        [DIREC_RIGHT] = 0.15, 
        [DIREC_BACK]  = 0.5}; // V

    // printf("voltages,L, R, F: %f, %f, %f\n",
    //         wallVoltages[WALL_SENS_L],
    //         wallVoltages[WALL_SENS_R],
    //         (wallVoltages[WALL_SENS_FL] + wallVoltages[WALL_SENS_FR])*0.5);

    if(wallVoltages[WALL_SENS_L] > THRESH_VOLTAGES[DIREC_LEFT]){
        gIsWall[DIREC_LEFT] = 1;
    }else{
        gIsWall[DIREC_LEFT] = 0;
    }

    if(wallVoltages[WALL_SENS_R] > THRESH_VOLTAGES[DIREC_RIGHT]){
        gIsWall[DIREC_RIGHT] = 1;
    }else{
        gIsWall[DIREC_RIGHT] = 0;
    }

    if( (wallVoltages[WALL_SENS_FL] + wallVoltages[WALL_SENS_FR]) * 0.5 
            > THRESH_VOLTAGES[DIREC_FRONT]){
        gIsWall[DIREC_FRONT] = 1;
    }else{
        gIsWall[DIREC_FRONT] = 0;
    }

}

void TaskDetectWall(void *arg){
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    adc1_config_width(ADC_WIDTH_BIT_12);
    for(int adc_i=0; adc_i<WALL_SENS_NUM; adc_i++){
       adc1_config_channel_atten(channels[adc_i], atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, WALL_DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    gpio_set_level(GPIO_RFLED_0, 0);
    gpio_set_level(GPIO_RFLED_1, 0);

    uint32_t adc_readings[WALL_SENS_NUM] = {0};
    uint32_t adc_offset[WALL_SENS_NUM] = {0};
    float wallVoltages[WALL_SENS_NUM] = {0};
    while (1) {

       // 反射する前のセンサ値を取得(オフセット)
       adc_offset[WALL_SENS_L] = adc1_get_raw((adc1_channel_t)channels[WALL_SENS_L]);
       adc_offset[WALL_SENS_FR] = adc1_get_raw((adc1_channel_t)channels[WALL_SENS_FR]);
       // LED ON
       gpio_set_level(GPIO_RFLED_0, 1);

       // 壁に反射するまで待つ
       vTaskDelay(1 / portTICK_RATE_MS);
       // ADC
       adc_readings[WALL_SENS_L] = adc1_get_raw((adc1_channel_t)channels[WALL_SENS_L]);
       adc_readings[WALL_SENS_FR] = adc1_get_raw((adc1_channel_t)channels[WALL_SENS_FR]);
       // LED OFF
       gpio_set_level(GPIO_RFLED_0, 0);


       // 反射する前のセンサ値を取得(オフセット)
       adc_offset[WALL_SENS_R] = adc1_get_raw((adc1_channel_t)channels[WALL_SENS_R]);
       adc_offset[WALL_SENS_FL] = adc1_get_raw((adc1_channel_t)channels[WALL_SENS_FL]);
       // LED ON
       gpio_set_level(GPIO_RFLED_1, 1);

       // 壁に反射するまで待つ
       vTaskDelay(1 / portTICK_RATE_MS);
       // ADC
       adc_readings[WALL_SENS_R] = adc1_get_raw((adc1_channel_t)channels[WALL_SENS_R]);
       adc_readings[WALL_SENS_FL] = adc1_get_raw((adc1_channel_t)channels[WALL_SENS_FL]);
       // LED OFF
       gpio_set_level(GPIO_RFLED_1, 0);

       for(int adc_i=0; adc_i<WALL_SENS_NUM; adc_i++){
           // printf("%d, value, offset: %d, %d\n", adc_i, adc_readings[adc_i], adc_offset[adc_i]);
           // センサ値のオフセットを引く
           if(adc_readings[adc_i] < adc_offset[adc_i]){
               // アンダーフローを防ぐ
               adc_readings[adc_i] = 0;
           }else{
               adc_readings[adc_i] -= adc_offset[adc_i];
           }

           wallVoltages[adc_i] = 
               (float)esp_adc_cal_raw_to_voltage(adc_readings[adc_i], adc_chars) 
               * 0.001; // mVからVに変換する
            
           gWallVoltage[adc_i] = wallVoltages[adc_i];
           updateWallExistence(wallVoltages);
       }
    }
}

