#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"

// 公式マニュアル:
// Ref: https://docs.espressif.com/projects/esp-idf/en/stable/api-reference/peripherals/mcpwm.html

static const gpio_num_t GPIO_NSLEEP = GPIO_NUM_33;

enum SIDE{
    LEFT = 0,
    RIGHT,
    SIDE_SIZE
};

static const mcpwm_unit_t UNIT = MCPWM_UNIT_0;
static const gpio_num_t GPIO_PH[SIDE_SIZE] = {
    GPIO_NUM_16, GPIO_NUM_25};
static const gpio_num_t GPIO_EN[SIDE_SIZE] = {
    GPIO_NUM_17, GPIO_NUM_26};

static const mcpwm_io_signals_t SIGNAL_PH[SIDE_SIZE] = {
    MCPWM0A, MCPWM1A};
static const mcpwm_io_signals_t SIGNAL_EN[SIDE_SIZE] = {
    MCPWM0B, MCPWM1B};

static const mcpwm_timer_t TIMER[SIDE_SIZE] = {
    MCPWM_TIMER_0, MCPWM_TIMER_1};

static const mcpwm_operator_t OPR_PH = MCPWM_OPR_A;
static const mcpwm_operator_t OPR_EN = MCPWM_OPR_B;

static const mcpwm_duty_type_t DUTY_MODE = MCPWM_DUTY_MODE_0; // アクティブハイ


static void drive_forward(mcpwm_timer_t timer_num , float duty)
{
    mcpwm_set_signal_low(UNIT, timer_num, OPR_PH);
    mcpwm_set_duty(UNIT, timer_num, OPR_EN, duty);
    // set_signal_low/highを実行した後は、毎回set_duty_typeを実行すること
    mcpwm_set_duty_type(UNIT, timer_num, OPR_EN, DUTY_MODE);
}

static void drive_backward(mcpwm_timer_t timer_num , float duty)
{
    mcpwm_set_signal_high(UNIT, timer_num, OPR_PH);
    mcpwm_set_duty(UNIT, timer_num, OPR_EN, duty);
    // set_signal_low/highを実行した後は、毎回set_duty_typeを実行すること
    mcpwm_set_duty_type(UNIT, timer_num, OPR_EN, DUTY_MODE); 
}

static void drive_brake(mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(UNIT, timer_num, OPR_EN);
}

static void motor_drive(const enum SIDE side, const float duty, const int go_back)
{
    const float DUTY_MAX = 100;
    const float DUTY_MIN = 5;

    float target_duty = duty;
    if(target_duty > DUTY_MAX){
        target_duty = DUTY_MAX;
    }

    if(target_duty < DUTY_MIN){
        drive_brake(TIMER[side]);
    }else{
        if(go_back){
            drive_backward(TIMER[side], duty);
        }else{
            drive_forward(TIMER[side], duty);
        }
    }
}

void app_main()
{
    // ----- GPIOの設定 -----
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = (1ULL<<GPIO_NSLEEP);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    // nSLEEPをONにして、モータの電源OFF
    gpio_set_level(GPIO_NSLEEP, 0);

    // ----- MCPWMの設定 -----
    // GPIOの割り当て
    mcpwm_gpio_init(UNIT, SIGNAL_PH[LEFT],  GPIO_PH[LEFT]);
    mcpwm_gpio_init(UNIT, SIGNAL_EN[LEFT],  GPIO_EN[LEFT]);
    mcpwm_gpio_init(UNIT, SIGNAL_PH[RIGHT], GPIO_PH[RIGHT]);
    mcpwm_gpio_init(UNIT, SIGNAL_EN[RIGHT], GPIO_EN[RIGHT]);

    // MCPWMの詳細設定
    // frequencyの値によりdutyの分解能が変わるので注意すること
    // frequency = 10kHz -> 1%刻みのduty
    // frequency = 100kHz -> 10%刻みのduty
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 10*1000; // PWM周波数= 10kHz,
    pwm_config.cmpr_a = 0; // デューティサイクルの初期値（0%）
    pwm_config.cmpr_b = 0; // デューティサイクルの初期値（0%）
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = DUTY_MODE; // アクティブハイ

    mcpwm_init(UNIT, TIMER[LEFT],  &pwm_config);
    mcpwm_init(UNIT, TIMER[RIGHT], &pwm_config);

    const int WAIT_TIME_MS = 100;
    const int DUTY_STEP = 1;
    const int DUTY_MAX = 50;
    const int DUTY_MIN = 0;

    // モータON
    gpio_set_level(GPIO_NSLEEP, 1);
    while (1) {
        for(int go_back=0; go_back<=1; go_back++){
            // 加速
            for(int duty=DUTY_MIN; duty<DUTY_MAX; duty+=DUTY_STEP){
                motor_drive(LEFT, duty, go_back);
                motor_drive(RIGHT, duty, go_back);
                vTaskDelay(WAIT_TIME_MS / portTICK_RATE_MS);
            }
        
            // 減速
            for(int duty=DUTY_MAX; duty>=DUTY_MIN; duty-=DUTY_STEP){
                motor_drive(LEFT, duty, go_back);
                motor_drive(RIGHT, duty, go_back);
                vTaskDelay(WAIT_TIME_MS / portTICK_RATE_MS);
            }
        }
    }
}
