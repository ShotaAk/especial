#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "driver/mcpwm.h"

#include "variables.h"
#include "motor.h"

#define GPIO_PWM0A_OUT 17   //Set GPIO 15 as PWM0A
#define GPIO_PWM0B_OUT 16   //Set GPIO 16 as PWM0B

#define GPIO_PWM1A_OUT 26
#define GPIO_PWM1B_OUT 25

#define GPIO_NSLEEP 33
#define GPIO_OUTPUT_PIN_SEL (1ULL<<GPIO_NSLEEP)


static mcpwm_timer_t MC_TIMER[SIDE_NUM];


static void mcpwm_example_gpio_initialize()
{
    printf("initializing mcpwm gpio...\n");
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0A, GPIO_PWM0A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM0B, GPIO_PWM0B_OUT);

    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1A, GPIO_PWM1A_OUT);
    mcpwm_gpio_init(MCPWM_UNIT_0, MCPWM1B, GPIO_PWM1B_OUT);
}

/**
 * @brief motor moves in forward direction, with duty cycle = duty %
 */
static void brushed_motor_forward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); //call this each time, if operator was previously in low/high state
}

/**
 * @brief motor moves in backward direction, with duty cycle = duty %
 */
static void brushed_motor_backward(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num , float duty_cycle)
{
    mcpwm_set_signal_high(mcpwm_num, timer_num, MCPWM_OPR_B);
    mcpwm_set_duty(mcpwm_num, timer_num, MCPWM_OPR_A, duty_cycle);
    mcpwm_set_duty_type(mcpwm_num, timer_num, MCPWM_OPR_A, MCPWM_DUTY_MODE_0); 
}

/**
 * @brief motor stop
 */
static void brushed_motor_stop(mcpwm_unit_t mcpwm_num, mcpwm_timer_t timer_num)
{
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_A);
    mcpwm_set_signal_low(mcpwm_num, timer_num, MCPWM_OPR_B);
}

static void motorDrive(void){
    // グローバル変数を直接加工しない
    float duty[SIDE_NUM];
    duty[RIGHT] = gMotorDuty[RIGHT];
    duty[LEFT] = gMotorDuty[LEFT];

    for(int side_i=0; side_i < SIDE_NUM; side_i++){
        // dutyを±100に加工する
        if(duty[side_i] < -100){
            duty[side_i] = -100;
        }else if(duty[side_i] > 100){
            duty[side_i] = 100;
        }
        
        if(duty[side_i] < 0){
            // dutyがマイナスのときは逆回転
            duty[side_i] *= -1.0; // dutyの符号を正にする
            brushed_motor_backward(MCPWM_UNIT_0, MC_TIMER[side_i], duty[side_i]);
        }else{
            brushed_motor_forward(MCPWM_UNIT_0, MC_TIMER[side_i], duty[side_i]);
        }
    }
}

void TaskMotorDrive(void *arg)
{

    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = (1ULL<<GPIO_NSLEEP);
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    /* 省略 */

    //1. mcpwm gpio initialization
    mcpwm_example_gpio_initialize();

    //2. initial mcpwm configuration
    printf("Configuring Initial Parameters of mcpwm...\n");
    mcpwm_config_t pwm_config;
    pwm_config.frequency = 100*1000;    //frequency = 100kHz,
    pwm_config.cmpr_a = 0;    //duty cycle of PWMxA = 0
    pwm_config.cmpr_b = 0;    //duty cycle of PWMxb = 0
    pwm_config.counter_mode = MCPWM_UP_COUNTER;
    pwm_config.duty_mode = MCPWM_DUTY_MODE_0;

    MC_TIMER[RIGHT] = MCPWM_TIMER_0;
    MC_TIMER[LEFT] = MCPWM_TIMER_1;

    mcpwm_init(MCPWM_UNIT_0, MC_TIMER[RIGHT], &pwm_config);    //Configure PWM0A & PWM0B with above settings
    mcpwm_init(MCPWM_UNIT_0, MC_TIMER[LEFT], &pwm_config);    //Configure PWM0A & PWM0B with above settings

    while (1) {

        if(gMotorState == MOTOR_ON){
            // nSLEEPをOFFにして、モータの電源ON
            gpio_set_level(GPIO_NSLEEP, 1);
            motorDrive();
        }else{
            // nSLEEPをONにして、モータの電源OFF
            gpio_set_level(GPIO_NSLEEP, 0);
            brushed_motor_stop(MCPWM_UNIT_0, MC_TIMER[RIGHT]);
            brushed_motor_stop(MCPWM_UNIT_0, MC_TIMER[LEFT]);
        }

        // printf("05_TaskMotor\n");
        vTaskDelay(1 / portTICK_RATE_MS);
    }
}

