//
// Created by EiveLL on 25-1-21.
//

#ifndef PWM_H
#define PWM_H

#include <stdint.h>

#define PWM_DUTY_MAX            10000 // PWM最大占空比
#define CMU_CLK_FREQ            1000000000.0f // CMU时钟频率1GHz

typedef enum // 枚举PWM引脚
{
    PWM_TIM0_GPIO64,   PWM_TIM1_GPIO65,   PWM_TIM2_GPIO66,   PWM_TIM3_GPIO67,
}pwm_channel_enum;

int get_pwmchip(pwm_channel_enum tim_pin);
void pwm_init(pwm_channel_enum tim_pin, uint32_t freq, uint32_t duty);
void pwm_set_duty(pwm_channel_enum tim_pin, uint32_t duty);
void pwm_test(pwm_channel_enum tim_pin);

#endif

