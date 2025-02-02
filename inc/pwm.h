//
// Created by EiveLL on 25-1-21.
//

#ifndef PWM_H
#define PWM_H

#include <stdint.h>

#define PWM_DUTY_MAX            10000 // PWM???????
#define CMU_CLK_FREQ            1000000000.0f // CMU??????1GHz

typedef enum // ???PWM????
{
    PWM_TIM0_GPIO64,   PWM_TIM1_GPIO65,   PWM_TIM2_GPIO66,   PWM_TIM3_GPIO67,
}pwm_channel_enum;

int get_pwmchip(pwm_channel_enum tim_pin);
void pwm_init(pwm_channel_enum tim_pin, uint32_t freq, uint32_t duty);
void pwm_set_duty(pwm_channel_enum tim_pin, uint32_t duty);
void pwm_test(pwm_channel_enum pwm_pin, uint32_t freq);

#endif

