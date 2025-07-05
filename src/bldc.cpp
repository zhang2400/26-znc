//
// Created by EiveLL on 25-6-8.
//

#include "bldc.h"

BLDC::BLDC(int pwmchip, int pwmnum, uint32_t freq_hz, uint32_t max_duty, uint32_t min_duty)
    : PwmController(pwmchip, pwmnum),
      min_duty_(min_duty),
      max_duty_(max_duty)
{
    disable();
    initialize();
    set_frequency(freq_hz);
    // set_duty(BLDC_DUTY_MIN);
    enable();
    printf("BLDC initialized\n");
}

void BLDC::set_bldc_duty(uint32_t duty) {
    if(duty < min_duty_) duty = min_duty_;
    if(duty > max_duty_) duty = max_duty_;

    set_duty(duty);
}