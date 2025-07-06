//
//  Created by EiveLL on 25-6-8.
//

#include "bldc.h"

void bldc_set_duty(uint16 duty) {
    pwm_set_duty(BLDC_MOTOR_PWM, (uint16)duty);
}
