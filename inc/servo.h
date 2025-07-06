//
// Created by EiveLL on 25-1-23.
//

#ifndef SERVO_H
#define SERVO_H

#include "zf_driver_pwm.h"
#include "config.h"

void servo_set_angle(float angle);

void servo_set_duty(uint16 duty);

#endif //SERVO_H
