//
// Created by EiveLL on 25-1-23.
//

#ifndef SERVO_H
#define SERVO_H

#include "pwm.h"
#include "config.h"

void servo_init(void);
void servo_set_angle(double angle);

#endif //SERVO_H
