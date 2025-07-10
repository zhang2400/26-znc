//
// Created by EiveLL on 25-1-23.
//

#include "servo.h"

void servo_set_angle(float angle) {
    if (angle > SERVO_MOTOR_R_MAX) {
        angle = SERVO_MOTOR_R_MAX;
    } else if (angle < SERVO_MOTOR_L_MAX) {
        angle = SERVO_MOTOR_L_MAX;
    }
    pwm_set_duty(SERVO_MOTOR_PWM, (uint16)SERVO_MOTOR_DUTY(angle));
}

void servo_set_duty(uint16 duty) {
    pwm_set_duty(SERVO_MOTOR_PWM, (uint16)duty);
}
