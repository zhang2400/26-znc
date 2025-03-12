//
// Created by EiveLL on 25-1-23.
//

#include "servo.h"

Servo::Servo(int pwm_channel, uint32_t freq_hz, float min_angle, float max_angle, float mid_angle)
    : PWM_GTIM(pwm_channel, 0b11, 2, freq_hz, SERVO_MOTOR_DUTY(mid_angle)),
      min_angle_(min_angle),
      max_angle_(max_angle),
      mid_angle_(mid_angle)
{
    set_frequency(freq_hz);
    set_angle(mid_angle_);
    enable();
    printf("servo initialized\n");
}

void Servo::set_angle(float angle) {
    if(angle < min_angle_) angle = min_angle_;
    if(angle > max_angle_) angle = max_angle_;

    uint32_t duty = SERVO_MOTOR_DUTY(angle);
    set_duty(duty);
}
