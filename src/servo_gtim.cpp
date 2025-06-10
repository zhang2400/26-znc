//
// Created by EiveLL on 25-6-11.
//

#include "servo_gtim.h"

float servo_motor_mid = SERVO_MOTOR_MID;
float servo_motor_l_max = SERVO_MOTOR_L_MAX;
float servo_motor_r_max = SERVO_MOTOR_R_MAX;

Servo_Gtim::Servo_Gtim(int pwmchip, int pwmnum, uint32_t freq_hz, float min_angle, float max_angle, float mid_angle)
    : PwmController(pwmchip, pwmnum),
      min_angle_(min_angle),
      max_angle_(max_angle),
      mid_angle_(mid_angle)
{
    initialize();
    disable();
    set_frequency(freq_hz);
    set_angle(mid_angle_);
    enable();
    printf("servo_gtim initialized\n");
}

void Servo_Gtim::set_angle(float angle) {
    if(angle < min_angle_) angle = min_angle_;
    if(angle > max_angle_) angle = max_angle_;

    uint32_t duty = SERVO_MOTOR_DUTY(angle);
    set_duty(duty);
}
