//
// Created by EiveLL on 25-1-23.
//

#include "servo.h"
#include <cstdint>

typedef uint32_t uint32;

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化舵机
// 参数说明     void
// 返回参数     void
// 备注信息     初始化舵机
void servo_init(void)
{
    pwm_init(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, 0);
    servo_set_angle(SERVO_MOTOR_MID);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     设置舵机角度
// 参数说明     angle  角度
// 返回参数     void
// 备注信息     设置舵机角度,角度范围70-100
//-------------------------------------------------------------------------------------------------------------------
void servo_set_angle(double angle){
    // 不直接使用 pwm_set_duty()，使用这个函数控制舵机角度
    if(angle > SERVO_MOTOR_R_MAX){
        angle = SERVO_MOTOR_R_MAX;
    } else if(angle < SERVO_MOTOR_L_MAX){
        angle = SERVO_MOTOR_L_MAX;
    }
    pwm_set_duty(SERVO_MOTOR_PWM, (uint32)SERVO_MOTOR_DUTY(angle));
}


