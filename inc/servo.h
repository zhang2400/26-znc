//
// Created by EiveLL on 25-1-23.
//

#ifndef SERVO_H
#define SERVO_H
#include "pwm.h"
#include "config.h"

class Servo : public PWM {
public:
    // 构造函数：指定PWM通道、频率、角度范围
    explicit Servo(
        int pwm_channel = SERVO_MOTOR_PWM,             // PWM通道号
        uint32_t freq_hz = SERVO_MOTOR_FREQ,      // 默认频率333Hz
        float min_angle = SERVO_MOTOR_L_MAX,     // 最小角度（左限位）
        float max_angle = SERVO_MOTOR_R_MAX,    // 最大角度（右限位）
        float mid_angle = SERVO_MOTOR_MID      // 中间位置角度
    );

    // 设置舵机角度（外部接口）
    void set_angle(float angle);

private:
    float min_angle_;   // 舵机最小角度
    float max_angle_;   // 舵机最大角度
    float mid_angle_;   // 舵机中间位置角度

};

#endif //SERVO_H
