//
// Created by EiveLL on 25-6-11.
//

#ifndef SERVO_GTIM_H
#define SERVO_GTIM_H

#include "PwmController.h"

class Servo_Gtim : public PwmController {
public:
    // 构造函数：指定PWM通道、频率、角度范围
    explicit Servo_Gtim(
        int pwmchip = 4,                // PWM芯片号
        int pwmnum = 1,
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

#endif //SERVO_GTIM_H
