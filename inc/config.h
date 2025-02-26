//
// Created by EiveLL on 25-2-2.
//

#ifndef CONFIG_H
#define CONFIG_H

enum PWM_Pin {
    PWM0_GPIO64 = 0,
    PWM1_GPIO65 = 1,
    PWM2_GPIO66 = 2,
    PWM3_GPIO67 = 3
};

// 舵机配置
#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#define SERVO_MOTOR_L_MAX           (70)             // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_R_MAX           (100)              // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_MID             (85)              // 定义主板上舵机中间位置 角度

#define SERVO_MOTOR_PWM             (PWM3_GPIO67)                 // 定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ            (333)              // 定义主板上舵机频率

#define GYROZ_CORRECT_DEFAULT -0.2

// 占空比最大值
#define PWM_DUTY_MAX 10000
#endif //CONFIG_H
