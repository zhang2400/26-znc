//
// Created by EiveLL on 25-2-2.
//

#ifndef CONFIG_H
#define CONFIG_H

#include "cstring"
#include "cstdint"
#include <ctgmath>

enum PWM_Pin {
    PWM0_GPIO64 = 0,
    PWM1_GPIO65 = 1,
    PWM2_GPIO66 = 2,
    PWM3_GPIO67 = 3,
    GTIM1_GPIO88 = 88,
    GTIM2_GPIO89 = 89,
};

#define PWM_DUTY_MAX 10000

enum Polarity {
    POLARITY_NORMAL,
    POLARITY_INVERTED
};

#define CAR_WHEELBASE_L 0.2
#define CAR_WHEELBASE_B 0.16

// 图像处理
#define DISTANCE(x1, y1, x2, y2) sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2))
#define IMAGE_MIDDLE            33

// MATH
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define ABS(x) ((x) > 0 ? (x) : -(x))


// 舵机配置
#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#define SERVO_MOTOR_L_MAX           (70)             // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_R_MAX           (100)              // 定义主板上舵机活动范围 角度
#define SERVO_MOTOR_MID             (85)              // 定义主板上舵机中间位置 角度

#define SERVO_MOTOR_PWM             (GTIM1_GPIO88)                 // 定义主板上舵机对应引脚
#define SERVO_MOTOR_FREQ            (50)              // 定义主板上舵机频率

#define GYROZ_CORRECT_DEFAULT -0.2

// 占空比最大值
#define PWM_DUTY_MAX 10000

#define WHEEL_MAX_DUTY              (4000)  // 电机最大占空比

#endif //CONFIG_H
