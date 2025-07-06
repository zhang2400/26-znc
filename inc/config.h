//
// Created by EiveLL on 25-2-2.
//

#ifndef CONFIG_H
#define CONFIG_H
#ifdef __cplusplus
#include <cstring>
#include <cstdint>
#include <ctgmath>
#else
#include <string.h>
#include <stdint.h>
#include <math.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

    enum PWM_Pin {
        PWM0_GPIO64 = 0,
        PWM1_GPIO65 = 1,
        PWM2_GPIO66 = 2,
        PWM3_GPIO67 = 3,
        GTIM1_GPIO88 = 88,
        GTIM2_GPIO89 = 89,
    };

// #define PWM_DUTY_MAX 10000

    enum Polarity {
        POLARITY_NORMAL,
        POLARITY_INVERTED
    };

#define CAR_WHEELBASE_L 0.2
#define CAR_WHEELBASE_B 0.16

    // 图像处理
#define DISTANCE(x1, y1, x2, y2) sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2))
#define IMAGE_MIDDLE            37

    // MATH
#define LIMIT(x, min, max) ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define ABS(x) ((x) > 0 ? (x) : -(x))

    // 舵机配置
#define SERVO_MOTOR_DUTY(x)         ((float)PWM_DUTY_MAX/(1000.0/(float)SERVO_MOTOR_FREQ)*(0.5+(float)(x)/90.0))

#define SERVO_MOTOR_MID             (82.5)
#define SERVO_MOTOR_L_MAX           (SERVO_MOTOR_MID - 15)
#define SERVO_MOTOR_R_MAX           (SERVO_MOTOR_MID + 15)

#define SERVO_MOTOR_CHIP            (8)
#define SERVO_MOTOR_NUM             (6)
// #define SERVO_MOTOR_FREQ            (50)


#define BLDC_CHIP                   (8)
#define BLDC_NUM                    (6)
#define BLDC_FREQ                   (50)
#define BLDC_PERIOD                 ((float)1.0 / ((float)SERVO_MOTOR_FREQ / 1000))
#define BLDC_DUTY_MIN               ((float)1.0 / (float)BLDC_PERIOD * (float)PWM_DUTY_MAX)
#define BLDC_DUTY_MAX               ((float)2.0 / (float)BLDC_PERIOD * (float)PWM_DUTY_MAX)
#define BLDC_DUTY                   ((float)1.8 / (float)BLDC_PERIOD * (float)PWM_DUTY_MAX)


    // 定义驱动路劲，该路劲由设备树生成
#define SERVO_MOTOR_PWM            "/dev/zf_device_pwm_esc_1"
#define BLDC_MOTOR_PWM              "/dev/zf_device_pwm_servo"

    // 定义主板上舵机频率  请务必注意范围 50-300
    // 如果要修改，需要直接修改设备树。
#define SERVO_MOTOR_FREQ            (50)

    // 在设备树中，默认设置的10000。如果要修改，需要直接修改设备树。
#define PWM_DUTY_MAX                (10000)

#define WHEEL_MAX_DUTY              (6000)

#ifdef __cplusplus
}
#endif

#endif //CONFIG_H
