/*
* @Author: ilikara 3435193369@qq.com
 * @Date: 2024-11-30 09:06:41
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2024-12-12 06:21:45
 * @FilePath: /ls2k0300_peripheral_library/src/main.cpp
 * @Description: 测试用主程序
 *
 * Copyright (c) 2024 by ilikara 3435193369@qq.com, All Rights Reserved.
 */
#include "GPIO.h"
#include "pwm.h"
#include <thread>
#include "pwm_gtim.h"
#include "pwm_atim.h"
#include "encoder.h"
#include "pit_sw.h"
#include "encoder.h"
#include <iostream>
#include "icm20602.h"

#include "servo.h"
#include "log.h"
ENCODER* L_Encoder = nullptr;
ENCODER* R_Encoder = nullptr;
int fd;

void image_processing_loop() {
    PWM pwm0(PWM0_GPIO64);
    pwm0.set_frequency(1000);
    pwm0.set_duty(1234);
    pwm0.enable();
//
//    PWM pwm1(1);
//    pwm1.set_frequency(2000);
//    pwm1.set_duty(680);
//    pwm1.enable();
//
//    PWM pwm2(2);
//    pwm2.set_frequency(30000);
//    pwm2.set_duty(5000);
//    pwm2.enable();
//
//    PWM pwm3(3);
//    pwm3.set_frequency(4000);
//    pwm3.set_duty(710);
//    pwm3.enable();
//
    log_init("app.logcat");
    Servo servo;
//
//    pwm0.disable();
//    pwm1.disable();
//    pwm2.disable();
//    pwm3.disable();
    while (true) {
        for(int i = SERVO_MOTOR_L_MAX; i <= SERVO_MOTOR_R_MAX; i++) {
            MEASURE_TIME("set_angle", {
                servo.set_angle(i);
            });
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        for(int i = SERVO_MOTOR_R_MAX; i >= SERVO_MOTOR_L_MAX; i--) {
            MEASURE_TIME("set_angle", {
                servo.set_angle(i);
            });
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
}

int main()
{
    icm20602_init(fd);
    // 创建GPIO对象，假设使用GPIO编号为73
    GPIO gpio73(73);

    // 设置GPIO方向为输出
    if (!gpio73.setDirection("out")) {
        std::cerr << "Failed to set GPIO direction" << std::endl;
        return 1;
    }

    // 设置GPIO输出值为低电平
    if (!gpio73.setValue(false)) {
        std::cerr << "Failed to set GPIO value" << std::endl;
        return 1;
    }

    int pwmChannel = 2;
    int dirGPIO = 73;

    // 创建编码器对象
    L_Encoder = new ENCODER(pwmChannel, dirGPIO);
    R_Encoder = new ENCODER(pwmChannel, dirGPIO);

    PWM_GTIM test2(88, 0b11, 2, 8000, 2000);
    test2.enable();

    PWM_GTIM test3(89, 0b11, 3, 8000, 4500);
    test3.enable();

    std::thread timerThread(pit_init_ms, 10, timer_interrupt_handler);
    timerThread.detach();  // 分离定时器线程
//
//    // 主线程执行图像处理任务
    image_processing_loop();

    return 0;
}