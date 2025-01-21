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
#include "pwm_gtim.h"
#include "pwm_atim.h"
#include "encoder.h"
#include <iostream>

int main()
{
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

    // ENCODER encoder(0, 67);
    int gpio_num, ch, per, duty;
    // std::cin >> gpio_num >> ch >> per >> duty;
    // PWM_ATIM test(gpio_num, 0b11, ch, per, duty);

    // PWM_ATIM test(86, 0b11, 3, 200000, 50000, 1);
    // test.enable();
    //
    // for (int i = 0; i < 100; ++i)
    // {
    //     // std::cout << encoder.pulse_counter_update() << std::endl;
    //     // usleep(5000);
    //     std::cin >> per >> duty;
    //     test.setPeriod(per);
    //     test.setDutyCycle(duty);
    // }

    PWM_GTIM test2(88, 0b11, 2, 200000, 50000);
    test2.enable();

    PWM_GTIM test3(89, 0b11, 3, 200000, 50000);
    test3.enable();

    pwm_init(PWM_TIM0_GPIO64, 15000, 5000);
    pwm_set_duty(PWM_TIM0_GPIO64, 2500);

    // pwm_test(PWM_TIM0_GPIO64);

    return 0;
}