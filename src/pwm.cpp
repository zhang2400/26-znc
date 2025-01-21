//
// Created by EiveLL on 25-1-21.
//

#include "pwm.h"
#include <iostream>
#include <fstream>
#include <string>
#include <unistd.h>

int period_ns_global = 0;

int get_pwmchip(pwm_channel_enum tim_pin) {
    switch (tim_pin) {
        case PWM_TIM0_GPIO64: return 0;
        case PWM_TIM1_GPIO65: return 1;
        case PWM_TIM2_GPIO66: return 2;
        case PWM_TIM3_GPIO67: return 3;
        default:
            std::cerr << "Invalid tim_pin value" << std::endl;
        return -1;
    }
}

void pwm_init(pwm_channel_enum tim_pin, uint32_t freq, uint32_t duty) {
    int pwmchip = get_pwmchip(tim_pin);
    if (pwmchip == -1) return;

    // 取消导出 PWM 通道
    std::string unexportCommand = "echo 0 > /sys/class/pwm/pwmchip" + std::to_string(pwmchip) + "/unexport";
    system(unexportCommand.c_str());

    // 导出 PWM 通道
    std::string exportCommand = "echo 0 > /sys/class/pwm/pwmchip" + std::to_string(pwmchip) + "/export";
    system(exportCommand.c_str());

    // 将频率转换为纳秒周期
    period_ns_global = CMU_CLK_FREQ / freq;

    std::string periodCommand = "echo " + std::to_string(period_ns_global) + " > /sys/class/pwm/pwmchip" + std::to_string(pwmchip) + "/pwm" + std::to_string(tim_pin) + "/period";
    system(periodCommand.c_str());

    int duty_ns = period_ns_global * duty / PWM_DUTY_MAX;

    std::string dutyCommand = "echo " + std::to_string(duty_ns) + " > /sys/class/pwm/pwmchip" + std::to_string(pwmchip) + "/pwm" + std::to_string(tim_pin) + "/duty_cycle";
    system(dutyCommand.c_str());

    std::string enableCommand = "echo 1 > /sys/class/pwm/pwmchip" + std::to_string(pwmchip) + "/pwm" + std::to_string(tim_pin) + "/enable";
    system(enableCommand.c_str());
}

void pwm_set_duty(pwm_channel_enum tim_pin, uint32_t duty) {
    int pwmchip = get_pwmchip(tim_pin);
    if (pwmchip == -1) return;

    // 计算高电平的持续时间
    int high_level_ns = period_ns_global * duty / PWM_DUTY_MAX;
    // 计算低电平的持续时间
    int low_level_ns = period_ns_global - high_level_ns;

    std::string dutyCommand = "echo " + std::to_string(low_level_ns) + " > /sys/class/pwm/pwmchip" + std::to_string(pwmchip) + "/pwm" + std::to_string(tim_pin) + "/duty_cycle";
    system(dutyCommand.c_str());
}

void pwm_test(pwm_channel_enum tim_pin) {
    const uint32_t freq = 0;
    const uint32_t step = 100; // 每次增加或减少的步长
    const uint32_t delay = 4000000 / (PWM_DUTY_MAX / step); // 每步的延迟时间，确保4秒内完成

    pwm_init(tim_pin, freq, 0); // 初始化频率为0，占空比为0

    while (true) {
        // 占空比从0增加到最大值
        for (uint32_t duty = 0; duty <= PWM_DUTY_MAX; duty += step) {
            pwm_set_duty(tim_pin, duty);
            usleep(delay); // 延迟
        }

        // 占空比从最大值减小到0
        for (uint32_t duty = PWM_DUTY_MAX; duty > 0; duty -= step) {
            pwm_set_duty(tim_pin, duty);
            usleep(delay); // 延迟
        }
    }
}
