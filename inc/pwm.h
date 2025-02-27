//
// Created by ashkore on 25-2-9.
//
#ifndef PWM_H
#define PWM_H

#include <cstdint>
#include "register.h"
#include <config.h>

#define PWM_BASE_ADDR      0x1611B000
#define PWM_OFFSET         0x10
#define DUTY_REG_OFFSET    0x4    // LOW_BUFFER
#define PERIOD_REG_OFFSET  0x8    // FULL_BUFFER
#define CTRL_REG_OFFSET    0xC

// 寄存器位定义
#define CTRL_EN     (1 << 0)
#define CTRL_OE     (1 << 3)
#define CTRL_INVERT (1 << 9)

class PWM {
public:
    explicit PWM(int pwmNum);
    ~PWM();

    // 核心函数
    void set_frequency(uint32_t freq_hz);           // 设置频率（Hz）
    void set_duty(uint32_t duty);      // 设置占空比（0~PWM_DUTY_MAX）
    void set_polarity(enum Polarity polarity);      // 设置极性
    void enable();
    void disable();

private:
    uint32_t base_addr;
    float period_ns;        // 当前周期（ns）
    float duty_ns;          // 当前占空比（ns）
    enum Polarity current_polarity;

    void *ctrl_reg;
    void *duty_reg;
    void *period_reg;

    void init();
    void update_period();
    void update_duty();
    void update_polarity();
};

#endif