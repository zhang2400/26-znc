//
// Created by ashkore on 25-2-9.
//
#include "pwm.h"

PWM::PWM(int pwmNum)
    : base_addr(PWM_BASE_ADDR + pwmNum * PWM_OFFSET),
      period_ns(0),
      duty_ns(0),
      current_polarity(POLARITY_INVERTED) {
    // 映射寄存器
    ctrl_reg = map_register(base_addr + CTRL_REG_OFFSET, PAGE_SIZE);
    duty_reg = map_register(base_addr + DUTY_REG_OFFSET, PAGE_SIZE);
    period_reg = map_register(base_addr + PERIOD_REG_OFFSET, PAGE_SIZE);
    init();
}

PWM::~PWM() {
    disable();
    munmap(ctrl_reg, PAGE_SIZE);
    munmap(duty_reg, PAGE_SIZE);
    munmap(period_reg, PAGE_SIZE);
}

void PWM::init() {
    uint32_t ctrl = REG_READ(ctrl_reg);
    ctrl &= ~CTRL_EN;               // EN=0 禁用PWM
    ctrl |= CTRL_INVERT;            // INVERT=1 反向输出
    ctrl &= ~CTRL_OE;               // OE=0 启用输出
    REG_WRITE(ctrl_reg, ctrl);
}

// 设置频率（单位：Hz）
void PWM::set_frequency(uint32_t freq_hz) {
    if (freq_hz == 0) return;
    period_ns = (float)1'000'000'000 / (float)freq_hz; // 转换为ns
    update_period();
}

// 设置占空比（0~PWM_DUTY_MAX）
void PWM::set_duty(uint32_t duty) {
    if (duty > PWM_DUTY_MAX) duty = PWM_DUTY_MAX;
    duty_ns = period_ns / PWM_DUTY_MAX * (float)duty;
    update_duty();
}

// 更新周期寄存器
void PWM::update_period() {
    auto val = (uint32_t)(period_ns / 10);
    uint32_t reg_value = (val == 0) ? 1 : val;
    REG_WRITE(period_reg, reg_value);
}

// 更新占空比寄存器
void PWM::update_duty() {
    auto val = (uint32_t)(duty_ns / 10);
    uint32_t reg_value = (val == 0) ? 1 : val;
    REG_WRITE(duty_reg, reg_value);
}

// 设置极性
void PWM::set_polarity(enum Polarity polarity) {
    current_polarity = polarity;
    update_polarity();
}

void PWM::update_polarity() {
    uint32_t ctrl = REG_READ(ctrl_reg);
    if (current_polarity == POLARITY_INVERTED) {
        ctrl |= CTRL_INVERT;
    } else {
        ctrl &= ~CTRL_INVERT;
    }
    REG_WRITE(ctrl_reg, ctrl);
}

void PWM::enable() {
    // 先写入周期和占空比
    update_period();
    update_duty();

    uint32_t ctrl = REG_READ(ctrl_reg);
    ctrl |= CTRL_EN;
    REG_WRITE(ctrl_reg, ctrl);
}

void PWM::disable() {
    // 根据极性设置占空比为0或满周期
    if (current_polarity == POLARITY_NORMAL) {
        REG_WRITE(duty_reg, 0); // 正常极性下禁用输出低电平
    } else {
        auto val = (uint32_t)(period_ns / 10);
        REG_WRITE(duty_reg, val); // 反向极性下禁用输出高电平
    }

    uint32_t ctrl = REG_READ(ctrl_reg);
    ctrl &= ~CTRL_EN;
    REG_WRITE(ctrl_reg, ctrl);
}