/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-11-30 08:14:34
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2024-12-01 13:23:26
 * @FilePath: /ls2k0300_peripheral_library/lib/pwm_gtim.h
 * @Description: 基于LS2K0300 GTIMER的PWM控制器类，可使用复用为TIM2_CHx的引脚
 *
 * Copyright (c) 2024 by ilikara 3435193369@qq.com, All Rights Reserved.
 */
#ifndef PWM_GTIM_H_
#define PWM_GTIM_H_

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>
#include "config.h"

#include "register.h"

#define GPIO_MUX_BASE_ADDR 0x16000490

#define GTIM_BASE_ADDR 0x16119000
#define GTIM_CR1_OFFSET 0x00
#define GTIM_CR2_OFFSET 0x04
#define GTIM_SMCR_OFFSET 0x08
#define GTIM_DIER_OFFSET 0x0C
#define GTIM_SR_OFFSET 0x10
#define GTIM_EGR_OFFSET 0x14
#define GTIM_CCMR1_OFFSET 0x18
#define GTIM_CCMR2_OFFSET 0x1C
#define GTIM_CCER_OFFSET 0x20
#define GTIM_CNT_OFFSET 0x24
#define GTIM_PSC_OFFSET 0x28
#define GTIM_ARR_OFFSET 0x2C
#define GTIM_CCR1_OFFSET 0x34
#define GTIM_CCR2_OFFSET 0x38
#define GTIM_CCR3_OFFSET 0x3C
#define GTIM_CCR4_OFFSET 0x40
#define GTIM_INSTA_OFFSET 0x50

class PWM_GTIM
{
public:
    PWM_GTIM(int gpio, int mux, int chNum_, float frequency, int duty_value);
    ~PWM_GTIM(void);

    void enable(void);
    void disable(void);


    void set_frequency(uint32_t freq_hz);           // 设置频率（Hz）
    void set_duty(uint32_t duty);      // 设置占空比（0~PWM_DUTY_MAX）

    void update_period();
    void update_duty();


    uint32_t period_10ns, duty_cycle_10ns;

private:
    float period_ns;        // 当前周期（ns）
    float duty_ns;          // 当前占空比（ns）
    uint32_t chNum;
    void *ccmr_buffer[2];
    void *ccer_buffer;
    void *period_buffer;
    void *duty_cycle_buffer;
    void *cnt_buffer;
};

#endif
