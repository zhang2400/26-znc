#ifndef ENCODER_H
#define ENCODER_H

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#endif

#include "GPIO.h"
#include "register.h"

#define ENCODER_PPR 32 // 编码器线数

#define PWM_BASE_ADDR 0x1611B000
#define PWM_OFFSET 0x10
#define LOW_BUFFER_OFFSET 0x4
#define FULL_BUFFER_OFFSET 0x8
#define CONTROL_REG_OFFSET 0xC

#define CNTR_ENABLE_BIT (1 << 0)       // 计数器使能
#define PULSE_OUT_ENABLE_BIT (1 << 3)  // 脉冲输出使能（低有效）
#define SINGLE_PULSE_BIT (1 << 4)      // 单脉冲控制位
#define INT_ENABLE_BIT (1 << 5)        // 中断使能
#define INT_STATUS_BIT (1 << 6)        // 中断状态
#define COUNTER_RESET_BIT (1 << 7)     // 计数器重置
#define MEASURE_PULSE_BIT (1 << 8)     // 测量脉冲使能
#define INVERT_OUTPUT_BIT (1 << 9)     // 输出翻转使能
#define DEAD_ZONE_ENABLE_BIT (1 << 10) // 防死区使能

class ENCODER
{
public:
    ENCODER(int pwmNum, int gpioNum);
    ~ENCODER(void);

    double pulse_counter_update(void);

private:
    uint32_t base_addr;
    GPIO directionGPIO;
    void *low_buffer;
    void *full_buffer;
    void *control_buffer;
    void PWM_Init(void);
    void reset_counter(void);
};

#endif