//
// Created by EiveLL on 25-1-23.
//

#ifndef PIT_SW_H
#define PIT_SW_H

#include <iostream>
#include <cstring>
#include <cerrno>
#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <time.h>
#include <sched.h>
#include <sys/timerfd.h>
#include "encoder.h"

// 定义回调函数类型，用户可自定义自己的中断处理逻辑
typedef void (*TimerCallback)(void);

void timer_interrupt_handler(void);
int pit_init_ms(uint32_t time_ms, TimerCallback cb);
int pit_init(uint32_t time_us, TimerCallback cb);
timespec get_current_time();
uint64_t time_diff_ns(const timespec& start, const timespec& end);

extern ENCODER* L_Encoder;
extern ENCODER* R_Encoder;

#endif //PIT_SW_H