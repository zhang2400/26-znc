//
// Created by EiveLL on 25-1-23.
//

#ifndef PID_H
#define PID_H

#include <cstdint>
#include <cmath>

typedef float float32;
typedef uint8_t uint8;

typedef struct{
    float32 Kp;
    float32 Kp2;
    float32 Ki;
    float32 Kd;
    float32 GzKd;
    float32 last_error;
    float32 last_out;
    float32 integral;
    float32 outmax;
    float32 outmin;
    uint8 use_lowpass_filter;
    float32 lowpass_filter_factor;
} PID_Position;

typedef struct{
    float32 Kp;
    float32 Ki;
    float32 Kd;
    float32 error;
    float32 last_error;
    float32 last_last_error;
    float32 last_out;
    float32 out;
    float32 outmax;
    float32 outmin;
    uint8 use_lowpass_filter;  // 驱动板或电机发热太大可以开，牺牲一点响应速度
    float32 lowpass_filter_factor;
} PID_Incremental;

PID_Position PID_Position_Init(float32 Kp, float32 Kp2, float32 Ki, float32 Kd, float32 GzKd, float32 outmax, float32 outmin, uint8 use_lowpass_filter, float32 lowpass_filter_factor);

float32 PID_Position_Calc(PID_Position *pid, float32 input_value, float32 gyroz_value, float32 setpoint);

PID_Incremental PID_Incremental_Init(float32 Kp, float32 Ki, float32 Kd, float32 outmax, float32 outmin, uint8 use_lowpass_filter, float32 lowpass_filter_factor);

float32 PID_Incremental_Calc(PID_Incremental *pid, float32 input_value, float32 setpoint);


#endif //PID_H
