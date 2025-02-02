//
// Created by EiveLL on 25-1-23.
//

#include "PID.h"

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID位置式初始化
// 参数说明     Kp  比例系数
// 参数说明     Kp2  比例系数2
// 参数说明     Ki  积分系数
// 参数说明     Kd  微分系数
// 参数说明     GzKd  角速度系数
// 参数说明     outmax  输出最大值
// 参数说明     outmin  输出最小值
// 参数说明     use_lowpass_filter  是否使用低通滤波
// 参数说明     lowpass_filter_factor  低通滤波系数
// 返回参数     PID_Position
// 备注信息     PID位置式初始化
//-------------------------------------------------------------------------------------------------------------------
PID_Position PID_Position_Init(float32 Kp, float32 Kp2, float32 Ki, float32 Kd,
                               float32 GzKd, float32 outmax, float32 outmin, uint8 use_lowpass_filter,
                               float32 lowpass_filter_factor){
    PID_Position pid;
    pid.Kp = Kp;
    pid.Kp2 = Kp2;
    pid.Ki = Ki;
    pid.Kd = Kd;
    pid.GzKd = GzKd;
    pid.last_error = 0;
    pid.last_out = 0;
    pid.integral = 0;
    pid.outmax = outmax;
    pid.outmin = outmin;
    pid.use_lowpass_filter = use_lowpass_filter;
    pid.lowpass_filter_factor = lowpass_filter_factor;
    return pid;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID位置式计算
// 参数说明     pid  PID位置式
// 参数说明     input_value  输入值
// 参数说明     gyroz_value  角速度值
// 参数说明     setpoint  设定值
// 返回参数     float32
// 备注信息     PID位置式计算,适用于舵机控制,返回输出值
//-------------------------------------------------------------------------------------------------------------------
float32 PID_Position_Calc(PID_Position *pid, float32 input_value, float32 gyroz_value, float32 setpoint) {
    float32 error = setpoint - input_value;
    float32 derivative = error - pid->last_error;
    pid->integral += error;
    pid->last_error = error;
    float32 output = pid->Kp * error + pid->Kp2 * error * fabsf(error) + pid->Ki * pid->integral + pid->Kd * derivative + pid->GzKd * gyroz_value;

    // 积分限幅
    if(pid->integral > pid->outmax/9){
        pid->integral = pid->outmax/9;
    } else if(pid->integral < pid->outmin/9){
        pid->integral = pid->outmin/9;
    }

    // 输出限幅
    if(output > pid->outmax){
        output = pid->outmax;
    } else if(output < pid->outmin){
        output = pid->outmin;
    }

    // 低通滤波
    if(pid->use_lowpass_filter){
        output = pid->last_out * pid->lowpass_filter_factor + output * (1 - pid->lowpass_filter_factor);
    }

    pid->last_out = output;

    return output;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID增量式初始化
// 参数说明     Kp  比例系数
// 参数说明     Ki  积分系数
// 参数说明     Kd  微分系数
// 参数说明     outmax  输出最大值
// 参数说明     outmin  输出最小值
// 参数说明     use_lowpass_filter  是否使用低通滤波
// 参数说明     lowpass_filter_factor  低通滤波系数
// 返回参数     PID_Incremental
// 备注信息     PID增量式初始化
//-------------------------------------------------------------------------------------------------------------------
PID_Incremental PID_Incremental_Init(float32 Kp, float32 Ki, float32 Kd, float32 outmax,
                                     float32 outmin, uint8 use_lowpass_filter,
                                     float32 lowpass_filter_factor) {
    PID_Incremental pid;
    pid.Kp = Kp;
    pid.Ki = Ki;
    pid.Kd = Kd;
    pid.error = 0;
    pid.last_error = 0;
    pid.last_last_error = 0;
    pid.last_out = 0;
    pid.out = 0;
    pid.outmax = outmax;
    pid.outmin = outmin;
    pid.use_lowpass_filter = use_lowpass_filter;
    pid.lowpass_filter_factor = lowpass_filter_factor;
    return pid;
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     PID增量式计算
// 参数说明     pid  PID增量式
// 参数说明     input_value  输入值
// 参数说明     setpoint  设定值
// 返回参数     float32
// 备注信息     PID增量式计算,适用于电机控制,返回输出值
float32 PID_Incremental_Calc(PID_Incremental *pid, float32 input_value, float32 setpoint){
    pid->last_last_error = pid->last_error;
    pid->last_error = pid->error;
    pid->error = setpoint - input_value;
    float32 derivative = (pid->error - 2 * pid->last_error + pid->last_last_error);
    float32 output_increment = pid->Kp * (pid->error - pid->last_error) + pid->Ki * pid->error + pid->Kd * derivative;

    pid->out += output_increment;

    // 输出限幅
    if(pid->out > pid->outmax){
        pid->out = pid->outmax;
    } else if(pid->out < pid->outmin){
        pid->out = pid->outmin;
    }

    // 低通滤波
    if(pid->use_lowpass_filter){
        pid->out = pid->last_out * pid->lowpass_filter_factor + pid->out * (1 - pid->lowpass_filter_factor);
    }

    pid->last_out = pid->out;

    return pid->out;
}