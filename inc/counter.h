//
// Created by ashkore on 2023/11/2.
//

#ifndef SMART_CAR_CAMERA_COUNTER_H
#define SMART_CAR_CAMERA_COUNTER_H

#include "config.h"

typedef struct{
    int erase;  // 擦除的白色像素点数量
    int16_t found_left_roundabout;  // 发现左环岛计数
    int16_t found_right_roundabout;  // 发现右环岛计数
    int16_t found_crossroad;  // 发现十字路口计数
    int16_t found_ramp;  // 发现坡道计数
    int16_t found_garage;  // 发现车库计数
    int16_t found_obstacle;  // 发现障碍物计数
    int16_t drive_in_left_roundabout;  // 正在驶入左环岛倒计时(ms)
    int16_t drive_in_right_roundabout;  // 正在驶入右环岛倒计时(ms)
    int16_t drive_in_crossroad;  // 正在驶入十字路口倒计时(ms)
    int16_t drive_in_ramp;  // 正在驶入坡道倒计时(ms)
    int16_t drive_in_obstacle;  // 正在驶入障碍物倒计时(ms)
    int16_t avoid_roundabout;  // 不进行环岛检测倒计时(ms)
    int16_t avoid_obstacle;  // 不进行障碍物检测倒计时(ms)
    int16_t serial_send;  // 串口发送计数处理
    int16_t stop_motor;  // 停止电机计数
    int16_t out_of_bound;  // 出界计数
    uint16_t beep_ms;  // 蜂鸣器计数
    int16_t boost;  // 加速计数
    int32_t stop_delay;  // 停车计数
    int16_t start_motor_delay;  // 启动电机计数
    int32_t save_flash_led;  // 保存闪存LED计数
    int32_t read_flash_led;  // 读取闪存LED计数
} Counter;

extern Counter counter;

#endif //SMART_CAR_CAMERA_COUNTER_H
