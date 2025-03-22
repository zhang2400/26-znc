//
// Created by ashkore on 2023/12/2.
//

#ifndef SMART_CAR_CAMERA_FLAG_H
#define SMART_CAR_CAMERA_FLAG_H


#define LOCK 1
#define UNLOCK 0

#include "config.h"

typedef struct{
    int8_t need_sec_border;
    int8_t left_sec_border;
    int8_t right_sec_border;
    int8_t left_border;
    int8_t right_border;
    int8_t drive_in_crossroad;
    int8_t drive_in_garage; // 驶入车库标志
    int8_t drive_out_garage;  // 驶出车库标志
    int8_t change_speed_setpoint;  // 允许改变速度设定点标志
    int8_t already_ramp;  // 已经上坡标志
    int8_t ramp_up;  // 上坡标志
    int8_t ramp_down;  // 下坡标志
    int8_t found_crossroad;  // 发现十字路口标志
    int8_t found_left_roundabout;  // 发现左环岛标志
    int8_t found_right_roundabout;  // 发现右环岛标志
    int8_t found_left_corner;  // 发现左拐角标志
    int8_t found_right_corner;  // 发现右拐角标志
    int8_t found_ramp;  // 发现坡道标志
    int8_t found_garage;  // 发现车库标志
    int8_t found_obstacle;  // 发现障碍物标志
    int8_t left_roundabout_type;  // 左环岛类型
    int8_t right_roundabout_type;  // 右环岛类型
    int8_t left_B_higher_than_C;  // 左拐角B点高于C点标志，环岛需要使用这个更严格的条件
    int8_t right_B_higher_than_C;  // 右拐角B点高于C点标志，环岛需要使用这个更严格的条件
    int8_t start;
    int8_t stop;  // 强制停车标志
    int8_t icm20602_error;  // 陀螺仪错误标志
    int8_t lost_control;  // 失控标志
    int8_t image_preprocess_method;  // 图像预处理方法
    int8_t crossroad_By_diff;  // 十字路口B点差值
    int8_t boost;  // 加速标志
    int8_t boost_drive;  // 加速驱动标志
    int8_t break_drive;  // 刹车驱动标志
    int8_t image_var_lock;  // 图像变量锁
    int8_t force_angle;  // 强制角度
    int8_t advance_avoid_obstacle;  // 提前避障标志
    int8_t advance_avoid_obstacle_dir;  // 提前避障方向
} Flag;

extern Flag flag;

#endif //SMART_CAR_CAMERA_FLAG_H
