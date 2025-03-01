//
// Created by ashkore on 2024/2/7.
//
#include "flag.h"

//标志位结构体初始化
Flag flag = {
        .drive_in_garage = false,
        .drive_out_garage = true,
        .change_speed_setpoint = true,
        .already_ramp = false,
        .ramp_up = false,
        .ramp_down = false,
        .found_crossroad = false,
        .found_left_roundabout = false,
        .found_right_roundabout = false,
        .stop = false,
        .icm20602_error = false,
        .lost_control = false
};
//标志位结构体初始化