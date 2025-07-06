//
// Created by ashkore on 25-2-1.
//

#ifndef MAIN_H
#define MAIN_H

#include <thread>
#include <atomic>
#include <chrono>
#include <vector>
#include <iostream>
#include <sys/timerfd.h>
#include <unistd.h>
#include <sys/epoll.h>
#include <pthread.h>
#include <opencv2/opencv.hpp>
#include <csignal>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include "mytag.h"
#include "log.h"
#include "vofa.h"
#include "config.h"
#include "pwm.h"
#include "beep.h"
#include "PID.h"
#include "moto.h"
#include "switch.h"
#include "servo.h"
#include "my_cv2.h"
#include "image_process.h"
#include "icm20948.h"
#include "tft180.h"
#include "UI.h"
#include <bldc.h>
#include <zf_driver_pwm.h>

void element_count(void);
void element_process(void);
void element_check(void);
void image_diff_process(void);

// 图像和传感器数据结构
struct FrameData {
    cv::Mat frame;
    int sensor_value{}; // 模拟传感器数据
};

// 无锁环形缓冲区
class RingBuffer {
public:
    explicit RingBuffer(int size) : buffer(size), head(0), tail(0) {}

    bool push(const FrameData& data) {
        int next_head = (head + 1) % buffer.size();
        if (next_head == tail) return false; // 缓冲区满
        buffer[head] = data;
        head = next_head;
        return true;
    }

    bool pop(FrameData& data) {
        if (tail == head) return false; // 缓冲区空
        data = buffer[tail];
        tail = (tail + 1) % buffer.size();
        return true;
    }

private:
    std::vector<FrameData> buffer;
    std::atomic<int> head;
    std::atomic<int> tail;
};

extern float left_wheel_pidout;
extern float right_wheel_pidout;
extern float speed_base;
extern float boost_ratio;
extern float speed_setpoint;
extern float left_speed_setpoint;
extern float right_speed_setpoint;
extern float Kp_max;
extern float Kd_max;
extern float CAR_ANGLE_CONVERT;
extern int lost_x1;
extern int lost_x2;
extern int lost_y1;
extern int lost_y2;
extern int left_lost_count;
extern int right_lost_count;
extern int left_lost_dir;
extern int right_lost_dir;
extern int blind_line;
extern int running_start_time;
extern int id;
extern int incision;
extern int incision_max;
extern int detect_count_max;
extern double distance;

extern PID_Incremental left_wheel_speed_pid;
extern PID_Incremental right_wheel_speed_pid;
extern struct pwm_info servo_pwm_info;
extern Moto Moto_L;
extern Moto Moto_R;
extern icm20948_data_t icm20948_data;
#endif //MAIN_H