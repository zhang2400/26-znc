//
// Created by EiveLL on 25-1-23.
//

#include "pit_sw.h"

timespec get_current_time();
uint64_t time_diff_ns(const timespec& start, const timespec& end);

using TimerCallback = void(*)();

void pit_callback(void) {
    icm20602_read_all(fd, &icm20602, 0.01);

    // if (L_Encoder != nullptr) {
    //     double rps = L_Encoder->pulse_counter_update();
    // printf("Current RPS: %.2f\n", rps);
    // }

    printf("AngleX: %f, AngleY: %f, AngleZ: %f\n", icm20602.KalmanAngleX, icm20602.KalmanAngleY, icm20602.AngleZ);
}

void timer_interrupt_handler() {
    static timespec last_ts = get_current_time();
    timespec current_ts = get_current_time();
    uint64_t diff_ns = time_diff_ns(last_ts, current_ts);
    last_ts = current_ts;
    double diff_ms = diff_ns / 1e6;
    // std::cout << "[Control] Timer triggered, time interval: " << diff_ms << " ms" << std::endl;

    pit_callback();
}

int pit_init_ms(uint32_t time_ms, TimerCallback cb) {
    return pit_init(time_ms * 1000, cb);
}

int pit_init(uint32_t time_us, TimerCallback cb) {
    int timer_fd = timerfd_create(CLOCK_MONOTONIC, 0);
    if (timer_fd == -1) {
        std::cerr << "timerfd_create 失败: " << strerror(errno) << std::endl;
        return -1;
    }

    struct itimerspec timer_spec = {};
    timer_spec.it_value.tv_sec = time_us / 1000000;
    timer_spec.it_value.tv_nsec = (time_us % 1000000) * 1000;
    timer_spec.it_interval = timer_spec.it_value;

    if (timerfd_settime(timer_fd, TFD_TIMER_ABSTIME, &timer_spec, nullptr) == -1) {
        std::cerr << "timerfd_settime 失败: " << strerror(errno) << std::endl;
        close(timer_fd);
        return -1;
    }

    // 绑定到高性能核心
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);

    // 提高进程优先级
    struct sched_param param;
    param.sched_priority = 99;
    sched_setscheduler(0, SCHED_FIFO, &param);

    // 事件循环
    while (true) {
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &timer_spec.it_value, nullptr);

        uint64_t expirations;
        if (read(timer_fd, &expirations, sizeof(expirations)) != sizeof(expirations)) {
            std::cerr << "read 定时器失败: " << strerror(errno) << std::endl;
            continue;
        }

        cb();  // 调用回调
    }

    close(timer_fd);
    return timer_fd;
}

// 时间相关辅助函数
timespec get_current_time() {
    timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts;
}

uint64_t time_diff_ns(const timespec& start, const timespec& end) {
    return (end.tv_sec - start.tv_sec) * 1000000000ULL + (end.tv_nsec - start.tv_nsec);
}
