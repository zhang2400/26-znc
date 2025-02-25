#include "main.h"

volatile sig_atomic_t g_signal_received = 0;
const int timer_period = 10;  // 定时器周期(ms)
std::atomic<bool> running{true};      // 控制线程运行标志
cv::Mat frame;
uint8_t image[120][160];
void* realtime_task(void* arg) {
    // 设置实时线程优先级
    auto vofa = VOFA("192.168.5.47", 1349);
    struct sched_param param = {.sched_priority = 99};
    pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(0, &cpuset);
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    // 创建高精度定时器
    int timer_fd = timerfd_create(CLOCK_MONOTONIC, 0);
    struct itimerspec timer_spec = {
        .it_interval = {0, timer_period * 1000 * 1000}, // 10ms周期
        .it_value = {0, timer_period * 1000 * 1000}
    };
    timerfd_settime(timer_fd, 0, &timer_spec, nullptr);

    // 创建epoll实例
    int epoll_fd = epoll_create1(0);
    struct epoll_event event = {.events = EPOLLIN, .data = {.fd = timer_fd}};
    epoll_ctl(epoll_fd, EPOLL_CTL_ADD, timer_fd, &event);

    std::chrono::steady_clock::time_point start;
    std::chrono::steady_clock::time_point end;

    cv::VideoCapture cap;
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 160);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 120);
    cap.set(cv::CAP_PROP_FPS, 120);
    cap.open(0);

    while (running) {
        // 等待定时器事件
        struct epoll_event events[1];
        int num_events = epoll_wait(epoll_fd, events, 1, -1);
        if (num_events > 0 && events[0].data.fd == timer_fd) {
            uint64_t expirations;
            read(timer_fd, &expirations, sizeof(expirations));

            cap >> frame;


            end = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
            start = std::chrono::steady_clock::now();
            fprintf(stdout,"timer_event 距离上次事件%.2lfms\n", time_used.count() * 1000);
            LOGI("timer_event", "距离上次事件%.2lfms", time_used.count() * 1000);
            if(std::abs(time_used.count() * 1000 - timer_period) > 1) {
                fprintf(stdout,"timer_event 定时器周期不准确，误差: %.2lfms\n", time_used.count() * 1000 - timer_period);
                LOGW("timer_event", "定时器周期不准确，误差: %.2lfms", time_used.count() * 1000 - timer_period);
            }
            memcpy(image, frame.data, 120 * 160);
            uint8_t pixel_value = image[60][80];
            vofa.printf("aaa:%d\n",pixel_value);
        }
    }

    close(timer_fd);
    close(epoll_fd);
    return nullptr;
}

// 非实时任务线程函数
void *non_realtime_task(void *arg) {
//    cv::VideoWriter http;
//    http.open("httpjpg", 7766);
    auto vofa = VOFA("192.168.5.47", 1347);
    cv::VideoWriter http;
    http.open("httpjpg",7766);
    auto atag = mytag("tag36h11", 0.5, 0, 1, false, false);
    int id;
    cv::Mat my_frame;
    cv::Mat gray;
    double distance;
    std::vector<uchar> jpg;
    while (running) {
            frame.copyTo(my_frame);
//            MEASURE_TIME("convert gray", {
                cvtColor(my_frame, gray, cv::COLOR_BGR2GRAY);
//            });
//            MEASURE_TIME("detect_time", {
                atag.detect(gray);
//            });
//            MEASURE_TIME("getclosettagindex", {
                atag.getClosetTagIndex();
//            });
//            MEASURE_TIME("draw", {
                atag.draw(my_frame);
//            });
//            MEASURE_TIME("getid", {
                id = atag.getClosetTagID();
//            });
//            MEASURE_TIME("getdistance", {
                distance = atag.getClosetTagDistance(1500);
                cv::putText(my_frame, std::to_string(distance), cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0xff, 0), 2);
//            });
//            MEASURE_TIME("http write", {
                vofa.imwrite(my_frame);
                // http << my_frame;
//            });
    }
    return nullptr;
}

void signal_handler(int sig) {
    g_signal_received = sig;
    running = false;
    log_shutdown();
}

int main()
{
    // 注册信号处理
    struct sigaction sa;
    sa.sa_handler = signal_handler;
    sa.sa_flags = 0;
    sigemptyset(&sa.sa_mask);

    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    if(!log_init("app.logcat")) {
        return 1;
    }
    LOGW("MAIN", "Application starting...");


    PWM pwm0(PWM0_GPIO64);
    pwm0.set_frequency(1000);
    pwm0.set_duty(4200);
    pwm0.enable();

    PWM pwm1(1);
    pwm1.set_frequency(3000);
    pwm1.set_duty(1500);
    pwm1.enable();

    PWM pwm2(2);
    pwm2.set_frequency(30000);
    pwm2.set_duty(5000);
    pwm2.enable();

    PWM pwm3(3);
    pwm3.set_frequency(4000);
    pwm3.set_duty(8000);
    pwm3.enable();

    // 创建posix线程
    pthread_t rt_thread;
    pthread_t nrt_thread;
    pthread_create(&rt_thread, nullptr, realtime_task, nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 运行1秒
    pthread_create(&nrt_thread, nullptr, non_realtime_task, nullptr);

    std::this_thread::sleep_for(std::chrono::seconds(1000)); // 运行100秒

    running = false; // 停止线程

    log_shutdown();
}