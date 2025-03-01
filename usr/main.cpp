#include "main.h"

#include <switch.h>
// PID相关变量
PID_Incremental left_wheel_speed_pid;
PID_Incremental right_wheel_speed_pid;

PID_Position wheel_turn_pid;

// 电机相关变量
float left_wheel_pidout = 0;
float right_wheel_pidout = 0;

float32 left_speed_setpoint = 60;
float32 right_speed_setpoint = 60;

// 信号处理变量
volatile sig_atomic_t g_signal_received = 0;

// 实时线程相关变量
const int timer_period = 10;            // 定时器周期(ms)
std::atomic<bool> running{true};      // 控制线程运行标志

// 图像处理相关变量
cv::Mat frame;
uint8_t image[120][160];
const char* i2c_dev = "/dev/i2c-0";
int fd = open(i2c_dev, O_RDWR);

// 传输层相关变量
auto tcp_transport = std::make_unique<TCPTransport>("0.0.0.0", 1347);
auto vofa_tcp = VOFA(std::move(tcp_transport));
auto udp_transport = std::make_unique<UDPTransport>("192.168.5.16", 1349);
auto vofa_udp = VOFA(std::move(udp_transport));

int i = SERVO_MOTOR_MID;
int j = 0;
int flag = 1;

void* realtime_task(void* arg) {
    wheel_turn_pid = PID_Position_Init(0.015, 0, 0, 0.24, 0, 50000, -50000, false, 0.2f);

    left_wheel_speed_pid = PID_Incremental_Init(45, 6, 4, 7000, -7000, false, 0.25f);
    right_wheel_speed_pid = PID_Incremental_Init(45, 6, 4, 7000, -7000, false, 0.25f);

    switch_init();

    BEEP beep(GPIO61);
    Moto Moto_L(PWM1_GPIO65, 75, PWM0_GPIO64, 73, false);
    Moto Moto_R(PWM2_GPIO66, 74, PWM3_GPIO67, 72, true);

    Servo Servo(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, SERVO_MOTOR_L_MAX, SERVO_MOTOR_R_MAX, SERVO_MOTOR_MID);

    icm20602_init(fd);

    // 设置实时线程优先级
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
            // uint8_t pixel_value = image[60][80];
            vofa_udp.printf("L_pid:%f,%f,%d,%d\n",left_speed_setpoint, right_speed_setpoint, Moto_L.speed, Moto_R.speed);

            Moto_L.update_speed();
            Moto_R.update_speed();

            left_wheel_pidout  = PID_Incremental_Calc(&left_wheel_speed_pid, (float) Moto_L.speed, left_speed_setpoint);
            right_wheel_pidout = PID_Incremental_Calc(&right_wheel_speed_pid, (float) Moto_R.speed, right_speed_setpoint);

            Moto_L.set_speed((int)left_wheel_pidout);
            Moto_R.set_speed((int)right_wheel_pidout);

            if (switch1()) {
                beep.beep_on();
            }
            else {
                beep.beep_off();
            }
            // icm20602_read_all(fd, &icm20602, 0.01);
            // printf("AngleX: %f, AngleY: %f, AngleZ: %f\n", icm20602.KalmanAngleX, icm20602.KalmanAngleY, icm20602.AngleZ);
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
                vofa_tcp.imwrite(my_frame);
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