#include "main.h"

float CAR_ANGLE_CONVERT = 3.1f;

// PID相关变量
PID_Incremental left_wheel_speed_pid;
PID_Incremental right_wheel_speed_pid;

PID_Position wheel_turn_pid;

// 电机相关变量
float left_wheel_pidout = 0;
float right_wheel_pidout = 0;

float speed_setpoint = 0;
float left_speed_setpoint = 0;
float right_speed_setpoint = 0;

float turn_pidout = 0;
float turn_angle = 0;
float turn_max = 15;

// 信号处理变量
volatile sig_atomic_t g_signal_received = 0;

// 实时线程相关变量
const int timer_period = 10;            // 定时器周期(ms)
std::atomic<bool> running{true};      // 控制线程运行标志

// 图像处理相关变量
cv::Mat frame;
cv::Mat myframe;
cv::Mat result_image;
uint8_t gray1ch_image[60][80];

int cornering;
int image_diff;
int force_roundabout;

int left_sum;
int right_sum;
int start;
int end;

int dis_index = 0;

// 陀螺仪相关变量
const char* i2c_dev = "/dev/i2c-0";
int fd = open(i2c_dev, O_RDWR);

// 传输层相关变量
auto tcp_transport = std::make_unique<TCPTransport>("0.0.0.0", 1347);
auto vofa_tcp = VOFA(std::move(tcp_transport));
auto udp_transport = std::make_unique<UDPTransport>("192.168.5.16", 1349);
auto vofa_udp = VOFA(std::move(udp_transport));

int i = SERVO_MOTOR_MID;
int j = 0;

void* realtime_task(void* arg) {
    wheel_turn_pid = PID_Position_Init(0.028, 0, 0, 0.12, 0, 50000, -50000, true, 0.2f);

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

    int otsu_threshold = 0;
    int contrast_threshold = 20;
    int canny_lowThreshold = 16;
    int canny_highThreshold = 40;
    int incision = 6;
    int incision_max = 6;

    cv::VideoCapture cap;
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(cv::CAP_PROP_FPS, 120);
    cap.open(0);
    result_image = cv::Mat(60, 80, CV_8UC1);

    while (running) {
        // 等待定时器事件
        struct epoll_event events[1];
        int num_events = epoll_wait(epoll_fd, events, 1, -1);
        if (num_events > 0 && events[0].data.fd == timer_fd) {
            uint64_t expirations;
            read(timer_fd, &expirations, sizeof(expirations));

            // 定时器周期检测
            end = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
            start = std::chrono::steady_clock::now();
            fprintf(stdout,"timer_event 距离上次事件%.2lfms\n", time_used.count() * 1000);
            LOGI("timer_event", "距离上次事件%.2lfms", time_used.count() * 1000);
            if(std::abs(time_used.count() * 1000 - timer_period) > 1) {
            fprintf(stdout,"timer_event 定时器周期不准确，误差: %.2lfms\n", time_used.count() * 1000 - timer_period);
                LOGW("timer_event", "定时器周期不准确，误差: %.2lfms", time_used.count() * 1000 - timer_period);
            }

            // 图像处理
            // MEASURE_TIME("rt task", {
            cap >> frame;
            // vofa_tcp.imwrite(frame);
            frame.copyTo(myframe);
            cv::resize(myframe, myframe, cv::Size(80,60));
            memcpy(gray_image, myframe.data, 80 * 60);
            // cover_car_head();
            calculate_contrast_x8((uint8_t *)contrast_image, (const uint8_t *)gray_image, 80, 60);
            memcpy((uint8_t *) binary_image, (const uint8_t *) contrast_image, 80 * 60);
            my_cv2_doubleThreshold((uint8_t *) binary_image, 80, 0, 0, 80, 60, canny_lowThreshold, canny_highThreshold);
            my_cv2_checkConnectivity((uint8_t *) binary_image, 80, 0, 0, 80, 60);
            my_cv2_threshold((uint8_t *) binary_image, 80, 0, 0, 80, 60, 127, 255);
            memcpy((uint8_t *) binary_image_bak, (const uint8_t *) binary_image, 80 * 60);
            memcpy((uint8_t *) gray_binary_image, (const uint8_t *) gray_image, 80 * 60);
            otsu_threshold = get_otsu_threshold(40, 30, 80, 60, (const uint8 *) gray_image);
            my_cv2_threshold((uint8 *) gray_binary_image, 80, 0, 0, 80, 60, otsu_threshold, 255);

            // vofa_tcp.imwrite((uint8_t *)gray_binary_image, 80, 60);

            bottom_start_end_x_get();
            // get_max_middle_line_height();

            incision = incision_max;
            max_white_column_get(bottom_start_x > 10 ? bottom_start_x : 10, 1, bottom_end_x < 64 ? bottom_end_x : 64 , 59);

            get_distance_line();
            get_lost_count();
            // check_garage_and_obstacle();
            // check_ramp();
            // check_crossroad();
            // check_roundabout();
            // get_narrow_line();
            draw_rectan();
            // element_process();

            int detect_count_max = get_border_line(100);
            // });
            // vofa_tcp.imwrite((uint8_t *)contrast_image, 80, 60);

            // vofa_udp.printf("L_pid:%f,%f,%d,%d\n",left_speed_setpoint, right_speed_setpoint, Moto_L.speed, Moto_R.speed);

            Moto_L.update_speed();
            Moto_R.update_speed();

            // 位置环PID(需要优化)
            turn_pidout = PID_Position_Calc(&wheel_turn_pid, 0, (float) icm20602.Gz, (float) image_diff);
            turn_angle = turn_pidout / 10;

            if(turn_angle > turn_max) turn_angle = turn_max;
            if(turn_angle < -turn_max) turn_angle = -turn_max;

            float turn_angle_real = turn_angle * CAR_ANGLE_CONVERT * (3.14159265358979323846 / 180);

            if(turn_angle_real == 0) {
                left_speed_setpoint = speed_setpoint;
                right_speed_setpoint = speed_setpoint;
            }
            else {
                left_speed_setpoint = fabs(speed_setpoint * (((CAR_WHEELBASE_L / tan(turn_angle_real)) + (CAR_WHEELBASE_B / 2)) / sqrt(pow(CAR_WHEELBASE_L / 2, 2) + pow(CAR_WHEELBASE_L / tan(turn_angle_real), 2))));
                right_speed_setpoint = fabs(speed_setpoint * (((CAR_WHEELBASE_L / tan(turn_angle_real)) - (CAR_WHEELBASE_B / 2)) / sqrt(pow(CAR_WHEELBASE_L / 2, 2) + pow(CAR_WHEELBASE_L / tan(turn_angle_real), 2))));

                double diff = (left_speed_setpoint > speed_setpoint) ? (left_speed_setpoint - speed_setpoint) : (right_speed_setpoint - speed_setpoint);
                left_speed_setpoint -= diff;
                right_speed_setpoint -= diff;
            }

            Servo.set_angle(SERVO_MOTOR_MID - turn_angle);

            // vofa_udp.printf("%d,%d\n",left_lost_count,right_lost_count);
            // vofa_udp.printf("%d,%d,%d,%d\n",max_white_column.left_x,max_white_column.right_x,max_white_column.start_y,max_white_column.end_y);
            vofa_udp.printf("%d,%d,%d,%d\n",dis_index,distances[dis_index],left_distance[dis_index][0],right_distance[dis_index][0]);
            // vofa_udp.printf("%d,%d,%f,%d,%d,%d\n",image_diff, right_sum, turn_pidout,left_distance[0][0],left_distance[5][0],left_distance[10][0]);

            // 速度环PID(需要优化)
            if(flag.stop){
                speed_setpoint = 0;
                left_speed_setpoint = 0;
                right_speed_setpoint = 0;
            }
            left_wheel_pidout  = PID_Incremental_Calc(&left_wheel_speed_pid, (float32) Moto_L.speed, left_speed_setpoint);
            right_wheel_pidout = PID_Incremental_Calc(&right_wheel_speed_pid, (float32) Moto_R.speed, right_speed_setpoint);

            Moto_L.set_speed((int)left_wheel_pidout);
            Moto_R.set_speed((int)right_wheel_pidout);

            left_sum = 0;
            right_sum = 0;
            int img_start = (0.1 * MAX(Moto_L.speed, Moto_R.speed) - 20);
            if(img_start < incision)img_start = incision;
            int img_end = img_start + 40;
            if(img_end > detect_count_max) img_end = detect_count_max;
            for(int i = img_start; i < img_end; i++) {
                left_sum -= (middle_line[i][0] - IMAGE_MIDDLE) * (1.4 + (i - (img_end - img_start) / 2) * 0.06);
                // left_sum -= middle_line[i][0] - IMAGE_MIDDLE;
            }
            left_sum *= 4;
            left_sum = left_sum < 0 ? left_sum : left_sum *= 3;
            left_sum += 4000 * cornering;
            image_diff = right_sum - left_sum;

            if (switch1()) {
                dis_index++;
            }else if (switch2()) {
                dis_index--;
            }

            // icm20602_read_all(fd, &icm20602, 0.01);
            // printf("AngleX: %f, AngleY: %f, AngleZ: %f\n", icm20602.KalmanAngleX, icm20602.KalmanAngleY, icm20602.AngleZ);

            if(counter.beep_ms > 0) {
                counter.beep_ms -= 10;
                beep.beep_on();
            }
            else {
                beep.beep_off();
            }
        }
    }

    close(timer_fd);
    close(epoll_fd);
    return nullptr;
}

void element_process(void) {

}

// 非实时任务线程函数
void *non_realtime_task(void *arg) {
    auto atag = mytag("tag36h11", 1.5, 0, 1, false, false);
    int id;
    cv::Mat gray;
    cv::Mat gray1ch(60, 80, CV_8UC1, (void*)gray1ch_image);
    cv::Mat gray3ch;
    double distance;
    std::vector<uchar> jpg;
    while (running) {
            frame.copyTo(gray);
            // MEASURE_TIME("non rt task", {
                memcpy(gray1ch_image, gray_binary_image, 80 * 60);
                cv::cvtColor(gray1ch, gray3ch, cv::COLOR_GRAY2BGR);
            // });
            MEASURE_TIME("detect_time", {
                atag.detect(gray);
            });
            // MEASURE_TIME("getclosettagindex", {
                atag.getClosetTagIndex();
//            });
//            MEASURE_TIME("draw", {
                atag.draw(gray3ch, 0.25);
//            });detect_count_max:
//            MEASURE_TIME("getid", {
                id = atag.getClosetTagID();
//            });
//            MEASURE_TIME("getdistance", {
                distance = atag.getClosetTagDistance(1500);
                // cv::putText(gray3ch, std::to_string(distance), cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0xff, 0), 2);
//            });
                tft180_draw_border_line(gray3ch, 0,0,left_border, cv::Scalar(0, 0xff, 0));
                tft180_draw_border_line(gray3ch, 0, 0, right_border, cv::Scalar(0, 0xff, 0));
                tft180_draw_border_line(gray3ch, 0, 0, middle_line, cv::Scalar(0, 0, 0xff));
            // MEASURE_TIME("http write", {
                vofa_tcp.imwrite(gray3ch);
                // http << gray3ch;
            // });
    }
    return nullptr;
}

void signal_handler(int sig) {
    running = false;
    g_signal_received = sig;
    log_shutdown();
}

int main()
{
    // 注册信号处理
    struct sigaction sa{};
    sa.sa_handler = signal_handler;
    sa.sa_flags = 0;
    sigemptyset(&sa.sa_mask);

    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
    if(!log_init("app.logcat")) {
        return 1;
    }
    LOGW("MAIN", "Application starting...");

    system("v4l2-ctl -d /dev/video0 -c contrast=100 -c gamma=500 -c auto_exposure=1 -c exposure_time_absolute=100");

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