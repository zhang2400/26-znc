#include "main.h"

#include <termios.h>

float CAR_ANGLE_CONVERT = 3.0f;

// PID相关变量
PID_Incremental left_wheel_speed_pid;
PID_Incremental right_wheel_speed_pid;

PID_Position wheel_turn_pid;

int speedtest = 1000;
// 电机相关变量
float left_wheel_pidout = 0;
float right_wheel_pidout = 0;

float speed_base = 190.0f; //基础速度
float boost_ratio = 0.0f; // 直道加速比率
float speed_setpoint = speed_base;
float left_speed_setpoint = 0;
float right_speed_setpoint = 0;

float turn_pidout = 0;
float turn_angle = 0;
float turn_max = 25;

float Kp_max = 0.028f;
float Kd_max = 0.28f;
float rate = 1;

// 信号处理变量
volatile sig_atomic_t g_signal_received = 0;

// 实时线程相关变量
constexpr int timer_period = 10;            // 定时器周期(ms)
std::atomic<bool> running{true};      // 控制线程运行标志

// 图像处理相关变量
cv::Mat frame;
cv::Mat myframe;
cv::Mat result_image;
uint8_t gray1ch_image[60][80];

int cornering;
int image_diff;
int last_diff;

int left_sum;
int right_sum;
int start;
int end;

int dis_index = 0;
int rstate = 0;
int8_t cnt = 0;
float angelZ = 0;
int cross_roundabout = 0;
float ramp_angle = 0;

// 陀螺仪相关变量
icm20948_data_t icm20948_data;
icm20948_handle_t icm20948 = icm20948_create(&icm20948_data, "icm20948");
int ret1,ret2;

// 传输层相关变量
auto tcp_transport = std::make_unique<TCPTransport>("0.0.0.0", 1347);
auto vofa_tcp = VOFA(std::move(tcp_transport));
auto udp_transport = std::make_unique<UDPTransport>("10.210.250.121", 1349);
auto vofa_udp = VOFA(std::move(udp_transport));
// 图传专用UDP通道（与vofa_udp分开，避免RT线程printf数据和图像数据互相交错）
auto udp_img_transport = std::make_unique<UDPTransport>("10.210.250.121", 1348);
auto vofa_udp_img = VOFA(std::move(udp_img_transport));

int incision = 1;
int incision_max = 1;
int detect_count_max = 0;

int blind_line;
int protect = 0;

// int running_time = 16500;
int running_time = 500000;

int running_start_time = 0;
int delay_time = running_time;
int stop_in_garage = true;

BEEP beep(GPIO61);
Moto Moto_L(PWM1_GPIO65, 72, PWM0_GPIO64, 51, false);
Moto Moto_R(PWM2_GPIO66, 73,PWM3_GPIO67 , 50, true);

// 获取PWM驱动信息
struct pwm_info servo_pwm_info;

#define max_white_column_height 53
#define min_white_column_height 40

// tag码相关变量
int id = -1;
double distance = -1;

// ===================== 识别板通信与目标融合 =====================

constexpr int TARGET_TRIGGER_Y_TH = 120;     // y 大于这个值才开始执行动作，按实测调
constexpr int TARGET_X_CENTER_MIN = 60;      // 目标横向有效范围，按识别图像宽度调
constexpr int TARGET_X_CENTER_MAX = 300;


enum {
    TARGET_UNKNOWN = -1,
    TARGET_WEAPON = 0,
    TARGET_MATERIAL = 1,
    TARGET_VEHICLE = 2
};

constexpr int TARGET_LISTEN_PORT = 2233;
constexpr int TARGET_PACKET_SIZE = 12;
constexpr int TARGET_CONF_TH = 60;           //控制识别类别可信度
constexpr int TARGET_TIMEOUT_MS = 200;       //控制通信结果多久过期

// 至少连续看到红块这么久，丢失红块时才允许补偿触发。
constexpr int TARGET_TRACK_MIN_MS = 200;

// 红块最后一次有效坐标距离现在不能超过这么久。
constexpr int TARGET_LOST_TRIGGER_MAX_MS = 400;

// 使用最近5个有效id投票。
constexpr int TARGET_ID_HISTORY_SIZE = 5;
constexpr int TARGET_ID_MIN_VOTE_COUNT = 2;

struct RemoteTarget {
    std::atomic<int> class_id{TARGET_UNKNOWN};
    std::atomic<int> confidence{0};
    std::atomic<int> target_x{-1};
    std::atomic<int> target_y{-1};
    std::atomic<uint64_t> last_recv_ms{0};
    std::atomic<uint8_t> seq{0};

    std::atomic<uint32_t> packet_counter{0};
};

RemoteTarget remote_target;

// 识别目标专用绕行状态，和原避障块 counter.drive_in_obstacle 分开
int target_avoid_ms = 0;
int target_avoid_dir = 0;
uint8_t last_target_action_seq = 255;

constexpr int TARGET_AVOID_TIME_MS = 900;       //识别目标绕行总时间
constexpr int TARGET_AVOID_BIAS = 9500;         //识别目标绕行偏置
constexpr int TARGET_AVOID_RETURN_MS = 100;     //最后多少 ms 做回正250
constexpr int TARGET_AVOID_RETURN_BIAS = 3000;  //回正偏置3000
constexpr uint16 BLDC_SAFE_STOP_DUTY = static_cast<uint16>(BLDC_DUTY_MIN);

bool has_last_target_action = false;
uint8_t tracked_target_seq = 0;
bool tracked_target_seq_valid = false;
// 最近处理到的通信包编号，防止10ms主循环重复统计同一通信包。
uint32_t last_processed_target_packet = 0;

// 红块坐标跟踪状态。
bool target_position_tracking = false;
uint64_t target_track_start_ms = 0;
uint64_t target_last_position_ms = 0;

// 最近5次有效id。
int target_id_history[TARGET_ID_HISTORY_SIZE] = {
    TARGET_UNKNOWN,
    TARGET_UNKNOWN,
    TARGET_UNKNOWN,
    TARGET_UNKNOWN,
    TARGET_UNKNOWN
};

int target_id_history_count = 0;
uint64_t get_ms()
{
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<milliseconds>(steady_clock::now().time_since_epoch()).count());
}

void emergency_stop_outputs()
{
    running = false;
    flag.stop = true;
    speed_base = 0;
    speed_setpoint = 0;
    left_speed_setpoint = 0;
    right_speed_setpoint = 0;
    left_wheel_pidout = 0;
    right_wheel_pidout = 0;

    Moto_L.set_speed(0);
    Moto_R.set_speed(0);
    pwm_set_duty("/dev/zf_device_pwm_esc_1", static_cast<uint16>(500));
    pwm_set_duty("/dev/zf_device_pwm_servo", static_cast<uint16>(500));
}

void terminal_stop_thread()
{
    if (!isatty(STDIN_FILENO)) {
        char ch = 0;
        while (running && std::cin.get(ch)) {
            if (ch == 'q' || ch == 'Q') {
                std::cout << "q pressed, emergency stop." << std::endl;
                emergency_stop_outputs();
                break;
            } else if (ch == 's' || ch == 'S') {
                flag.start = true;
                counter.beep_ms = 300;
                std::cout << "s pressed, car started." << std::endl;
            }
            // else if (ch == 'w' || ch == 'W'){
            //
            //     speedtest += 100 ;
            //     std::cout << "w pressed, add10" << std::endl;
            // }else if (ch == 'e' || ch == 'E'){
            //     speedtest -= 100;
            //     std::cout << "e pressed, pooe10" << std::endl;
            // }
        }
        return;
    }

    termios old_term {};
    if (tcgetattr(STDIN_FILENO, &old_term) != 0) {
        return;
    }

    termios raw_term = old_term;
    raw_term.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
    raw_term.c_cc[VMIN] = 0;
    raw_term.c_cc[VTIME] = 1;

    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw_term) != 0) {
        return;
    }

    while (running) {
        char ch = 0;
        ssize_t n = read(STDIN_FILENO, &ch, 1);
        if (n == 1 && (ch == 'q' || ch == 'Q')) {
            std::cout << "q pressed, emergency stop." << std::endl;
            emergency_stop_outputs();
            break;
        } else if (n == 1 && (ch == 's' || ch == 'S')) {
            flag.start = true;
            counter.beep_ms = 300;
            std::cout << "s pressed, car started." << std::endl;
        }
        // else if (n == 1 && (ch == 'w' || ch == 'W'))
        // {
        //     speedtest += 100;
        //     std::cout << "w pressed, add100" << std::endl;
        //
        // }else if (n == 1 && (ch == 'e' || ch == 'E'))
        // {
        //     speedtest -= 100;
        //     std::cout << "e pressed, poor100" << std::endl;
        //
        // }
        if (n < 0 && errno != EINTR && errno != EAGAIN) {
            break;
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &old_term);
}

static uint8_t target_checksum(const uint8_t* data, size_t len)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < len; ++i) {
        sum = static_cast<uint8_t>(sum + data[i]);
    }
    return sum;
}

static int16_t read_i16_le(const uint8_t* src)
{
    uint16_t value = static_cast<uint16_t>(src[0])
        | (static_cast<uint16_t>(src[1]) << 8);
    return static_cast<int16_t>(value);
}

bool remote_target_valid()
{
    int cls = remote_target.class_id.load();
    int conf = remote_target.confidence.load();
    uint64_t last = remote_target.last_recv_ms.load();
    uint64_t now = get_ms();

    return cls >= TARGET_WEAPON
        && cls <= TARGET_VEHICLE
        && conf >= TARGET_CONF_TH
        && last > 0
        && now - last < TARGET_TIMEOUT_MS;
}

void clear_remote_target_if_timeout()
{
    uint64_t last = remote_target.last_recv_ms.load();
    uint64_t now = get_ms();

    if (last > 0 && now - last > TARGET_TIMEOUT_MS) {
        remote_target.class_id.store(TARGET_UNKNOWN);
        remote_target.confidence.store(0);
        remote_target.target_x.store(-1);
        remote_target.target_y.store(-1);
        remote_target.last_recv_ms.store(0);
    }
}
void target_recv_thread()
{
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        return;
    }

    int reuse = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

    sockaddr_in addr {};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(TARGET_LISTEN_PORT);

    if (bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0) {
        close(server_fd);
        return;
    }

    if (listen(server_fd, 1) < 0) {
        close(server_fd);
        return;
    }

    while (running) {
        sockaddr_in client_addr {};
        socklen_t client_len = sizeof(client_addr);
        int client_fd = accept(server_fd, reinterpret_cast<sockaddr*>(&client_addr), &client_len);
        if (client_fd < 0) {
            continue;
        }

        int nodelay = 1;
        setsockopt(client_fd, IPPROTO_TCP, TCP_NODELAY, &nodelay, sizeof(nodelay));

        std::vector<uint8_t> pending;
        uint8_t buffer[128];

        while (running) {
            ssize_t n = recv(client_fd, buffer, sizeof(buffer), 0);
            if (n <= 0) {
                break;
            }

            for (ssize_t i = 0; i < n; ++i) {
                pending.push_back(buffer[i]);
            }

            while (pending.size() >= TARGET_PACKET_SIZE)
            {
                if (pending[0] != 0xAA || pending[1] != 0x55) {
                    auto next = std::find(pending.begin() + 1, pending.end(), 0xAA);
                    pending.erase(pending.begin(), next);
                    continue;
                }

                if (target_checksum(pending.data(), TARGET_PACKET_SIZE - 1)
                    != pending[TARGET_PACKET_SIZE - 1]) {
                    pending.erase(pending.begin());
                    continue;
                    }

                int cls = static_cast<int8_t>(pending[3]);
                int conf = pending[4];
                int16_t x = read_i16_le(pending.data() + 5);
                int16_t y = read_i16_le(pending.data() + 7);

                // 奇数表示接收线程正在更新字段。
                remote_target.packet_counter.fetch_add(
                    1, std::memory_order_acq_rel);

                remote_target.seq.store(pending[2]);
                remote_target.class_id.store(cls);
                remote_target.confidence.store(conf);
                remote_target.target_x.store(x);
                remote_target.target_y.store(y);
                remote_target.last_recv_ms.store(get_ms());

                // 偶数表示完整数据包已经写完。
                remote_target.packet_counter.fetch_add(
                    1, std::memory_order_release);

                pending.erase(pending.begin(),pending.begin() + TARGET_PACKET_SIZE);
            }
            if (pending.size() > 128) {
                pending.clear();
            }
        }
        remote_target.class_id.store(TARGET_UNKNOWN);
        remote_target.confidence.store(0);
        remote_target.target_x.store(0);
        remote_target.target_y.store(0);
        remote_target.last_recv_ms.store(0);
        close(client_fd);
    }

    close(server_fd);
}

void reset_target_tracking()
{
    target_position_tracking = false;
    target_track_start_ms = 0;
    target_last_position_ms = 0;
    target_id_history_count = 0;

    for (int i = 0; i < TARGET_ID_HISTORY_SIZE; ++i) {
        target_id_history[i] = TARGET_UNKNOWN;
    }
}

void push_target_id(int cls)
{
    if (cls < TARGET_WEAPON || cls > TARGET_VEHICLE) {
        return;
    }

    if (target_id_history_count < TARGET_ID_HISTORY_SIZE) {
        target_id_history[target_id_history_count] = cls;
        ++target_id_history_count;
        return;
    }

    // 数组满后删除最早的id，保留最近5次。
    for (int i = 1; i < TARGET_ID_HISTORY_SIZE; ++i) {
        target_id_history[i - 1] = target_id_history[i];
    }

    target_id_history[TARGET_ID_HISTORY_SIZE - 1] = cls;
}

    int get_target_majority_id()
    {
        if (target_id_history_count < TARGET_ID_MIN_VOTE_COUNT) {
            return TARGET_UNKNOWN;
        }

        int counts[3] = {0, 0, 0};

        for (int i = 0; i < target_id_history_count; ++i) {
            int cls = target_id_history[i];

            if (cls >= TARGET_WEAPON &&
                cls <= TARGET_VEHICLE) {
                ++counts[cls];
                }
        }

        int majority_id = TARGET_WEAPON;

        if (counts[TARGET_MATERIAL] > counts[majority_id]) {
            majority_id = TARGET_MATERIAL;
        }

        if (counts[TARGET_VEHICLE] > counts[majority_id]) {
            majority_id = TARGET_VEHICLE;
        }

        // 3或4个结果可能平票，平票使用最近一次有效ID。
        int max_count = counts[majority_id];
        int same_max_count = 0;

        for (int cls = TARGET_WEAPON;
             cls <= TARGET_VEHICLE;
             ++cls) {
            if (counts[cls] == max_count) {
                ++same_max_count;
            }
             }

        if (same_max_count > 1) {
            return target_id_history[target_id_history_count - 1];
        }

        return majority_id;
    }

void execute_target_action(int cls, uint8_t seq)
{
    last_target_action_seq = seq;
    has_last_target_action = true;

    // 执行后清除跟踪，确保同一过程只执行一次。
    reset_target_tracking();

    if (cls == TARGET_WEAPON) {
        // 武器：正常直行。
        target_avoid_dir =0;
        target_avoid_ms = TARGET_AVOID_TIME_MS;
        return;
    }

    if (cls == TARGET_MATERIAL) {
        // 物资：右绕。
        target_avoid_dir = 1;
        target_avoid_ms = TARGET_AVOID_TIME_MS;
        return;
    }

    if (cls == TARGET_VEHICLE) {
        // 交通工具：左绕。
        target_avoid_dir = 0;
        target_avoid_ms = TARGET_AVOID_TIME_MS;
    }
}
void target_fusion_process(){
    // 先读取包编号，再读取数据，最后再次确认包编号。
    // 如果读取期间刚好收到新包，本周期不处理，下一周期再处理。
    uint32_t packet_before =
    remote_target.packet_counter.load(
        std::memory_order_acquire);

    if ((packet_before & 1U) != 0U ||
        packet_before == last_processed_target_packet) {
        return;
        }

    uint8_t seq = remote_target.seq.load();
    int cls = remote_target.class_id.load();
    int conf = remote_target.confidence.load();
    int x = remote_target.target_x.load();
    int y = remote_target.target_y.load();
    uint64_t last = remote_target.last_recv_ms.load();

    uint32_t packet_after =
    remote_target.packet_counter.load(
        std::memory_order_acquire);

    if (packet_before != packet_after ||
        (packet_after & 1U) != 0U) {
        return;}

    last_processed_target_packet = packet_after;

    const uint64_t now = get_ms();

    // 通信数据已经过期：不执行补偿动作。
    if (last == 0 || now - last > TARGET_TIMEOUT_MS) {
        return;
    }

    // 识别动作正在执行时，不建立新的识别动作。
    if (target_avoid_ms > 0) {
        return;
    }

    // 这个seq已经执行过，禁止同一目标重复执行。
    if (has_last_target_action &&
        seq == last_target_action_seq) {
        reset_target_tracking();
        return;
    }
    // seq变化表示出现了新的目标，不能混用上一个目标的投票数据。
    if (!tracked_target_seq_valid ||
        seq != tracked_target_seq) {
        reset_target_tracking();
        tracked_target_seq = seq;
        tracked_target_seq_valid = true;
        }

    // 识别板没有红块时发送-1,-1。
    const bool position_present = (x >= 0 && y >= 0);

    const bool position_centered =
        position_present &&
        x >= TARGET_X_CENTER_MIN &&
        x <= TARGET_X_CENTER_MAX;

    // 红块坐标存在且类别可信时，才加入最近5次投票。
    if (position_present &&
        conf >= TARGET_CONF_TH &&
        cls >= TARGET_WEAPON &&
        cls <= TARGET_VEHICLE) {
        push_target_id(cls);
        }

    if (position_centered) {
        if (!target_position_tracking) {
            target_position_tracking = true;
            target_track_start_ms = now;
        }

        target_last_position_ms = now;

        // 达到正常距离条件时，用最近5次id多数结果执行。
        if (y >= TARGET_TRIGGER_Y_TH) {
            int voted_cls = get_target_majority_id();

            if (voted_cls != TARGET_UNKNOWN) {
                execute_target_action(voted_cls, seq);
            }
        }

        return;
    }

    if (!target_position_tracking) {
        return;
    }

    const uint64_t tracked_ms =
        now - target_track_start_ms;

    const uint64_t position_lost_ms =
        now - target_last_position_ms;

    // x/y仍然有效，但目标暂时跑出横向中心区域，不当作红块丢失。
    if (position_present) {
        if (position_lost_ms >
            TARGET_LOST_TRIGGER_MAX_MS) {
            reset_target_tracking();
        }

        return;
    }

    // 坐标已经变成-1,-1：
    // 连续跟踪时间足够，而且刚刚才丢失，立即补偿执行。
    if (tracked_ms >= TARGET_TRACK_MIN_MS &&
        position_lost_ms <= TARGET_LOST_TRIGGER_MAX_MS) {
        int voted_cls = get_target_majority_id();

        if (voted_cls != TARGET_UNKNOWN) {
            execute_target_action(voted_cls, seq);
            return;
        }
    }

    // 丢失太久后不再补偿，防止旧目标延迟触发。
    if (position_lost_ms >
        TARGET_LOST_TRIGGER_MAX_MS) {
        reset_target_tracking();
    }
}


void* realtime_task(void* arg) {
    wheel_turn_pid = PID_Position_Init(0.015, 0, 0, 0.20, 0, 50000, -50000, false, 0.2f);;
    left_wheel_speed_pid = PID_Incremental_Init(35, 8, 4, 6000, -6000, true, 0.25f);
    right_wheel_speed_pid = PID_Incremental_Init(35, 8, 4, 6000, -6000, true, 0.25f);

    switch_init();

    // tft180_init("/dev/fb0");

    ret1 = icm20948_i2c_bus_init(icm20948, "/dev/i2c-0", 0x68);
    ret2 = icm20948_configure(icm20948, ACCE_FS_8G, GYRO_FS_2000DPS);

    InitLookupTable();

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

    int otsu_threshold = 0;
    int otsu_threshold_pers = 0;
    int canny_lowThreshold = 11;
    int canny_highThreshold = 23;


    // 使用OpenCV捕获并解码 MJPEG 数据
    cv::VideoCapture cap;
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(cv::CAP_PROP_FPS, 120);
    cap.open(0);
    result_image = cv::Mat(60, 80, CV_8UC1);
    pwm_get_dev_info(SERVO_MOTOR_PWM, &servo_pwm_info);

    // UI_init();
    BEEP::beep_ms(200);

    if(ret1 != 0) goto OUT;
    if(ret2 != 0) goto OUT;

    while (running) {
        // 等待定时器事件
        struct epoll_event events[1];
        int num_events = epoll_wait(epoll_fd, events, 1, -1);
        if (num_events > 0 && events[0].data.fd == timer_fd) {
            uint64_t expirations;
            read(timer_fd, &expirations, sizeof(expirations));

            // 定时器周期检测
            std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
            start = std::chrono::steady_clock::now();
            // fprintf(stdout,"timer_event 距离上次事件%.2lfms\n", time_used.count() * 1000);
            LOGI("timer_event", "距离上次事件%.2lfms", time_used.count() * 1000);
            if(std::abs(time_used.count() * 1000 - timer_period) > 1) {
                fprintf(stdout,"timer_event 定时器周期不准确，误差: %.2lfms\n", time_used.count() * 1000 - timer_period);
                // LOGW("timer_event", "`定时器周期不准确，误差: %.2lfms", time_used.count() * 1000 - timer_period);
            }

            // Moto_L.set_speed(speedtest);
            // Moto_R.set_speed(speedtest);
            if (flag.start && !flag.stop && running) {
                pwm_set_duty("/dev/zf_device_pwm_esc_1", static_cast<uint16>(1050));
                pwm_set_duty("/dev/zf_device_pwm_servo", static_cast<uint16>(1050));
            }else
            {
                pwm_set_duty("/dev/zf_device_pwm_esc_1", static_cast<uint16>(500)),
                pwm_set_duty("/dev/zf_device_pwm_servo", static_cast<uint16>(500));
            }
            // 定时器中断开始
            MEASURE_TIME("realtime_task_cost", {
            icm20948_get_anglez(icm20948, 0.01f);
            // printf("Anglez:%f\n", icm20948_data.anglez);
            // });

            // 图像处理
            // MEASURE_TIME("realtime_task_cost", {
            cap >> frame;
            frame.copyTo(myframe);
            cv::flip(myframe, myframe, -1); // 上下翻转

            memcpy(LQU_CAM_image, myframe.data, 320 * 240);
            cv::resize(myframe, myframe, cv::Size(80,60));
            memcpy(gray_image, myframe.data, 80 * 60);
            ImagePerspective();
            cover_car_head_pers();
            cover_car_head();
            calculate_contrast_x8(reinterpret_cast<uint8_t *>(contrast_image), reinterpret_cast<const uint8_t *>(gray_image), 80, 60);
            memcpy((uint8_t *) binary_image, (const uint8_t *) contrast_image, 80 * 60);
            // MEASURE_TIME("rt task", {
            my_cv2_doubleThreshold(reinterpret_cast<uint8_t *>(binary_image), 80, 0, 0, 80, 60, canny_lowThreshold, canny_highThreshold);
            my_cv2_checkConnectivity(reinterpret_cast<uint8_t *>(binary_image), 80, 0, 0, 80, 60);
            my_cv2_threshold(reinterpret_cast<uint8_t *>(binary_image), 80, 0, 0, 80, 60, 127, 255);
            // });
            memcpy((uint8_t *) binary_image_bak, (const uint8_t *) binary_image, 80 * 60);
            memcpy((uint8_t *) gray_binary_image, (const uint8_t *) gray_image, 80 * 60);
            otsu_threshold = get_otsu_threshold(0, 20, 80, 60, reinterpret_cast<const uint8 *>(gray_image));
            my_cv2_threshold(reinterpret_cast<uint8 *>(gray_binary_image), 80, 0, 0, 80, 60, otsu_threshold, 255);

            calculate_contrast_x8(reinterpret_cast<uint8_t *>(contrast_pers_image), reinterpret_cast<const uint8_t *>(gray_pers_image), 40, 60);
            memcpy((uint8 *) binary_pers_image, (const uint8 *) contrast_pers_image, 40 * 60);  // 复制对比度图像到待二值化图像
            my_cv2_doubleThreshold(reinterpret_cast<uint8 *>(binary_pers_image), 40, 0, 0, 40, 60, canny_lowThreshold, canny_highThreshold);
            my_cv2_checkConnectivity(reinterpret_cast<uint8 *>(binary_pers_image), 40, 0, 0, 40, 60);  // 检查连通性
            my_cv2_threshold(reinterpret_cast<uint8 *>(binary_pers_image), 40, 0, 0, 40, 60, 127, 255);
            memcpy(gray_binary_pers_image, (const uint8 *) gray_pers_image, 40 * 60);
            otsu_threshold_pers = get_otsu_threshold(0, 20, 40, 60, reinterpret_cast<const uint8 *>(gray_pers_image));
            my_cv2_threshold(reinterpret_cast<uint8 *>(gray_binary_pers_image), 40, 0, 0, 40, 60, otsu_threshold_pers, 255);

            bottom_start_end_x_get();
            // get_max_middle_line_height();

            incision = incision_max;
            max_white_column_get(bottom_start_x > 30 ? bottom_start_x : 30, 1, bottom_end_x < 50 ? bottom_end_x : 50 , 59);
            // max_white_column_get(bottom_start_x > 20 ? bottom_start_x : 20, 1, bottom_end_x < 20 ? bottom_end_x : 20 , 59);

            get_distance_line();
            get_lost_count();
            draw_rectan();

            bottom_start_end_x_get_pers();
            max_white_column_get_pers(bottom_start_x_pers > 10 ? bottom_start_x_pers : 10, 1, bottom_end_x_pers < 30 ? bottom_end_x_pers : 30 , 58);
            get_distance_line_pers();
            get_narrow_line();

            element_check();

            clear_remote_target_if_timeout();
            target_fusion_process();

            element_count();
            element_process();

            image_diff_process();

            detect_count_max = get_border_line(80);
            outbounds_detection();

            // });
            // 动态Kp，Kd
            // if ((flag.need_sec_border && flag.right_sec_border && flag.right_border) ||
            //     (flag.need_sec_border && flag.left_sec_border && flag.left_border)){
            //     // wheel_turn_pid.Kp = Kp_max * (0.7 * (tanh(fabs((double)image_diff) / 6000)) + 0.3);
            //     // wheel_turn_pid.Kd = Kd_max * (0.6 * (tanh(fabs((double)image_diff) / 10000)) + 0.4);
            // }
            if ((counter.drive_in_left_roundabout > 200) || (counter.drive_in_right_roundabout > 200)) {

                // if ((angelZ - icm20948_data.anglez > 140 && angelZ - icm20948_data.anglez < 270)
                //     || (angelZ - icm20948_data.anglez < -140 && angelZ - icm20948_data.anglez > -270)) {
                //     max_white_column.left_height = 42;
                //     wheel_turn_pid.Kp = Kp_max;
                //     wheel_turn_pid.Kd = Kd_max;
                //     // wheel_turn_pid.Kp = Kp_max * 0.8;
                //     // wheel_turn_pid.Kd = Kd_max * 0.8;
                // }else{
                    max_white_column.left_height = 43;
                    wheel_turn_pid.Kp = Kp_max;
                    wheel_turn_pid.Kd = Kd_max;
                // }
            }else if(counter.drive_in_crossroad > 400){
                wheel_turn_pid.Kp = Kp_max ;//* 1.10f;
                wheel_turn_pid.Kd = Kd_max ;//* 0.90f;
            }else{

                wheel_turn_pid.Kp = Kp_max;
                wheel_turn_pid.Kd = Kd_max;
                // wheel_turn_pid.Kp = Kp_max * (0.7 * (tanh(fabs(static_cast<double>(image_diff)) / 8000)) + 0.3);
                // wheel_turn_pid.Kd = Kd_max * (0.6 * (tanh(fabs(static_cast<double>(image_diff)) / 8000)) + 0.4);
            }

            // 更新电机速度
            Moto_L.update_speed();
            Moto_R.update_speed();

            // 位置环PID(需要优化)
            turn_pidout = PID_Position_Calc(&wheel_turn_pid, 0, 0, static_cast<float>(image_diff));
            turn_angle = turn_pidout / 10;

            if(turn_angle > turn_max) turn_angle = turn_max;
            if(turn_angle < -turn_max) turn_angle = -turn_max;

            float turn_angle_real = turn_angle * CAR_ANGLE_CONVERT * (3.14159265358979323846 / 180);

            if(turn_angle_real == 0) {
                left_speed_setpoint = speed_setpoint;
                right_speed_setpoint = speed_setpoint;
            } else {
                left_speed_setpoint = fabs(speed_setpoint * (((CAR_WHEELBASE_L / tan(turn_angle_real)) + (CAR_WHEELBASE_B / 2)) / sqrt(pow(CAR_WHEELBASE_L / 2, 2) + pow(CAR_WHEELBASE_L / tan(turn_angle_real), 2))));
                right_speed_setpoint = fabs(speed_setpoint * (((CAR_WHEELBASE_L / tan(turn_angle_real)) - (CAR_WHEELBASE_B / 2)) / sqrt(pow(CAR_WHEELBASE_L / 2, 2) + pow(CAR_WHEELBASE_L / tan(turn_angle_real), 2))));

                double diff = (left_speed_setpoint > speed_setpoint) ? (left_speed_setpoint - speed_setpoint) : (right_speed_setpoint - speed_setpoint);
                // float diff_gain = 1.0f;
                left_speed_setpoint -= diff;
                right_speed_setpoint -= diff;
                // float outer_boost = 1.08f;
                // float diff_gain = 1.15f;
                // //
                // if (left_speed_setpoint > right_speed_setpoint) {
                //     left_speed_setpoint = speed_setpoint * (outer_boost);
                //     right_speed_setpoint -= diff * diff_gain;
                // } else {
                //     right_speed_setpoint = speed_setpoint * (outer_boost);
                //     left_speed_setpoint -= diff * diff_gain;
                // }

            }
            if (counter.start_motor_delay > MOTO_START_DELAY && counter.start_motor_delay < MOTO_START_DELAY+500) {
                left_speed_setpoint = speed_setpoint/2;
                right_speed_setpoint = speed_setpoint/2;
            }

            // servo_set_angle(SERVO_MOTOR_MID - turn_angle);

            // MEASURE_TIME("realtime_task_cost", {
            // vofa_udp.printf("%d,%d,%d\n",Moto_L.speed,Moto_R.speed,blind_line);
            // vofa_udp.printf("%d,%d,%d,%d\n",distances[40],distances[35], distances[30], distances[25]);
            // vofa_udp.printf("%d,%d,%d,%d,%d,%d,%d\n",max_white_column.left_x,max_white_column.right_x,max_white_column.start_y,max_white_column.end_y,distance_middle_line[0][0] - distance_middle_line[20][0],counter.drive_in_ramp, flag.found_ramp);
            // vofa_udp.printf("%d,%d,%d\n",abs(max_white_column.left_x - max_white_column.right_x),max_white_column.end_y,distance_middle_line[0][0] - distance_middle_line[20][0]);
            // vofa_udp.printf("%d,%d,%d,%d,%d\n",dis_index,distances[dis_index],left_distance[dis_index][0],right_distance[dis_index][0],detect_count_max);
            // vofa_udp.printf("%d,%d,%d,%.3f,%.3f\n",image_diff, flag.drive_in_crossroad, counter.drive_in_crossroad, wheel_turn_pid.Kp, wheel_turn_pid.Kd);
            // vofa_udp.printf("%d,%d,%d,%d,%d\n",left_lost_count,right_lost_count,abs(left_lost_count - right_lost_count),distances[25] - road_distances[25],distances[20] - road_distances[20]);
            // vofa_udp.printf("%d,%d,%d,%d,%d,%.1f\n",id,left_lost_count,right_lost_count,rstate,counter.drive_in_left_roundabout,angelZ - icm20948_data.anglez);
            // vofa_udp.printf("%d,%d,%d,%d,%d\n",dis_index,left_border[dis_index][0],right_border[dis_index][0],left_border[dis_index][1],right_border[dis_index][1]);
            // vofa_udp.printf("%d,%d\n",bottom_start_x,bottom_end_x);
            // vofa_udp.printf("%d,%d,%d,%d,%d,%d,%d,%d\n",left_lost_count,right_lost_count,max_white_column.left_height,lost_y1, left_lost_dir,right_lost_dir,left_reach_edge,right_reach_edge);
            // vofa_udp.printf("%d,%d,%d,%d\n",left_lost_count,right_lost_count,flag.found_left_roundabout,flag.found_right_roundabout);
            // vofa_udp.printf("%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d\n",left_speed_setpoint,right_speed_setpoint, Moto_L.speed, Moto_R.speed,left_wheel_pidout,right_wheel_pidout,image_diff);
            // vofa_udp.printf("%d,%d,%d,%d,%d,%d,%.1f\n",image_diff,id,left_lost_count,right_lost_count,rstate,counter.drive_in_right_roundabout,angelZ - icm20948_data.anglez);
            // vofa_udp.printf("%d,%d,%d,%d,%d,%d\n",lost_x1,lost_x2,lost_y1,lost_y2,x_left,x_right,left_reach_edge,right_reach_edge);
            // vofa_udp.printf("%d,%d,%d,%d,%d,%d\n",lost_x1,lost_x2,lost_y1,lost_y2,middle_line[60-lost_y1][0], middle_line[60-lost_y1][1]);
            // vofa_udp.printf("%d,%d,%d,%d\n",flag.found_garage,counter.found_garage, garage_count,detect_count_max);
            // vofa_udp.printf("%d,%d,%.2f,%d\n",image_diff,counter.drive_in_crossroad,turn_angle,counter.drive_in_obstacle);
            // vofa_udp.printf("crossroad:%d,diff%d\n",counter.drive_in_crossroad,image_diff);
            // vofa_udp.printf("%d,%d,%d,%d,%d,%d\n",lost_x1,lost_x2,lost_y1,lost_y2,left_reach_edge,right_reach_edge);
            // vofa_udp.printf("%d,%d,%d,%d,%d,%d\n",lost_x1,lost_x2,lost_y1,lost_y2,left_reach_edge,right_reach_edge);
            // vofa_udp.printf("%d\n",flag.stop);
            // vofa_udp.printf("%d,%d,%d,%d\n",narrow_line_index,flag.advance_avoid_obstacle_dir,flag.found_obstacle,counter.drive_in_obstacle);
            // vofa_udp.printf("%d,%d\n",dis_index,distance_middle_line_pers[dis_index][0]);
            // vofa_udp.printf("%d,%.2f,%.2f,%.2f\n",flag.stop,speed_setpoint,left_speed_setpoint,right_speed_setpoint);
            // vofa_udp.printf("diff:%d,cnt:%d,kp:%.2f,kd:%.2f,ang:%.2f\n",image_diff,counter.drive_in_left_roundabout,wheel_turn_pid.Kp,wheel_turn_pid.Kd,angelZ - icm20948_data.anglez);
            // vofa_udp.printf("rt:%d,dt:%d\n",running_time,delay_time);
            // vofa_udp.printf("%d,%d,%d,%d,%d,%d,%d,%d\n", center_target_found, center_target_count, remote_target.class_id.load(), remote_target.confidence.load(), remote_target.target_x.load(), remote_target.target_y.load(), flag.advance_avoid_obstacle_dir, counter.drive_in_obstacle);
            // });
                // vofa_udp.printf("%d\n",speedtest);
                // vofa_udp.printf("%d,%d,%d,%d,%d,%d,%.2f,%d,%d\n",
                //     remote_target.class_id.load(),
                //     remote_target.confidence.load(),
                //     remote_target.target_x.load(),
                //     remote_target.target_y.load(),
                //     target_avoid_dir,
                //     target_avoid_ms,
                //     turn_angle,
                //     flag.advance_avoid_obstacle_dir,
                //     counter.drive_in_obstacle);


                // vofa_udp.printf("%d,%d,%d,%d,%d,%d,%d,%d\n",
                //     image_diff,
                //     counter.drive_in_obstacle,
                //     narrow_line_index,
                //     flag.advance_avoid_obstacle_dir,
                //     flag.found_obstacle,
                //     counter.found_obstacle,
                //     left_lost_count,
                //     right_lost_count);

                vofa_udp.printf("%.2f,%.2f,%.2f,%d,%d,%d,%.2f,%d\n",
                    speed_setpoint,
                    left_speed_setpoint,
                    right_speed_setpoint,
                    counter.drive_in_obstacle,
                    left_lost_count,
                    image_diff,
                    turn_angle,
                    distances[39]);

            // 速度环PID
            if (counter.drive_in_left_roundabout > 5000 || counter.drive_in_right_roundabout > 5000) {
                speed_setpoint = 190;
            }else if(counter.drive_in_crossroad > 400) {
                speed_setpoint = 190;
            }else {
                if(max_white_column.left_height > max_white_column_height) {
                    speed_setpoint = (speed_base / (1 - boost_ratio)) * (1 - boost_ratio * (tanh(static_cast<float>(abs(max_white_column_height - max_white_column_height)) / 3.3)));
                } else {
                    speed_setpoint = (speed_base / (1 - boost_ratio)) * (1 - boost_ratio * (tanh(static_cast<float>(abs(max_white_column.left_height - max_white_column_height)) / 3.3)));
                }
            }
                if (remote_target.target_x > 0) {
                    // speed_setpoint *= 0.60f;
                    speed_setpoint = 40;
                // } else if (counter.drive_in_obstacle > 0) {
                //     // speed_setpoint *= 0.65f;
                }

            // left_wheel_pidout  = PID_Incremental_Calc(&left_wheel_speed_pid, (float32) Moto_L.speed, left_speed_setpoint);
            // right_wheel_pidout = PID_Incremental_Calc(&right_wheel_speed_pid, (float32) Moto_R.speed, right_speed_setpoint);
            left_wheel_pidout  = PID_Incremental_Calc(&left_wheel_speed_pid, static_cast<float32>(Moto_L.speed), left_speed_setpoint);
            right_wheel_pidout = PID_Incremental_Calc(&right_wheel_speed_pid, static_cast<float32>(Moto_R.speed), right_speed_setpoint);

            // 设置速度
            if(flag.stop){
                speed_base = 0;
                speed_setpoint = 0;
                left_speed_setpoint = 0;
                right_speed_setpoint = 0;
                if(counter.start_motor_delay > 0){
                    Moto_L.set_speed(static_cast<int>(left_wheel_pidout));
                    Moto_R.set_speed(static_cast<int>(right_wheel_pidout));
                    counter.start_motor_delay -= 10;
                } else {
                    Moto_L.set_speed(0);
                    Moto_R.set_speed(0);
                }
            }else {
                if(flag.start == true){
                    if(counter.start_motor_delay > MOTO_START_DELAY){
                        Moto_L.set_speed(static_cast<int>(left_wheel_pidout));
                        Moto_R.set_speed(static_cast<int>(right_wheel_pidout));
                    } else {
                        left_wheel_speed_pid.error = 0;
                        left_wheel_speed_pid.last_error = 0;
                        left_wheel_speed_pid.last_last_error = 0;
                        left_wheel_speed_pid.last_out = 0;
                        left_wheel_speed_pid.out = 0;
                        right_wheel_speed_pid.error = 0;
                        right_wheel_speed_pid.last_error = 0;
                        right_wheel_speed_pid.last_last_error = 0;
                        right_wheel_speed_pid.last_out = 0;
                        right_wheel_speed_pid.out = 0;
                        wheel_turn_pid.last_error = 0;
                        wheel_turn_pid.last_out = 0;
                        wheel_turn_pid.integral = 0;
                        turn_pidout = 0;
                        turn_angle = 0;

                    }
                    counter.start_motor_delay += 10;
                }
            }


            image_diff_process();

            if(counter.beep_ms > 0) {
                counter.beep_ms -= 10;
                beep.beep_on();
            }else {
                beep.beep_off();
            }

            // 出界检测
            if(blind_line <= 5 && abs(bottom_start_x - bottom_end_x) <= 20 && Moto_L.speed > 40 && Moto_R.speed > 40) {
                counter.out_of_bound += 5;
            }else {
                counter.out_of_bound = 0;
            }
            if(counter.out_of_bound > 15) {
                flag.stop = true;
            }

            if (Moto_L.speed > 50 || Moto_R.speed > 50) {
                if (flag.start == false) {
                    left_wheel_speed_pid.error = 0;
                    left_wheel_speed_pid.last_error = 0;
                    left_wheel_speed_pid.last_last_error = 0;
                    left_wheel_speed_pid.last_out = 0;
                    left_wheel_speed_pid.out = 0;
                    right_wheel_speed_pid.error = 0;
                    right_wheel_speed_pid.last_error = 0;
                    right_wheel_speed_pid.last_last_error = 0;
                    right_wheel_speed_pid.last_out = 0;
                    right_wheel_speed_pid.out = 0;
                }
            }

            // 电机堵转或编码器异常时停止电机
            if(protect == true) {
                if ((abs(Moto_L.speed) < 10 && abs(Moto_L.speed - Moto_R.speed) > 60) ||
                    (abs(Moto_R.speed) < 10 && abs(Moto_L.speed - Moto_R.speed) > 60)) {
                        counter.stop_motor += 10;
                } else {
                    counter.stop_motor = 0;
                }

                if (counter.stop_motor > 250) {
                    flag.stop = true;
                }
            }

            if (running_time <= 0) {
                flag.stop = true;
            }
            if (running_time > 0 && flag.start && counter.start_motor_delay > MOTO_START_DELAY) {
                running_time -= 10;
            }
            });
        }

    }
            // 定时器中断结束

OUT:
    emergency_stop_outputs();
    close(timer_fd);
    close(epoll_fd);
    icm20948_delete(icm20948);
    running = false;
    return nullptr;
}

void element_count() {
    // 十字
    if(flag.found_crossroad == true && counter.drive_in_crossroad == 0 && counter.drive_in_ramp == 0) {
        counter.found_crossroad += 2;
        counter.found_left_roundabout = 0;
        counter.found_right_roundabout = 0;
        if(counter.found_crossroad > 2){
            BEEP::beep_ms(500);
            counter.drive_in_crossroad = 4000;
        }
    }

    if(counter.found_crossroad > 0){
        counter.found_crossroad--;
    }

    if(counter.drive_in_crossroad > 0){
        counter.drive_in_crossroad -= 10;
    }

    // 车库
    if (flag.found_garage == true && counter.drive_in_ramp == 0 && counter.drive_in_crossroad == 0 && running_time < delay_time - 2000) {
        counter.found_garage += 2;
        if (counter.found_garage > 3) {
            BEEP::beep_ms(400);
            counter.drive_in_garage = 300;
        }
    }

    if (counter.found_garage > 0) {
        counter.found_garage--;
    }

    if (counter.drive_in_garage > 0) {
        counter.drive_in_garage -= 10;
    }

    // 坡道
    if(flag.found_ramp && counter.drive_in_ramp == 0){
        counter.found_ramp += 2;
        if(counter.found_ramp > 7){
            BEEP::beep_ms(200);
            counter.drive_in_ramp = 300;
        }
    }

    if(counter.found_ramp > 0){
        counter.found_ramp--;
    }

    if(counter.drive_in_ramp > 0){
        counter.drive_in_ramp -= 10;
    }

    // 障碍计数处理

    if(flag.found_obstacle == true
        && counter.drive_in_obstacle == 0
        && counter.drive_in_ramp == 0
        && counter.drive_in_crossroad == 0) {

        counter.found_obstacle += 2;
        if (counter.found_obstacle > 5) {
            BEEP::beep_ms(200);
            counter.drive_in_obstacle = 1000;
            counter.found_obstacle = 0;

            target_avoid_ms = 0;
            target_avoid_dir = 0;
        }
    }

    if (flag.found_obstacle > 0) {
        flag.found_obstacle--;
    }

    if (counter.drive_in_obstacle > 0) {
        counter.drive_in_obstacle -= 10;
    }
    if (target_avoid_ms > 0) {
        target_avoid_ms -= 10;
        if (target_avoid_ms <= 0) {
            target_avoid_ms = 0;
            target_avoid_dir = 0;
        }
    }
    // 左环岛计数处理
    if(flag.found_left_roundabout && counter.drive_in_left_roundabout == 0 && counter.drive_in_right_roundabout == 0 && counter.drive_in_ramp == 0) {
        counter.found_left_roundabout += 2;
        if(counter.found_left_roundabout > 11){
            BEEP::beep_ms(200);
            counter.drive_in_left_roundabout = 10000;
        }
    }

    if(counter.found_left_roundabout > 0){
        counter.found_left_roundabout--;
    }

    if(counter.drive_in_left_roundabout > 0){
        counter.drive_in_left_roundabout -= 10;
        counter.drive_in_left_roundabout -= (counter.drive_in_left_roundabout % 10);
    }

    // 右环岛计数处理
    if(flag.found_right_roundabout && counter.drive_in_left_roundabout == 0 && counter.drive_in_right_roundabout == 0 && counter.drive_in_ramp == 0) {
        counter.found_right_roundabout += 2;
        if(counter.found_right_roundabout >11){
            BEEP::beep_ms(200);
            counter.drive_in_right_roundabout = 10000;
        }
    }

    if(counter.found_right_roundabout > 0){
        counter.found_right_roundabout--;
    }

    if(counter.drive_in_right_roundabout > 0){
        counter.drive_in_right_roundabout -= 10;
        counter.drive_in_right_roundabout -= (counter.drive_in_right_roundabout % 10);
    }
    // vofa_udp.printf("anglez:%.2f\n",icm20948_data.anglez);

}

void element_process() {
    int count=0;
    // 十字
    if (counter.drive_in_crossroad > 50) {
        // if (counter.drive_in_crossroad > 50) {
        //     fix_crossroad();
        // }
        if(flag.found_crossroad) {
            counter.drive_in_crossroad = 4000;
        } else if (distances[5] > road_distances[10] + 5 && rstate == 0){
            rstate = 1;
            counter.drive_in_crossroad = 2500;
        } else if(distances[5] < road_distances[10] + 5 && rstate == 1){
            rstate = 0;
            counter.drive_in_crossroad = 0;
            flag.drive_in_crossroad = !flag.drive_in_crossroad;
        }
    }

    // 车库
    if (counter.drive_in_garage > 0 && stop_in_garage == true && counter.drive_in_garage < 50) {
        flag.stop = true;
    }

    // 坡道
    if(counter.drive_in_ramp > 0){
        if(flag.found_ramp == true && counter.drive_in_ramp > 280){
            counter.drive_in_ramp = 300;
            ramp_angle = icm20948_data.anglez;
        }
    }

    // 环岛
    count = 0;
    for (int q = 80 / 2 - 1; q >= 0; q--) {
        if (binary_image_bak[50][q] == 0) {
            count++;
        } else {
            break;
        }
    }

    // 左环岛
    if (counter.drive_in_left_roundabout > 8600) {
        count = 0;
        for (int q = 80 / 2 - 1; q >= 0; q--) {
            if (binary_image_bak[50][q] == 0) {
                count++;
            } else {
                break;
            }
        }
        if (rstate == 0 && count > 35) {
            rstate = 1;
        }
        if((rstate == 1 && count < 31) || count < 5){
            rstate = 0;
            counter.drive_in_left_roundabout = 5000;
            angelZ = icm20948_data.anglez;
        }
        fix_left_break(0, 60);
    } else if(counter.drive_in_left_roundabout > 5000){
        rstate = 0;
        counter.drive_in_left_roundabout = 5000;
        angelZ = icm20948_data.anglez;
        fix_left_break(0, 60);
    } else if(counter.drive_in_left_roundabout > 80){
        int end_x = bottom_end_x;
        if(end_x < road_distances[0]) end_x = road_distances[0];
        if(end_x > 78) end_x = 78;
        if(angelZ - icm20948_data.anglez < 60) {
            erase_top_right_road(end_x - road_distances[15], 0, end_x, 60);
        } else if(angelZ - icm20948_data.anglez < 325){
            erase_top_right_road(end_x - road_distances[35], 0, end_x, 60);
            counter.drive_in_left_roundabout = 800;
        } else if(counter.drive_in_left_roundabout > 300){
            fix_left_break(0, 60);
        } else if(counter.drive_in_left_roundabout > 100){
            fix_left_break(0, 60);
            if(distances[0] < road_distances[0] + 5){
                counter.drive_in_left_roundabout = 100;
            }
        }
    }

    // 右环岛
    if (counter.drive_in_right_roundabout > 8600){
        count = 0;
        for (int q = 80 / 2 - 1; q < 80; q++) {
            if (binary_image_bak[50][q] == 0) {
                count++;
            } else {
                break;
            }
        }
        if (rstate == 0 && count > 35) {
            rstate = 1;
        }
        if(rstate == 1 && count < 31 || count < 5){
            rstate = 0;
            counter.drive_in_right_roundabout = 5000;
            angelZ = icm20948_data.anglez;
        }
        fix_right_break(0, 60);
    } else if(counter.drive_in_right_roundabout > 5000){
        rstate = 0;
        counter.drive_in_right_roundabout = 5000;
        fix_right_break(0, 60);
        angelZ = icm20948_data.anglez;
    } else if(counter.drive_in_right_roundabout > 80){
        int start_x = bottom_start_x;
        if(start_x > 80 - road_distances[0]) start_x = 80 - road_distances[0];
        if(start_x < 1) start_x = 1;
        if(angelZ - icm20948_data.anglez > -60) {
            erase_top_left_road(start_x + road_distances[15], 0, start_x, 60);
        } else if(angelZ - icm20948_data.anglez > -325) {
            erase_top_left_road(start_x + road_distances[35], 0, start_x, 60);
            counter.drive_in_right_roundabout = 800;
        } else if(counter.drive_in_right_roundabout > 300){
            fix_right_break(0, 60);
        } else if(counter.drive_in_right_roundabout > 100) {
            fix_right_break(0, 60);
            if (distances[0] < road_distances[0] + 5) {
                counter.drive_in_right_roundabout = 100;
            }
        }
    }
}


void element_check() {
    check_crossroad();
    check_garage();
    check_ramp();
    check_obstacle();
    check_roundabout();
}

void image_diff_process() {
    if(counter.drive_in_crossroad > 400) {
        // 十字处理
        left_sum = 0;
        right_sum = 0;
        for(int i = 0; i < distance_middle_line_index; i++){
            float y_weight= 60 - distance_middle_line[i][1];
            if (y_weight<0) y_weight=0;
            left_sum -= static_cast<float>(distance_middle_line[i][0] - IMAGE_MIDDLE) * (1.0f + y_weight/15.0f);
        }
        left_sum *= 10;
        image_diff = right_sum - left_sum;
    }else if(counter.drive_in_obstacle > 0) {
        left_sum = 0;
        right_sum = 0;
        int img_start = 0;
        if(img_start < incision) img_start = incision;
        int img_end = img_start + IMAGE_VALID_NUM;
        if(img_end > detect_count_max) img_end = detect_count_max;

        for(int i = img_start; i < img_end; i++) {
            float dec = 4 * max_white_column.left_height - 150;
            if (dec < 0) dec = 0;

            float y_weight = static_cast<float>(middle_line[i][1]) - dec;
            if (y_weight < 0) y_weight = 0;

            left_sum -= static_cast<float>(middle_line[i][0] - IMAGE_MIDDLE)
                        * (1.0f + y_weight / 20.0f);
        }

        left_sum *= 10;
        left_sum += (6000 * flag.advance_avoid_obstacle_dir);
        image_diff = right_sum - left_sum;
    }else if(target_avoid_ms > 0) {
        left_sum = 0;
        right_sum = 0;
        int img_start = 0;
        if(img_start < incision) img_start = incision;
        int img_end = img_start + IMAGE_VALID_NUM;
        if(img_end > detect_count_max) img_end = detect_count_max;

        for(int i = img_start; i < img_end; i++) {
            float dec = 4 * max_white_column.left_height - 150;
            if (dec < 0) dec = 0;

            float y_weight = static_cast<float>(middle_line[i][1]) - dec;
            if (y_weight < 0) y_weight = 0;

            left_sum -= static_cast<float>(middle_line[i][0] - IMAGE_MIDDLE)
                        * (1.0f + y_weight / 20.0f);
        }

        left_sum *= 10;

        int bias = TARGET_AVOID_BIAS * target_avoid_dir;

        // 最后一段时间加反向小偏置，帮助回正
        if (target_avoid_ms <= TARGET_AVOID_RETURN_MS) {
            bias = -TARGET_AVOID_RETURN_BIAS * target_avoid_dir;
        }

        left_sum += bias;
        image_diff = right_sum - left_sum;

    }else {
        left_sum = 0;
        right_sum = 0;
        int img_start = 0;
        if(img_start < incision)img_start = incision;
        int img_end = img_start + IMAGE_VALID_NUM;
        if(img_end > detect_count_max) img_end = detect_count_max;
        for(int i = img_start; i < img_end; i++) {
            left_sum -= static_cast<float>(middle_line[i][0] - IMAGE_MIDDLE);
        }
        left_sum *= 10;
        // left_sum += 4000 * cornering;
        image_diff = right_sum - left_sum;
    }
    // if ((flag.need_sec_border && flag.right_sec_border && flag.right_border) ||
    // (flag.need_sec_border && flag.left_sec_border && flag.left_border)) {
    //     if (image_diff < 0) {
    //         image_diff -= left_reach_edge * 60;
    //     }else {
    //         image_diff += right_reach_edge * 60;
    //     }
    // }else if (left_reach_edge > 25 || right_reach_edge > 25) {
    //     if (image_diff < 0) {
    //         image_diff -= left_reach_edge * 30;
    //     }else {
    //         image_diff += right_reach_edge * 30;
    //     }
    // }
}

// 非实时任务线程函数
void *non_realtime_task(void *arg) {
    cv::Mat gray;
    cv::Mat gray1ch(60, 80, CV_8UC1, binary_image);
    cv::Mat gray3ch;
    cv::Mat cv_image(60, 40, CV_8UC1, gray_pers_image); // 60行40列的灰度图
    cv::Mat cv_image3ch;
    std::vector<uchar> jpg;


    while (running) {
            cv::cvtColor(gray1ch, gray3ch, cv::COLOR_GRAY2BGR);

            // MEASURE_TIME("ncnn input convert", {
            // // 将 OpenCV 的 BGR 数据转换为 ncnn 专属的浮点张量格式 (ncnn::Mat)
            // // 这是每次运行模型前必须要做的操作。
            // // 参数：像素指针, 格式转换类型, 宽, 高
            // ncnn::Mat in = ncnn::Mat::from_pixels(
            //     gray1ch.data,            // 直接喂单通道的指针
            //     ncnn::Mat::PIXEL_GRAY,   // 关键：指定为纯灰度格式
            //     gray1ch.cols,
            //     gray1ch.rows
            // );

            // });
            // fprintf(stdout,"%d\n",iii++);
            frame.copyTo(gray);
            // MEASURE_TIME("non rt task", {
                // memcpy(gray1ch_image,gray_image, 80 * 60);
                // memcpy(gray1ch_image, binary_image, 80 * 60);
            // });
            //
            //     atag.detect(gray);
            // // });
            // // MEASURE_TIME("getclosettagindex", {
            //     atag.getClosetTagIndex();
            // // });
            // // MEASURE_TIME("draw", {
            //     atag.draw(gray3ch, 0.25);
            // // });detect_count_max:
            // // MEASURE_TIME("getid", {
            //     id = atag.getClosetTagID();
            // // });
            // // MEASURE_TIME("getdistance", {
            //     distance = atag.getClosetTagDistance(1500);
            //     // cv::putText(gray3ch, std::to_string(distance), cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0xff, 0), 2);
            // // });
                tcp_draw_real_border_line(gray3ch, 0, 0, left_border, cv::Scalar(0, 0xff, 0));
                tcp_draw_real_border_line(gray3ch, 0, 0, right_border, cv::Scalar(0xff, 0, 0));
                tcp_draw_real_border_line(gray3ch, 0, 0, middle_line, cv::Scalar(0, 0, 0xff));
            if (counter.drive_in_crossroad>0) {
                tcp_draw_real_border_line(gray3ch, 0, 0, distance_middle_line, cv::Scalar(0xff, 0xff, 0));
            }
                // tcp_draw_real_border_line(gray3ch, 0, 0, distance_middle_line, cv::Scalar(0xff, 0xff, 0));
                cv::circle(gray3ch, cv::Point(lost_x1, lost_y1), 0, cv::Scalar(255, 0, 255), -1);
                cv::circle(gray3ch, cv::Point(lost_x2, lost_y2), 0, cv::Scalar(255, 0, 255), -1);
                std::string info_text = "C:" + std::to_string(counter.drive_in_crossroad);
                cv::putText(gray3ch, info_text, cv::Point(2, 12), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 255, 255), 1);
               //看识别目标线路绘制
                cv::cvtColor(cv_image,cv_image3ch, cv::COLOR_GRAY2BGR);
                tcp_draw_border_line(cv_image3ch,0,0,left_distance_line_pers, cv::Scalar(0xff, 0xFF, 0));
                tcp_draw_border_line(cv_image3ch,0,0,right_distance_line_pers, cv::Scalar(0xff, 0xFF, 0));
                tcp_draw_border_line(cv_image3ch,0,0,distance_middle_line_pers, cv::Scalar(0, 0xff, 0));
                tcp_draw_border_line(cv_image3ch,0,0,narrow_line, cv::Scalar(0xff, 0, 0));

            // MEASURE_TIME("image write", {
            // 发送已绘制标注的图像，避免只发送原始帧
            // vofa_udp_img.imwrite(*LQU_CAM_image, 320,240);
            // vofa_tcp.imwrite(cv_image3ch);
            vofa_tcp.imwrite(gray3ch);
            // vofa_tcp.imwrite(*LQU_CAM_image, 320,240);

                // http << gray3ch;
        if (Moto_L.speed < 10 && Moto_R.speed < 10 && !flag.start) {
            // MEASURE_TIME("UI_time", {
                UI_key_process();
                // UI_show();
            // });
        }
        // 帧率限制 ~30fps，避免无节制循环烧CPU
        usleep(33000);
    }
    return nullptr;
}

void signal_handler(int sig) {
    emergency_stop_outputs();
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

    system("v4l2-ctl -d /dev/video0 -c contrast=38 -c gamma=200 -c exposure_auto=1 -c exposure_absolute=90 -c sharpness=57");



    // 创建posix线程
    pthread_t rt_thread;
    pthread_t nrt_thread;

    std::thread(target_recv_thread).detach();
    std::thread(terminal_stop_thread).detach();

    pthread_create(&rt_thread, nullptr, realtime_task, nullptr);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000)); // 运行1秒
    pthread_create(&nrt_thread, nullptr, non_realtime_task, nullptr);
    std::this_thread::sleep_for(std::chrono::seconds(1000)); // 运行100秒

    running = false; // 停止线程

    log_shutdown();
}
