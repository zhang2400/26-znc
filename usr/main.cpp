#include "main.h"

float CAR_ANGLE_CONVERT = 3.1f;

// PID相关变量
PID_Incremental left_wheel_speed_pid;
PID_Incremental right_wheel_speed_pid;

PID_Position wheel_turn_pid;

// 电机相关变量
float left_wheel_pidout = 0;
float right_wheel_pidout = 0;

float speed_setpoint = 70;
float left_speed_setpoint = 0;
float right_speed_setpoint = 0;

float turn_pidout = 0;
float turn_angle = 0;
float turn_max = 14;

float Kp_max = 0.022f;
float Kd_max = 0.20f;

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

int left_sum;
int right_sum;
int start;
int end;

int dis_index = 0;
int rstate = 0;
int count = 0;
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
auto udp_transport = std::make_unique<UDPTransport>("192.168.5.16", 1349);
auto vofa_udp = VOFA(std::move(udp_transport));

int incision = 12;
int incision_max = 12;
int detect_count_max = 0;

int blind_line;
int protect = true;

int i = SERVO_MOTOR_MID;
int j = 0;
int running_time = 2000;
int stop_in_garage = true;

BEEP beep(GPIO61);
Moto Moto_L(PWM1_GPIO65, 75, PWM0_GPIO64, 73, false);
Moto Moto_R(PWM2_GPIO66, 74, PWM3_GPIO67, 72, true);

// tag码相关变量
int id = -1;
double distance = -1;

void* realtime_task(void* arg) {
    wheel_turn_pid = PID_Position_Init(0.015, 0, 0, 0.20, 0, 50000, -50000, false, 0.2f);;
    left_wheel_speed_pid = PID_Incremental_Init(50, 12, 4, 7000, -7000, false, 0.25f);
    right_wheel_speed_pid = PID_Incremental_Init(50, 12, 4, 7000, -7000, false, 0.25f);

    Servo Servo(SERVO_MOTOR_PWM, SERVO_MOTOR_FREQ, SERVO_MOTOR_L_MAX, SERVO_MOTOR_R_MAX, SERVO_MOTOR_MID);

    switch_init();

    ret1 = icm20948_i2c_bus_init(icm20948, "/dev/i2c-1", 0x68);
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
    std::chrono::steady_clock::time_point end;

    int otsu_threshold = 0;
    int contrast_threshold = 20;
    int canny_lowThreshold = 17;
    int canny_highThreshold = 35;

    cv::VideoCapture cap;
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 320);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 240);
    cap.set(cv::CAP_PROP_FPS, 120);
    cap.open(0);
    result_image = cv::Mat(60, 80, CV_8UC1);

    beep.beep_ms(400);

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
            end = std::chrono::steady_clock::now();
            std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end - start);
            start = std::chrono::steady_clock::now();
            // fprintf(stdout,"timer_event 距离上次事件%.2lfms\n", time_used.count() * 1000);
            LOGI("timer_event", "距离上次事件%.2lfms", time_used.count() * 1000);
            if(std::abs(time_used.count() * 1000 - timer_period) > 1) {
                // fprintf(stdout,"timer_event 定时器周期不准确，误差: %.2lfms\n", time_used.count() * 1000 - timer_period);
                LOGW("timer_event", "定时器周期不准确，误差: %.2lfms", time_used.count() * 1000 - timer_period);
            }

            // MEASURE_TIME("realtime_task_cost", {
            icm20948_get_anglez(icm20948, 0.01f);
            printf("Anglez:%f\n", icm20948_data.anglez);
            // });

            // 图像处理
            cap >> frame;
            // vofa_tcp.imwrite(frame);
            frame.copyTo(myframe);
            memcpy(LQU_CAM_image, myframe.data, 320 * 240);
            cv::resize(myframe, myframe, cv::Size(80,60));
            memcpy(gray_image, myframe.data, 80 * 60);
            ImagePerspective();

            calculate_contrast_x8((uint8_t *)contrast_image, (const uint8_t *)gray_image, 80, 60);
            memcpy((uint8_t *) binary_image, (const uint8_t *) contrast_image, 80 * 60);
            // MEASURE_TIME("rt task", {
            my_cv2_doubleThreshold((uint8_t *) binary_image, 80, 0, 0, 80, 60, canny_lowThreshold, canny_highThreshold);
            my_cv2_checkConnectivity((uint8_t *) binary_image, 80, 0, 0, 80, 60);
            my_cv2_threshold((uint8_t *) binary_image, 80, 0, 0, 80, 60, 127, 255);
            // });
            memcpy((uint8_t *) binary_image_bak, (const uint8_t *) binary_image, 80 * 60);
            memcpy((uint8_t *) gray_binary_image, (const uint8_t *) gray_image, 80 * 60);
            otsu_threshold = get_otsu_threshold(0, 40, 60, 80, (const uint8 *) gray_image);
            my_cv2_threshold((uint8 *) gray_binary_image, 80, 0, 0, 80, 60, otsu_threshold, 255);

            // vofa_tcp.imwrite((uint8_t *)gray_binary_image, 80, 60);

            bottom_start_end_x_get();
            // get_max_middle_line_height();

            incision = incision_max;
            max_white_column_get(bottom_start_x > 10 ? bottom_start_x : 10, 1, bottom_end_x < 64 ? bottom_end_x : 64 , 59);

            get_distance_line();
            get_lost_count();
            get_narrow_line();
            draw_rectan();

            element_check();
            element_count();
            element_process();

            detect_count_max = get_border_line(80);
            outbounds_detection();
            // });
            // vofa_tcp.imwrite((uint8_t *)contrast_image, 80, 60);

            // vofa_udp.printf("L_pid:%f,%f,%d,%d\n",left_speed_setpoint, right_speed_setpoint, Moto_L.speed, Moto_R.speed);

            // 动态Kp，Kd
            // if ((flag.need_sec_border && flag.right_sec_border && flag.right_border) ||
            //     (flag.need_sec_border && flag.left_sec_border && flag.left_border)){
            //     // wheel_turn_pid.Kp = Kp_max * (0.7 * (tanh(fabs((double)image_diff) / 6000)) + 0.3);
            //     // wheel_turn_pid.Kd = Kd_max * (0.6 * (tanh(fabs((double)image_diff) / 10000)) + 0.4);
            // }
            if (counter.drive_in_left_roundabout || counter.drive_in_right_roundabout) {
                wheel_turn_pid.Kp = Kp_max * 0.8;
                wheel_turn_pid.Kd = Kd_max * 0.8;
            }else if (counter.drive_in_crossroad > 800){
                wheel_turn_pid.Kp = Kp_max * 0.4;
                wheel_turn_pid.Kd = Kd_max * 0.4;
            }else{
                wheel_turn_pid.Kp = Kp_max * (0.7 * (tanh(fabs((double)image_diff) / 8000)) + 0.3);
                wheel_turn_pid.Kd = Kd_max * (0.6 * (tanh(fabs((double)image_diff) / 10000)) + 0.4);
            }

            Moto_L.update_speed();
            Moto_R.update_speed();

            // 位置环PID(需要优化)
            turn_pidout = PID_Position_Calc(&wheel_turn_pid, 0, (float) icm20948_data.gz, (float) image_diff);
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

            // vofa_udp.printf("%d,%d,%d\n",Moto_L.speed,Moto_R.speed,blind_line);
            // vofa_udp.printf("%d,%d,%d,%d\n",distances[40],distances[35], distances[30], distances[25]);
            // vofa_udp.printf("%d,%d,%d,%d,%d,%d,%d\n",max_white_column.left_x,max_white_column.right_x,max_white_column.start_y,max_white_column.end_y,distance_middle_line[0][0] - distance_middle_line[20][0],counter.drive_in_ramp, flag.found_ramp);
            // vofa_udp.printf("%d,%d,%d\n",abs(max_white_column.left_x - max_white_column.right_x),max_white_column.end_y,distance_middle_line[0][0] - distance_middle_line[20][0]);
            // vofa_udp.printf("%d,%d,%d,%d,%d,%d\n",image_diff,dis_index,distances[dis_index],left_distance[dis_index][0],right_distance[dis_index][0],detect_count_max);
            // vofa_udp.printf("%d,%d,%d,%.3f,%.3f\n",image_diff, flag.drive_in_crossroad, counter.drive_in_crossroad, wheel_turn_pid.Kp, wheel_turn_pid.Kd);
            // vofa_udp.printf("%d,%d,%d,%d,%d\n",left_lost_count,right_lost_count,abs(left_lost_count - right_lost_count),distances[25] - road_distances[25],distances[20] - road_distances[20]);
            // vofa_udp.printf("%d,%d,%d,%d,%d,%.1f\n",id,left_lost_count,right_lost_count,rstate,counter.drive_in_left_roundabout,angelZ - icm20948_data.anglez);
            // vofa_udp.printf("%d,%d,%d,%d,%d\n",dis_index,left_border[dis_index][0],right_border[dis_index][0],left_border[dis_index][1],right_border[dis_index][1]);
            // vofa_udp.printf("%d,%d\n",bottom_start_x,bottom_end_x);
            // vofa_udp.printf("%d,%d,%d,%d,%d,%d,%d,%d\n",left_lost_count,right_lost_count,max_white_column.left_height,lost_y1, left_lost_dir,right_lost_dir,left_reach_edge,right_reach_edge);
            // vofa_udp.printf("%d,%d,%d,%.2f,%d,%d\n",id,left_lost_count,right_lost_count,distance,flag.found_left_roundabout,flag.found_right_roundabout);
            // vofa_udp.printf("%f,%f,%d,%d,%.2f,%.2f\n",left_speed_setpoint,right_speed_setpoint, Moto_L.speed, Moto_R.speed,left_wheel_pidout,right_wheel_pidout);
            // vofa_udp.printf("%d,%d,%d,%d,%d,%.1f\n",id,left_lost_count,right_lost_count,rstate,counter.drive_in_right_roundabout,angelZ - icm20948_data.anglez);
            // vofa_udp.printf("%d,%d,%d,%d,%d,%d\n",lost_x1,lost_x2,lost_y1,lost_y2,x_left,x_right);
            // vofa_udp.printf("%d,%d,%d,%d,%d,%d\n",lost_x1,lost_x2,lost_y1,lost_y2,middle_line[60-lost_y1][0], middle_line[60-lost_y1][1]);
            // vofa_udp.printf("%d,%d,%d,%d\n",flag.found_garage,counter.found_garage, garage_count,detect_count_max);
            // vofa_udp.printf("%d,%d,%d,%.2f\n",image_diff,left_reach_edge,right_reach_edge,turn_angle);
            // vofa_udp.printf("%d,%d,%d,%d,%d,%d\n",lost_x1,lost_x2,lost_y1,lost_y2,left_reach_edge,right_reach_edge);
            vofa_udp.printf("%d\n",flag.stop);

            // 速度环PID
            if(flag.stop){
                speed_setpoint = 0;
                left_speed_setpoint = 0;
                right_speed_setpoint = 0;
            }
            left_wheel_pidout  = PID_Incremental_Calc(&left_wheel_speed_pid, (float32) Moto_L.speed, left_speed_setpoint);
            right_wheel_pidout = PID_Incremental_Calc(&right_wheel_speed_pid, (float32) Moto_R.speed, right_speed_setpoint);

            Moto_L.set_speed((int)left_wheel_pidout);
            Moto_R.set_speed(-(int)right_wheel_pidout);

            image_diff_process();

            if (switch1()) {
                dis_index++;
            }else if (switch2()) {
                dis_index--;
            }

            if(counter.beep_ms > 0) {
                counter.beep_ms -= 10;
                beep.beep_on();
            }else {
                beep.beep_off();
            }

            // 出界检测
            if(blind_line <= 5 && abs(bottom_start_x - bottom_end_x) <= 20) {
                counter.out_of_bound += 5;
            }else {
                flag.stop = false;
                counter.out_of_bound = 0;
            }
            if(counter.out_of_bound > 15) {
                flag.stop = true;
            }

            if (Moto_L.speed > 10 || Moto_R.speed > 10) {
                flag.start = true;
            }

            // 电机堵转或编码器异常时停止电机
            if(protect == true) {
                if ((abs(Moto_L.speed) < 10 && abs(left_speed_setpoint) > 40) ||
                    (abs(Moto_R.speed) < 10 && abs(right_speed_setpoint) > 40)) {
                        counter.stop_motor += 10;
                } else {
                    counter.stop_motor = 0;
                    }

                if (counter.stop_motor > 500) {
                    // flag.stop = true;
                }
            }

            if (running_time <= 0) {
                flag.stop = true;
            }
            if (running_time > 0 && flag.start) {
                running_time -= 10;
            }
        }
    }

OUT:
    close(timer_fd);
    close(epoll_fd);
    icm20948_delete(icm20948);
    running = false;
    return nullptr;
}

void element_count(void) {
    // 十字
    if(flag.found_crossroad == true && counter.drive_in_crossroad == 0 && counter.drive_in_ramp == 0) {
        counter.found_crossroad += 2;
        counter.found_left_roundabout = 0;
        counter.found_right_roundabout = 0;
        if(counter.found_crossroad > 2){
            beep.beep_ms(500);
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
    if (flag.found_garage == true && counter.drive_in_ramp == 0 && counter.drive_in_crossroad == 0) {
        counter.found_garage += 2;
        if (counter.found_garage > 3) {
            beep.beep_ms(400);
            counter.drive_in_garage = 1000;
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
            beep.beep_ms(200);
            counter.drive_in_ramp = 300;
        }
    }

    if(counter.found_ramp > 0){
        counter.found_ramp--;
    }

    if(counter.drive_in_ramp > 0){
        counter.drive_in_ramp -= 10;
    }

    // 左环岛计数处理
    if(flag.found_left_roundabout && counter.drive_in_left_roundabout == 0 && counter.drive_in_right_roundabout == 0 && counter.drive_in_ramp == 0) {
        counter.found_left_roundabout += 2;
        if(counter.found_left_roundabout > 3){
            beep.beep_ms(200);
            counter.drive_in_left_roundabout = 10000;
        }
    }

    if(counter.found_left_roundabout > 0){
        counter.found_left_roundabout--;
    }

    if(counter.drive_in_left_roundabout > 0){
        counter.drive_in_left_roundabout -= 10;
    }

    // 右环岛计数处理
    if(flag.found_right_roundabout && counter.drive_in_left_roundabout == 0 && counter.drive_in_right_roundabout == 0 && counter.drive_in_ramp == 0) {
        counter.found_right_roundabout += 2;
        if(counter.found_right_roundabout > 3){
            beep.beep_ms(200);
            counter.drive_in_right_roundabout = 10000;
        }
    }

    if(counter.found_right_roundabout > 0){
        counter.found_right_roundabout--;
    }

    if(counter.drive_in_right_roundabout > 0){
        counter.drive_in_right_roundabout -= 10;
    }
}

void element_process() {
    // 十字
    if (counter.drive_in_crossroad > 50) {
        if (counter.drive_in_crossroad > 50) {
            fix_crossroad();
        }
        if(flag.found_crossroad) {
            counter.drive_in_crossroad = 4000;
        } else if (distances[5] > road_distances[5] && rstate == 0 && counter.drive_in_crossroad >= 50){
            rstate = 1;
            counter.drive_in_crossroad = 800;
        } else if(distances[5] < road_distances[5] + 5 && rstate == 1){
            rstate = 0;
            counter.drive_in_crossroad = 200;
            flag.drive_in_crossroad = !flag.drive_in_crossroad;
        }
    }

    // 车库
    if (counter.drive_in_garage > 0 && stop_in_garage == true) {
        flag.stop = true;
    }

    // 坡道
    if(counter.drive_in_ramp > 0){
        if(flag.found_ramp == true && counter.drive_in_ramp > 280){
            counter.drive_in_ramp = 300;
            ramp_angle = icm20948_data.anglez;
        }
    }

    // 左环岛
    if (counter.drive_in_left_roundabout > 5001) {
        fix_left_break(0,60);
        if (rstate == 0 && left_lost_count > 15) {
            rstate = 1;
        }
        if ((rstate == 1 && id == -1 && left_lost_count < 12)){
            rstate = 0;
            counter.drive_in_left_roundabout = 5000;
            angelZ = icm20948_data.anglez;
        }
    }else if(counter.drive_in_left_roundabout > 100) {
        int end_x = 79;
        if(angelZ - icm20948_data.anglez < 60) {
            erase_top_right_road(end_x - 65, 0, end_x, 60);
        } else if(angelZ - icm20948_data.anglez < 300){
            erase_top_right_road(end_x - 65, 0, end_x, 60);
            counter.drive_in_left_roundabout = 500;
        } else if(angelZ - icm20948_data.anglez > 300){
            fix_left_break(0, 60);
            beep.beep_ms(200);
        } else if(counter.drive_in_left_roundabout > 100){
            fix_left_break(0, 60);
            if(distances[10] < road_distances[10] + 5){
                counter.drive_in_left_roundabout = 100;
            }
        }
    }

    // 右环岛
    if (counter.drive_in_right_roundabout > 5001) {
        fix_right_break(0,60);
        if (rstate == 0 && right_lost_count > 15) {
            rstate = 1;
        }
        if ((rstate == 1 && id == -1 && right_lost_count < 12)){
            rstate = 0;
            counter.drive_in_right_roundabout = 5000;
            angelZ = icm20948_data.anglez;
        }
    }else if(counter.drive_in_right_roundabout > 100) {
        int start_x = 0;
        if(angelZ - icm20948_data.anglez > -60) {
            erase_top_left_road(start_x + 65, 0, start_x, 60);
        } else if(angelZ - icm20948_data.anglez > -300) {
            erase_top_left_road(start_x + 65, 0, start_x, 60);
            counter.drive_in_right_roundabout = 500;
        } else if(angelZ - icm20948_data.anglez < -300){
            fix_right_break(0,45);
            beep.beep_ms(200);
        } else if(counter.drive_in_right_roundabout > 100) {
            fix_right_break(0, 45);
            if (distances[10] < road_distances[10] + 5) {
                counter.drive_in_right_roundabout = 100;
            }
        }
    }
}


void element_check(void) {
    check_crossroad();
    check_garage();
    // check_ramp();
    check_roundabout();
}

void image_diff_process(void) {
    if((counter.drive_in_crossroad > 400 && counter.drive_in_obstacle == 0)) {
        // 十字处理
        left_sum = 0;
        right_sum = 0;
        for(int i = 0; i < distance_middle_line_index; i++){
            left_sum -= (distance_middle_line[i][0] - IMAGE_MIDDLE);
        }
        left_sum *= 35;
        image_diff = right_sum - left_sum;
    }
    else if(counter.drive_in_ramp > 0 && counter.drive_in_ramp < 280){
        right_sum = 0;
        left_sum = -(icm20948_data.anglez - ramp_angle) * 50;
        image_diff = right_sum - left_sum;
    }else {
        left_sum = 0;
        right_sum = 0;
        int img_start = 12;
        if(img_start < incision)img_start = incision;
        int img_end = img_start + 50;
        if(img_end > detect_count_max) img_end = detect_count_max;
        for(int i = img_start; i < img_end; i++) {
            // left_sum -= (middle_line[i][0] - IMAGE_MIDDLE) * (1.5 + (i - (img_end - img_start) / 2) * 0.16);
            left_sum -= middle_line[i][0] - IMAGE_MIDDLE;
        }
        left_sum *= 10;
        // left_sum += 4000 * cornering;
        image_diff = right_sum - left_sum;
    }

    if ((flag.need_sec_border && flag.right_sec_border && flag.right_border) ||
    (flag.need_sec_border && flag.left_sec_border && flag.left_border)) {
        if (image_diff < 0) {
            image_diff -= left_reach_edge * 60;
        }else {
            image_diff += right_reach_edge * 60;
        }
    }else if (left_reach_edge > 25 || right_reach_edge > 25) {
        if (image_diff < 0) {
            image_diff -= left_reach_edge * 20;
        }else {
            image_diff += right_reach_edge * 20;
        }
    }
}

// 非实时任务线程函数
void *non_realtime_task(void *arg) {
    auto atag = mytag("tag36h11", 1.5, 0, 1, false, false);
    cv::Mat gray;
    cv::Mat gray1ch(60, 80, CV_8UC1, (void*)gray1ch_image);
    cv::Mat gray3ch;
    cv::Mat cv_image(60, 40, CV_8UC1, gray_pers_image); // 60行60列的灰度图
    std::vector<uchar> jpg;
    int iii=0;
    while (running) {
            // fprintf(stdout,"%d\n",iii++);
            frame.copyTo(gray);
            // MEASURE_TIME("non rt task", {
                memcpy(gray1ch_image,gray_image, 80 * 60);
                // memcpy(gray1ch_image, binary_image, 80 * 60);
                cv::cvtColor(gray1ch, gray3ch, cv::COLOR_GRAY2BGR);
            // });
            MEASURE_TIME("detect_time", {
                atag.detect(gray);
            // });
            // MEASURE_TIME("getclosettagindex", {
                atag.getClosetTagIndex();
            // });
            // MEASURE_TIME("draw", {
                atag.draw(gray3ch, 0.25);
            // });detect_count_max:
            // MEASURE_TIME("getid", {
                id = atag.getClosetTagID();
            // });
            // MEASURE_TIME("getdistance", {
                distance = atag.getClosetTagDistance(1500);
                // cv::putText(gray3ch, std::to_string(distance), cv::Point(0, 20), cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 0xff, 0), 2);
            // });
                tft180_draw_real_border_line(gray3ch, 0, 0, left_border, cv::Scalar(0, 0xff, 0));
                tft180_draw_real_border_line(gray3ch, 0, 0, right_border, cv::Scalar(0xff, 0, 0));
                tft180_draw_real_border_line(gray3ch, 0, 0, middle_line, cv::Scalar(0, 0, 0xff));
                tft180_draw_real_border_line(gray3ch, 0, 0, distance_middle_line, cv::Scalar(0, 0, 0));
            // MEASURE_TIME("http write", {
                // vofa_tcp.imwrite(gray3ch);

                vofa_tcp.imwrite(cv_image);
                // http << gray3ch;
            });
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

    system("v4l2-ctl -d /dev/video0 -c contrast=36 -c gamma=80 -c exposure_auto=1 -c exposure_absolute=90 -c sharpness=55");

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