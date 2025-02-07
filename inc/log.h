//
// Created by ashkore on 25-2-1.
//

#ifndef CV_LOG_H
#define CV_LOG_H
#include <atomic>
#include <vector>
#include <string>
#include <thread>
#include <chrono>
#include <fstream>
#include <sys/time.h>
#include <cstdarg>
#include <memory>
// 日志级别定义
enum LogLevel {
    LOG_ERROR = 1,
    LOG_WARN  = 2,
    LOG_INFO  = 3,
    LOG_DEBUG = 4
};

// 编译时日志级别过滤 (通过-DLOG_LEVEL=4 设置)
#ifndef LOG_LEVEL
#define LOG_LEVEL LOG_INFO
#endif

// 日志消息结构
struct LogMessage {
    struct timeval timestamp;
    LogLevel level;
    std::string tag;
    std::string content;
};

// 无锁环形缓冲区
class LogRingBuffer {
public:
    explicit LogRingBuffer(size_t size)
        : buffer_(size), capacity_(size), head_(0), tail_(0) {}

    bool push(const LogMessage& msg);

    bool pop(LogMessage& msg);

    void drain(std::vector<LogMessage>& output);

private:
    std::vector<LogMessage> buffer_;
    const size_t capacity_;
    std::atomic<size_t> head_;
    std::atomic<size_t> tail_;
};
void log_consumer_thread();
void signal_handler(int sig);
bool log_init(const std::string& path);
void log_shutdown();
void log_write(LogLevel level, const char* tag, const char* format, ...);
// 日志宏定义
#define LOGE(tag, format, ...) log_write(LOG_ERROR, tag, format, ##__VA_ARGS__)
#define LOGW(tag, format, ...)  log_write(LOG_WARN,  tag, format, ##__VA_ARGS__)
#define LOGI(tag, format, ...)  log_write(LOG_INFO,  tag, format, ##__VA_ARGS__)
#define LOGD(tag, format, ...) log_write(LOG_DEBUG, tag, format, ##__VA_ARGS__)

#define MEASURE_TIME(TAG, code_block) \
    do { \
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now(); \
    code_block \
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now(); \
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end - start); \
    LOGI(TAG, "Time used: %.2lfms", time_used.count() * 1000); \
    } while (0)

#define MEASURE_TIME_TO_VAR(duration_var, code_block) \
    do { \
    std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now(); \
    code_block \
    std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now(); \
    std::chrono::duration<double> time_used = std::chrono::duration_cast<std::chrono::duration<double>>(end - start); \
    duration_var = time_used.count(); \
    } while (0)



#endif //CV_LOG_H
