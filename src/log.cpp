//
// Created by ashkore on 25-2-1.
//
#include "log.h"

bool LogRingBuffer::push(const LogMessage& msg) {
    size_t current_head = head_.load(std::memory_order_relaxed);
    size_t next_head = (current_head + 1) % capacity_;

    if(next_head == tail_.load(std::memory_order_acquire)) {
        return false; // 缓冲区满
    }

    buffer_[current_head] = msg;
    head_.store(next_head, std::memory_order_release);
    return true;
}

bool LogRingBuffer::pop(LogMessage& msg) {
    size_t current_tail = tail_.load(std::memory_order_relaxed);
    if(current_tail == head_.load(std::memory_order_acquire)) {
        return false; // 缓冲区空
    }

    msg = buffer_[current_tail];
    tail_.store((current_tail + 1) % capacity_, std::memory_order_release);
    return true;
}

void LogRingBuffer::drain(std::vector<LogMessage>& output) {
    LogMessage msg;
    while(pop(msg)) {
        output.push_back(std::move(msg));
    }
}

// 全局日志组件
namespace {
    constexpr size_t LOG_BUFFER_SIZE = 4096;
    LogRingBuffer g_log_buffer(LOG_BUFFER_SIZE);
    std::atomic<bool> g_log_running{false};
    std::unique_ptr<std::ofstream> g_log_file;
    std::string g_log_path = "cv.log"; // 默认日志路径

}

// 获取当前时间字符串
static std::string get_time_str(const timeval& tv) {
    struct tm tm_time;
    localtime_r(&tv.tv_sec, &tm_time);

    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%02d:%02d:%02d.%03ld",
             tm_time.tm_hour, tm_time.tm_min, tm_time.tm_sec,
             tv.tv_usec / 1000);
    return buffer;
}

// 日志消费者线程
void log_consumer_thread() {
    std::vector<LogMessage> batch;
    batch.reserve(LOG_BUFFER_SIZE);

    while (g_log_running || !batch.empty()) {
        batch.clear();
        g_log_buffer.drain(batch); // 使用drain批量抽取

        if(g_log_file && g_log_file->is_open()) {
            for(const auto& msg : batch) {
                std::string level_str;
                switch(msg.level) {
                    case LOG_ERROR: level_str = "E"; break;
                    case LOG_WARN:  level_str = "W"; break;
                    case LOG_INFO:  level_str = "I"; break;
                    case LOG_DEBUG: level_str = "D"; break;
                }

                *g_log_file << level_str << " [" << get_time_str(msg.timestamp) << "] "
                            << msg.tag << ": " << msg.content << "\n";
            }
            g_log_file->flush(); // 定期刷新
        }

        if (!g_log_running && batch.empty()) {
            break;
        }
        if(batch.empty()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }
}

// 日志系统初始化
bool log_init(const std::string& path) {

    // 打开日志文件
    g_log_file = std::make_unique<std::ofstream>();
    g_log_path = path;
#ifdef OVERWRITE_LOG
    std::ios::openmode mode = std::ios::out | std::ios::trunc; // 覆盖模式
#else
    std::ios::openmode mode = std::ios::out | std::ios::app;   // 追加模式
#endif
    g_log_file->open(g_log_path, mode);

    if(!g_log_file->is_open()) {
        fprintf(stderr, "Failed to open log file: %s\n", g_log_path.c_str());
        return false;
    }

    // 启动消费者线程
    g_log_running = true;
    std::thread(log_consumer_thread).detach();
    return true;
}

// 日志系统关闭
void log_shutdown() {
    g_log_running = false;
    if(g_log_file) {
        g_log_file->close();
    }
}

// 核心日志记录函数
void log_write(LogLevel level, const char* tag, const char* format, ...) {
    // 级别过滤
    if(level > LOG_LEVEL) return;

    LogMessage msg;
    gettimeofday(&msg.timestamp, nullptr);
    msg.level = level;
    msg.tag = tag;

    // 格式化日志内容
    va_list args;
    va_start(args, format);
    char content_buffer[256];
    vsnprintf(content_buffer, sizeof(content_buffer), format, args);
    va_end(args);

    msg.content = content_buffer;

    // 推送到缓冲区
    if(!g_log_buffer.push(msg)) {
        // 缓冲区满时的降级处理
        fprintf(stderr, "Log buffer full! Dropping message: %s\n", content_buffer);
    }
}