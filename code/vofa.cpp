#include "vofa.h"
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <mutex>

VOFA::VOFA(const char *vofa_ip, int vofa_port) : running(true), client_sockfd(-1) {
    // 创建TCP socket
    sockfd = socket(AF_INET, SOCK_STREAM, 0);
    if (sockfd < 0) {
        std::cerr << "Socket creation failed: " << strerror(errno) << std::endl;
        return;
    }

    // 设置SO_REUSEADDR
    int opt = 1;
    setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 配置服务器地址
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(vofa_port);
    if (inet_pton(AF_INET, vofa_ip, &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address: " << strerror(errno) << std::endl;
        close(sockfd);
        return;
    }

    // 绑定
    if (bind(sockfd, (struct sockaddr*)&server_addr, sizeof(server_addr)) < 0) {
        std::cerr << "Bind failed: " << strerror(errno) << std::endl;
        close(sockfd);
        return;
    }

    // 监听
    if (listen(sockfd, 5) < 0) {
        std::cerr << "Listen failed: " << strerror(errno) << std::endl;
        close(sockfd);
        return;
    }

    // 启动接受线程
    accept_thread = std::thread(&VOFA::accept_loop, this);
}

VOFA::~VOFA() {
    running = false;
    // 关闭监听socket使accept线程退出
    close(sockfd);
    if (accept_thread.joinable()) {
        accept_thread.join();
    }
    // 关闭客户端socket
    std::lock_guard<std::mutex> lock(client_mutex);
    if (client_sockfd != -1) {
        close(client_sockfd);
    }
}

void VOFA::accept_loop() {
    while (running) {
        struct sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int new_client = accept(sockfd, (struct sockaddr*)&client_addr, &client_len);
        if (new_client < 0) {
            if (running) {
                std::cerr << "Accept failed: " << strerror(errno) << std::endl;
            }
            continue;
        }

        std::lock_guard<std::mutex> lock(client_mutex);
        // 关闭旧连接
        if (client_sockfd != -1) {
            close(client_sockfd);
        }
        client_sockfd = new_client;

        // 设置TCP_NODELAY避免缓冲延迟
        int flag = 1;
        setsockopt(client_sockfd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
    }
}

void VOFA::printf(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    std::lock_guard<std::mutex> lock(client_mutex);
    if (client_sockfd < 0) {
        return; // 无连接
    }

    size_t len = strlen(buffer);
    ssize_t sent = send(client_sockfd, buffer, len, MSG_NOSIGNAL);
    if (sent < 0) {
        std::cerr << "Send failed: " << strerror(errno) << std::endl;
        close(client_sockfd);
        client_sockfd = -1;
    } else if (sent != len) {
        std::cerr << "Partial data sent: " << sent << "/" << len << std::endl;
    }
}

// imwrite和其他函数需类似修改，用send替换sendto，并处理部分发送
void VOFA::imwrite(const std::vector<uchar> &jpg) {
    std::lock_guard<std::mutex> lock(client_mutex);
    if (client_sockfd < 0) return;

    char image_header[128];
    unsigned int img_size = jpg.size();
    int img_id = 0;
    snprintf(image_header, sizeof(image_header),
             "\nimage:%d,%u,%d,%d,%d\n",
             img_id, img_size, -1, -1, static_cast<int>(VOFAImgFormat::Format_JPG));
    
    // 发送header
    size_t header_len = strlen(image_header);
    ssize_t sent = sendAll(client_sockfd, image_header, header_len);
    if (sent < 0) {
        close(client_sockfd);
        client_sockfd = -1;
        return;
    }

    // 发送数据
    sent = sendAll(client_sockfd, jpg.data(), img_size);
    if (sent < 0) {
        close(client_sockfd);
        client_sockfd = -1;
    }
}

void VOFA::imwrite(const cv::Mat &frame) {
    int img_format;
    switch (frame.type()) {
        case CV_8UC1:
            img_format = static_cast<int>(VOFAImgFormat::Format_Grayscale8);
        break;
        case CV_8UC3:
            img_format = static_cast<int>(VOFAImgFormat::Format_RGB888);
        break;
        case CV_8UC4:
            img_format = static_cast<int>(VOFAImgFormat::Format_RGBA8888);
        break;
        default:
            std::cerr << "Unsupported image format" << std::endl;
        return;
    }
    char image_header[128];
    unsigned int img_size = frame.total() * frame.elemSize(); // 计算字节数
    int img_id = 0;
    int img_width = frame.cols;
    int img_height = frame.rows;
    snprintf(image_header, sizeof(image_header),
             "\nimage:%d,%u,%d,%d,%d\n",
             img_id, img_size, img_width, img_height, img_format);

    // 发送header
    size_t header_len = strlen(image_header);
    ssize_t sent = sendAll(client_sockfd, image_header, header_len);
    if (sent < 0) {
        close(client_sockfd);
        client_sockfd = -1;
        return;
    }

    // 发送数据
    sent = sendAll(client_sockfd, frame.data, img_size);
    if (sent < 0) {
        close(client_sockfd);
        client_sockfd = -1;
    }

}

// 辅助函数确保发送完整数据
ssize_t VOFA::sendAll(int sockfd, const void* buffer, size_t length) {
    size_t sent_total = 0;
    const char* ptr = static_cast<const char*>(buffer);
    while (sent_total < length) {
        ssize_t sent = send(sockfd, ptr + sent_total, length - sent_total, MSG_NOSIGNAL);
        if (sent < 0) {
            return -1; // 错误
        }
        sent_total += sent;
    }
    return (ssize_t)sent_total;
}

// 其他成员函数类似修改，使用sendAll发送数据