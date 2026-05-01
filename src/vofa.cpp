#include "vofa.h"
#include <iostream>
#include <cstring>
#include <algorithm>
#include <sys/socket.h>
#include <fcntl.h>
#include <netinet/tcp.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <thread>
#include <mutex>

UDPTransport::UDPTransport(const char* ip, int port) {
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Socket creation failed: " << strerror(errno) << std::endl;
        return;
    }

    // 设置非阻塞模式，发不出去就丢帧，绝不阻塞
    int flags = fcntl(sockfd, F_GETFL, 0);
    fcntl(sockfd, F_SETFL, flags | O_NONBLOCK);

    // 增大发送缓冲区到512KB，可缓存数帧图像数据
    int sndbuf = 512 * 1024;
    setsockopt(sockfd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));

    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, ip, &server_addr.sin_addr) <= 0) {
        close(sockfd);
        sockfd = -1;
        std::cerr << "Invalid address: " << strerror(errno) << std::endl;
    }
}

UDPTransport::~UDPTransport() {
    if (sockfd != -1) close(sockfd);
}

bool UDPTransport::t_send(const void* data, size_t length) {
    if (sockfd < 0) return false;

    const uint8_t* buf = static_cast<const uint8_t*>(data);
    size_t offset = 0;

    // 分片发送：每片1400字节，低于MTU(1500)避免IP层分片
    // 这样单个UDP包丢失不会导致其他包也丢失
    while (offset < length) {
        size_t chunk = std::min(length - offset, static_cast<size_t>(1400));
        ssize_t sent = sendto(sockfd, buf + offset, chunk, MSG_DONTWAIT,
                              (struct sockaddr*)&server_addr, sizeof(server_addr));
        if (sent < 0) {
            // EAGAIN/EWOULDBLOCK = 内核缓冲区满，丢弃本帧
            // 其他错误也直接丢弃，调试图传不需要可靠性
            return false;
        }
        offset += sent;
    }
    return true;
}

bool UDPTransport::t_send(const std::vector<unsigned char>& data) {
    return t_send(data.data(), data.size());
}

TCPTransport::TCPTransport(const char* bind_ip, int port) {
    // 创建TCP socket
    listen_sockfd_ = socket(AF_INET, SOCK_STREAM, 0);
    if (listen_sockfd_ < 0) {
        std::cerr << "Socket creation failed: " << strerror(errno) << std::endl;
        return;
    }

    // 设置SO_REUSEADDR
    int opt = 1;
    setsockopt(listen_sockfd_, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    // 配置服务器地址
    server_addr_.sin_family = AF_INET;
    server_addr_.sin_port = htons(port);
    if (inet_pton(AF_INET, bind_ip, &server_addr_.sin_addr) <= 0) {
        std::cerr << "Invalid address: " << strerror(errno) << std::endl;
        close(listen_sockfd_);
        return;
    }

    // 绑定
    if (bind(listen_sockfd_, (struct sockaddr*)&server_addr_, sizeof(server_addr_)) < 0) {
        std::cerr << "Bind failed: " << strerror(errno) << std::endl;
        close(listen_sockfd_);
        return;
    }

    // 监听
    if (listen(listen_sockfd_, 5) < 0) {
        std::cerr << "Listen failed: " << strerror(errno) << std::endl;
        close(listen_sockfd_);
        return;
    }

    // 启动接受线程
    accept_thread_ = std::thread(&TCPTransport::accept_loop, this);
}

TCPTransport::~TCPTransport() {
    running_ = false;

    // 关闭监听socket使接受线程退出
    close(listen_sockfd_);

    if (accept_thread_.joinable()) {
        accept_thread_.join();
    }

    // 关闭客户端socket
    std::lock_guard<std::mutex> lock(sock_mutex_);
    if (client_sockfd_ != -1) {
        close(client_sockfd_);
    }
}

void TCPTransport::accept_loop() {
    while (running_) {
        struct sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);

        int new_sock = accept(listen_sockfd_, (struct sockaddr*)&client_addr, &client_len);
        if (new_sock < 0) {
            if (running_) {
                std::cerr << "Accept error: " << strerror(errno) << std::endl;
            }
            continue;
        }

        std::lock_guard<std::mutex> lock(sock_mutex_);
        // 关闭旧连接
        if (client_sockfd_ != -1) {
            close(client_sockfd_);
        }
        client_sockfd_ = new_sock;

        // 设置TCP_NODELAY
        int flag = 1;
        setsockopt(client_sockfd_, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

        std::cout << "New client connected: " << inet_ntoa(client_addr.sin_addr) << std::endl;
    }
}

bool TCPTransport::t_send(const void* data, size_t length) {
    std::lock_guard<std::mutex> lock(sock_mutex_);

    if (client_sockfd_ == -1) {
        return false; // 无客户端连接
    }

    return send_all(client_sockfd_, data, length);
}

bool TCPTransport::t_send(const std::vector<unsigned char>& data) {
    return t_send(data.data(), data.size());
}

bool TCPTransport::send_all(int sockfd, const void* data, size_t length) {
    const char* buffer = static_cast<const char*>(data);
    size_t total_sent = 0;

    while (total_sent < length) {
        ssize_t sent = send(sockfd, buffer + total_sent, length - total_sent, MSG_NOSIGNAL); // 防止SIGPIPE

        if (sent < 0) {
            if (errno == EPIPE) { // 连接断开
                std::cerr << "Connection broken, resetting client" << std::endl;
                close(sockfd);
                client_sockfd_ = -1;
            }
            return false;
        }

        total_sent += sent;
    }

    return true;
}

VOFA::VOFA(std::unique_ptr<Transport> transport)
    : transport_(std::move(transport)) {}

void VOFA::printf(const char* format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    transport_->t_send(buffer, strlen(buffer));
}

void VOFA::send_image_header(unsigned int img_size, int width, int height, int format) {
    char header[128];
    snprintf(header, sizeof(header),
            "\nimage:0,%u,%d,%d,%d\n",
            img_size, width, height, format);
    transport_->t_send(header, strlen(header));
}

void VOFA::imwrite(const std::vector<uchar>& jpg) {
    send_image_header(jpg.size(), -1, -1, static_cast<int>(VOFAImgFormat::Format_JPG));
    transport_->t_send(jpg);
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
    send_image_header(frame.total() * frame.elemSize(), frame.cols, frame.rows, img_format);
    transport_->t_send(frame.data, frame.total() * frame.elemSize());
}

void VOFA::imwrite(const uint8_t *image, int width, int height) {
    send_image_header(width * height, width, height, static_cast<int>(VOFAImgFormat::Format_Grayscale8));
    transport_->t_send(image, width * height);
}

VOFA &VOFA::operator<<(const cv::Mat &frame) {
    imwrite(frame);
    return *this;
}

VOFA &VOFA::operator<<(const std::vector<uchar> &jpg) {
    imwrite(jpg);
    return *this;
}


VOFA &VOFA::operator<<(const std::string &str) {
    transport_->t_send(str.c_str(), str.size());
    return *this;
}
