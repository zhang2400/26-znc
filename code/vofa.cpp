//
// Created by ashkore on 25-2-8.
//

#include "vofa.h"
#include <iostream>
#include <cstring>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <unistd.h>

VOFA::VOFA(const char *vofa_ip, int vofa_port){
    // 创建UDP socket
    sockfd = socket(AF_INET, SOCK_DGRAM, 0);
    if (sockfd < 0) {
        std::cerr << "Socket creation failed: " << strerror(errno) << std::endl;
    }

    // 配置服务器地址
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(vofa_port);
    if (inet_pton(AF_INET, vofa_ip, &server_addr.sin_addr) <= 0) {
        std::cerr << "Invalid address: " << strerror(errno) << std::endl;
        close(sockfd);
    }
}

VOFA::~VOFA() {
    close(sockfd);
}

void VOFA::printf(const char *format, ...) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    sendto(sockfd, buffer, strlen(buffer), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));
}

void VOFA::imwrite(const std::vector<uchar> &jpg) {
    char image_header[128];
    unsigned int img_size = jpg.size();
    int img_id = 0;
    snprintf(image_header, sizeof(image_header),
             "\nimage:%d,%u,%d,%d,%d\n",
             img_id, img_size, -1, -1, static_cast<int>(VOFAImgFormat::Format_JPG));
    ssize_t sent_bytes = sendto(sockfd, image_header, strlen(image_header), 0,
                                (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (sent_bytes < 0) {
        std::cerr << "Failed to send image header: " << strerror(errno) << std::endl;
        return;
    }
    sent_bytes = sendto(sockfd, jpg.data(), img_size, 0,
                        (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (sent_bytes < 0) {
        std::cerr << "Failed to send image data: " << strerror(errno) << std::endl;
    } else if (sent_bytes != img_size) {
        std::cerr << "Partial data sent: " << sent_bytes << "/" << img_size << std::endl;
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
    ssize_t sent_bytes = sendto(sockfd, image_header, strlen(image_header), 0,
                                (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (sent_bytes < 0) {
        std::cerr << "Failed to send image header: " << strerror(errno) << std::endl;
        return;
    }
    sent_bytes = sendto(sockfd, frame.data, img_size, 0,
                        (struct sockaddr*)&server_addr, sizeof(server_addr));
    if (sent_bytes < 0) {
        std::cerr << "Failed to send image data: " << strerror(errno) << std::endl;
    } else if (sent_bytes != img_size) {
        std::cerr << "Partial data sent: " << sent_bytes << "/" << img_size << std::endl;
    }
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
    sendto(sockfd, str.c_str(), str.size(), 0, (struct sockaddr*)&server_addr, sizeof(server_addr));
    return *this;
}
