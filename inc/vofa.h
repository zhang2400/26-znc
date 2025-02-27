//
// Created by ashkore on 25-2-8.
//

#ifndef CV_VOFA_H
#define CV_VOFA_H

#include <thread>
#include <opencv2/opencv.hpp>
#include <netinet/in.h>

enum class VOFAImgFormat {
    Format_Invalid,
    Format_Mono,
    Format_MonoLSB,
    Format_Indexed8,
    Format_RGB32,
    Format_ARGB32,
    Format_ARGB32_Premultiplied,
    Format_RGB16,
    Format_ARGB8565_Premultiplied,
    Format_RGB666,
    Format_ARGB6666_Premultiplied,
    Format_RGB555,
    Format_ARGB8555_Premultiplied,
    Format_RGB888,
    Format_RGB444,
    Format_ARGB4444_Premultiplied,
    Format_RGBX8888,
    Format_RGBA8888,
    Format_RGBA8888_Premultiplied,
    Format_BGR30,
    Format_A2BGR30_Premultiplied,
    Format_RGB30,
    Format_A2RGB30_Premultiplied,
    Format_Alpha8,
    Format_Grayscale8,

    // 以下格式发送时，IMG_WIDTH和IMG_HEIGHT不需要强制指定，设置为-1即可
    Format_BMP,
    Format_GIF,
    Format_JPG,
    Format_PNG,
    Format_PBM,
    Format_PGM,
    Format_PPM,
    Format_XBM,
    Format_XPM,
    Format_SVG,
};

class VOFA {
public:
    VOFA(const char *vofa_ip, int vofa_port);
    ~VOFA();
    void imwrite(const cv::Mat &frame);
    void imwrite(const std::vector<uchar> &jpg);
    void printf(const char* format, ...);
    VOFA& operator<<(const cv::Mat& frame);
    VOFA& operator<<(const std::vector<uchar> &jpg);
    VOFA& operator<<(const std::string& str);

private:
    std::mutex client_mutex;
    void accept_loop();
    static ssize_t sendAll(int sockfd, const void* buffer, size_t length);
    int sockfd;
    struct sockaddr_in server_addr = {};
    std::string vofa_ip;
    bool running;
    int client_sockfd;
    std::thread accept_thread;

};


#endif //CV_VOFA_H
