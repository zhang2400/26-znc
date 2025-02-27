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

class Transport {
public:
    virtual ~Transport() = default;
    virtual bool t_send(const void* data, size_t length) = 0;
    virtual bool t_send(const std::vector<unsigned char>& data) = 0;
};

class UDPTransport : public Transport {
public:
    UDPTransport(const char* ip, int port);
    ~UDPTransport() override;

    bool t_send(const void* data, size_t length) override;
    bool t_send(const std::vector<unsigned char>& data) override;

private:
    int sockfd = -1;
    struct sockaddr_in server_addr{};
};

class TCPTransport : public Transport {
public:
    TCPTransport(const char* bind_ip, int port);
    ~TCPTransport() override;

    bool t_send(const void* data, size_t length) override;
    bool t_send(const std::vector<unsigned char>& data) override;

private:
    void accept_loop();
    bool send_all(int sockfd, const void* data, size_t length);

    bool running_{true};
    std::thread accept_thread_;

    int listen_sockfd_ = -1;
    int client_sockfd_ = -1;
    std::mutex sock_mutex_;

    struct sockaddr_in server_addr_{};
};

class VOFA {
public:
    // 通过传输层构造
    explicit VOFA(std::unique_ptr<Transport> transport);

    void printf(const char* format, ...);
    void imwrite(const std::vector<uchar>& jpg);
    void imwrite(const cv::Mat& frame);

    // 操作符重载保持原有设计
    VOFA& operator<<(const cv::Mat& frame);
    VOFA& operator<<(const std::vector<uchar>& jpg);
    VOFA& operator<<(const std::string& str);

private:
    std::unique_ptr<Transport> transport_;

    void send_image_header(unsigned int img_size,
                          int width, int height,
                          int format);
};


#endif //CV_VOFA_H
