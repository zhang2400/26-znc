/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-10 15:02:00
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2024-12-01 13:27:38
 * @FilePath: /ls2k0300_peripheral_library/lib/GPIO.h
 * @Description: GPIO类
 *
 * Copyright (c) 2024 by ilikara 3435193369@qq.com, All Rights Reserved.
 */

#ifndef GPIO_H
#define GPIO_H

#include <string>
#include <fcntl.h>
#include <unistd.h>
#include <stdexcept>
#include <iostream>
#include <fstream>
#include <sstream>
#include <cstring>

enum GPIO_pin {
    GPIO20 = 20,
    GPIO22 = 22,
    GPIO24 = 24,
    GPIO41 = 41,
    GPIO42 = 42,
    GPIO43 = 43,
    GPIO44 = 44,
    GPIO45 = 45,
    GPIO50 = 50,
    GPIO51 = 51,
    GPIO60 = 60,
    GPIO61 = 61,
    GPIO62 = 62,
    GPIO63 = 63,
    GPIO72 = 72,
    GPIO73 = 73,
    GPIO74 = 74,
    GPIO75 = 75,
};

class GPIO
{
public:
    explicit GPIO(int gpioNum_);
    ~GPIO();

    int getFileDescriptor() const;               // 获取 GPIO 文件描述符
    bool setDirection(const std::string &direction); // 设置GPIO方向，out为输出，in为输入
    bool setValue(bool value);                       // 设置 GPIO 输出值
    bool readValue();                            // 读取 GPIO 输入值
    bool setEdge(const std::string &edge);

private:
    int gpioNum;
    int fd;
    std::string gpioPath;

    bool writeToFile(const std::string &path, const std::string &value);
};
#endif // GPIO_H
