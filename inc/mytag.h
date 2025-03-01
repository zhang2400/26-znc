//
// Created by ashkore on 25-1-13.
//

#ifndef MYTAG_H
#define MYTAG_H

#include <opencv2/opencv.hpp>
#include <apriltag/apriltag.h>
#include <apriltag/tag36h11.h>
#include <apriltag/tag36h10.h>
#include <apriltag/tag25h9.h>
#include <apriltag/tag16h5.h>
#include <apriltag/tagCircle21h7.h>
#include <apriltag/tagCircle49h12.h>
#include <apriltag/tagCustom48h12.h>
#include <apriltag/tagStandard41h12.h>
#include <apriltag/tagStandard52h13.h>


class mytag {
public:
    /**
     * @brief mytag类的构造函数，用于初始化AprilTag检测器
     *
     * @param tagFamily 使用的标签家族名称
     * @param decimate 降采样系数，默认1，越小精度越高，检测距离越远，速度越慢；越大精度越低，检测距离越近，速度越快，可能会出现误检
     * @param sigma 高斯模糊系数，默认0，越大抗噪越高，速度越慢；越小抗噪越低，速度越快
     * @param threads 线程数，默认1，越大速度越快，2核以上没区别，单核CPU开到2以上反而会变慢
     * @param debug 调试模式，默认false，会输出检测过程的一系列图片
     * @param refine 边缘细化，默认false，给码的四个点定位更准
     */
    mytag(const std::string& tagFamily, float decimate, float sigma, int threads, bool debug, bool refine);

    /**
     * @brief mytag类的析构函数，用于释放AprilTag检测器和标签家族资源
     */
    ~mytag();

    /**
     * @brief 检测图像中的AprilTag标签
     *
     * @param gray 输入的灰度图像
     */
    void detect(const cv::Mat& gray);

    /**
     * @brief 在图像上绘制检测到的AprilTag标签
     *
     * @param frame 输入的图像帧
     */
    void draw(cv::Mat &frame);

    /**
     * @brief 在图像上绘制检测到的AprilTag标签
     *
     * @param frame 输入的图像帧
     * @param zoom 坐标缩放系数
     */
    void draw(cv::Mat &frame, double zoom);

    /**
     * @brief 清理检测结果
     */
    void clean();

    /**
     * @brief 获取距离最近的标签的索引
     *
     * @return int 距离最近的标签的索引，如果没有检测到标签则返回-1
     */
    int getClosetTagIndex();

    /**
     * @brief 获取距离最近的标签的ID
     *
     * @return int 距离最近的标签的ID，如果没有检测到标签则返回-1
     */
    int getClosetTagID();

    /**
     * @brief 获取距离最近的标签的距离
     *
     * @param t 距离系数，自行测量实际距离调整t
     * @return double 距离最近的标签的距离，如果没有检测到标签则返回-1
     */
    double getClosetTagDistance(double t);

private:
    std::string tagFamily;  // 标签家族名称
    apriltag_detector_t *td;  // AprilTag检测器
    apriltag_family_t *tf;  // AprilTag标签家族
    zarray_t *detections;  // 检测结果
    int closestTagIndex;  // 距离最近的标签的索引
};

#endif //MYTAG_H
