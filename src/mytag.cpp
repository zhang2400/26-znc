//
// Created by ashkore on 25-1-13.
//

#include "mytag.h"

cv::Mat matd_to_cvmat(const matd_t* mat) {
    // 创建一个cv::Mat对象，行数为nrows，列数为ncols，数据类型为CV_64F（64位浮动类型）
    return {static_cast<int>(mat->nrows), static_cast<int>(mat->ncols), CV_64F, (void*)mat->data};
}

mytag::mytag(const char *tagFamily, float decimate, float sigma, int threads, bool debug, bool refine) {
    // 创建AprilTag检测器
    td = apriltag_detector_create();
    if (strcmp(tagFamily, "tag36h11") == 0) {
        tf = tag36h11_create();
    } else if (strcmp(tagFamily, "tag36h10") == 0) {
        tf = tag36h10_create();
    } else if (strcmp(tagFamily, "tag25h9") == 0) {
        tf = tag25h9_create();
    } else if (strcmp(tagFamily, "tag16h5") == 0) {
        tf = tag16h5_create();
    } else if (strcmp(tagFamily, "tagCircle21h7") == 0) {
        tf = tagCircle21h7_create();
    } else if (strcmp(tagFamily, "tagCircle49h12") == 0) {
        tf = tagCircle49h12_create();
    } else if (strcmp(tagFamily, "tagCustom48h12") == 0) {
        tf = tagCustom48h12_create();
    } else if (strcmp(tagFamily, "tagStandard41h12") == 0) {
        tf = tagStandard41h12_create();
    } else if (strcmp(tagFamily, "tagStandard52h13") == 0) {
        tf = tagStandard52h13_create();
    } else {
        std::cerr << "Error: Invalid tag family" << std::endl;
        return;
    }
    apriltag_detector_add_family(td, tf);

    // 设置参数
    td->quad_decimate = decimate;  // 降采样，默认1，越小精度越高，速度越慢;越大精度越低，速度越快，会出现误检
    td->quad_sigma = sigma;  // 高斯模糊，默认0.5，越大抗噪越高，速度越慢;越小抗噪越低，速度越快
    td->nthreads = threads;  // 线程数，默认1，越大速度越快，2核以上没区别
    td->debug = debug;  // 调试模式，默认false，具体看官网介绍
    td->refine_edges = refine;  // 优化边缘，默认false，如果标签边缘不清晰，可以设置为true

    detections = nullptr;

}

mytag::~mytag() {
    apriltag_detector_destroy(td);
    if (strcmp(tf->name, "tag36h11") == 0) {
        tag36h11_destroy(tf);
    } else if (strcmp(tf->name, "tag36h10") == 0) {
        tag36h10_destroy(tf);
    } else if (strcmp(tf->name, "tag25h9") == 0) {
        tag25h9_destroy(tf);
    } else if (strcmp(tf->name, "tag16h5") == 0) {
        tag16h5_destroy(tf);
    } else if (strcmp(tf->name, "tagCircle21h7") == 0) {
        tagCircle21h7_destroy(tf);
    } else if (strcmp(tf->name, "tagCircle49h12") == 0) {
        tagCircle49h12_destroy(tf);
    } else if (strcmp(tf->name, "tagCustom48h12") == 0) {
        tagCustom48h12_destroy(tf);
    } else if (strcmp(tf->name, "tagStandard41h12") == 0) {
        tagStandard41h12_destroy(tf);
    } else if (strcmp(tf->name, "tagStandard52h13") == 0) {
        tagStandard52h13_destroy(tf);
    }
}

void mytag::detect(cv::Mat &gray) {
    // 将OpenCV图像转换为AprilTag所需的格式
    image_u8_t im = {
        .width = gray.cols,
        .height = gray.rows,
        .stride = gray.cols,
        .buf = gray.data
    };

    // 使用AprilTag检测标签
    detections = apriltag_detector_detect(td, &im);
}

void mytag::draw(cv::Mat &frame) {
    // 绘制检测到的标签
    for (int i = 0; i < zarray_size(detections); i++) {
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        // 在框架上绘制检测到的标签
        line(frame,cv::Point(det->p[0][0], det->p[0][1]),
                 cv::Point(det->p[1][0], det->p[1][1]), cv::Scalar(0, 0xff, 0), 2);
        line(frame, cv::Point(det->p[1][0], det->p[1][1]),
                 cv::Point(det->p[2][0], det->p[2][1]), cv::Scalar(0, 0xff, 0), 2);
        line(frame,cv::Point(det->p[2][0], det->p[2][1]),
                 cv::Point(det->p[3][0], det->p[3][1]), cv::Scalar(0, 0xff, 0), 2);
        line(frame, cv::Point(det->p[3][0], det->p[3][1]),
                 cv::Point(det->p[0][0], det->p[0][1]), cv::Scalar(0, 0xff, 0), 2);
        putText(frame, std::to_string(det->id), cv::Point(det->c[0], det->c[1]), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0xff, 0), 2);
    }
}

void mytag::clean() {
    apriltag_detections_destroy(detections);
}

int mytag::getTag0ID() {
    if (zarray_size(detections) > 0) {
        apriltag_detection_t *det;
        zarray_get(detections, 0, &det);
        return det->id;
    }
    return -1;
}

// t是距离系数，自行测量实际距离调整t
double mytag::getTag0Distance(double t) {
    if (zarray_size(detections) > 0) {
        apriltag_detection_t *det;
        zarray_get(detections, 0, &det);
        auto H = det->H;
        // ss是一个缩放因子
        double ss = 0.5;

        // 定义源点
        cv::Mat src = (cv::Mat_<double>(4, 3) <<
            -ss, -ss, 0,
            ss, -ss, 0,
            ss, ss, 0,
            -ss, ss, 0);

        // 相机内参矩阵 K
        cv::Mat Kmat = (cv::Mat_<double>(3, 3) <<
            700, 0, 0,
            0, 700, 0,
            0, 0, 1);

        // 畸变系数
        cv::Mat disCoeffs = cv::Mat::zeros(4, 1, CV_64F);

        // 输入点
        cv::Mat ipoints = (cv::Mat_<double>(4, 2) <<
            -1, -1,
            1, -1,
            1, 1,
            -1, 1);

        // 根据单应矩阵 H 更新 ipoints
        cv::Mat Hcv = matd_to_cvmat(H);  // 将matd_t类型的H转换为cv::Mat
        for (int i = 0; i < ipoints.rows; i++) {
            double x = ipoints.at<double>(i, 0);
            double y = ipoints.at<double>(i, 1);

            // 计算z值
            double z = Hcv.at<double>(2, 0) * x + Hcv.at<double>(2, 1) * y + Hcv.at<double>(2, 2);

            // 更新ipoints
            ipoints.at<double>(i, 0) = (Hcv.at<double>(0, 0) * x + Hcv.at<double>(0, 1) * y + Hcv.at<double>(0, 2)) / z;
            ipoints.at<double>(i, 1) = (Hcv.at<double>(1, 0) * x + Hcv.at<double>(1, 1) * y + Hcv.at<double>(1, 2)) / z;
        }

        // 计算PnP
        cv::Mat opoints = (cv::Mat_<double>(4, 3) <<
            -1.0, -1.0, 0.0,
            1.0, -1.0, 0.0,
            1.0, 1.0, 0.0,
            -1.0, 1.0, 0.0);

        // 缩放opoints
        opoints = opoints * 0.5;

        cv::Mat rvec, tvec;
        cv::solvePnP(opoints, ipoints, Kmat, disCoeffs, rvec, tvec);

        // 投影点
        cv::Mat point;
        cv::projectPoints(src, cv::Mat::zeros(3, 1, CV_64F), tvec, Kmat, disCoeffs, point);

        // 转换为整数类型并计算距离
        cv::Mat points = point.reshape(1, 4);  // 变为 [4, 2] 形状
        cv::Mat diff = points.row(0) - points.row(1);
        double distance = std::abs(t / cv::norm(diff));

        return distance;
    }
    return -1;
}