//
// Created by ashkore on 25-1-13.
//

#include "mytag.h"

/**
 * @brief 将matd_t类型的矩阵转换为OpenCV的cv::Mat类型
 *
 * @param mat 输入的matd_t类型矩阵
 * @return cv::Mat 转换后的cv::Mat对象
 */
cv::Mat matd_to_cvmat(const matd_t* mat) {
    // 创建一个cv::Mat对象，行数为nrows，列数为ncols，数据类型为CV_64F（64位浮动类型）
    return {static_cast<int>(mat->nrows), static_cast<int>(mat->ncols), CV_64F, (void*)mat->data};
}

mytag::mytag(const std::string& tagFamily, float decimate, float sigma, int threads, bool debug, bool refine) {
    // 创建AprilTag检测器
    td = apriltag_detector_create();
    if (tagFamily == "tag36h11") {
        tf = tag36h11_create();
    } else if (tagFamily == "tag36h10") {
        tf = tag36h10_create();
    } else if (tagFamily == "tag25h9") {
        tf = tag25h9_create();
    } else if (tagFamily == "tag16h5") {
        tf = tag16h5_create();
    } else if (tagFamily == "tagCircle21h7") {
        tf = tagCircle21h7_create();
    } else if (tagFamily == "tagCircle49h12") {
        tf = tagCircle49h12_create();
    } else if (tagFamily == "tagCustom48h12") {
        tf = tagCustom48h12_create();
    } else if (tagFamily == "tagStandard41h12") {
        tf = tagStandard41h12_create();
    } else if (tagFamily == "tagStandard52h13") {
        tf = tagStandard52h13_create();
    } else {
        std::cerr << "Error: Invalid tag family" << std::endl;
        return;
    }
    this->tagFamily = tagFamily;
    apriltag_detector_add_family(td, tf);

    // 设置参数
    td->quad_decimate = decimate;
    td->quad_sigma = sigma;
    td->nthreads = threads;
    td->debug = debug;
    td->refine_edges = refine;

    detections = nullptr;
    closestTagIndex = -1;
}

mytag::~mytag() {
    apriltag_detector_destroy(td);
    if (tagFamily == "tag36h11") {
        tag36h11_destroy(tf);
    } else if (tagFamily == "tag36h10") {
        tag36h10_destroy(tf);
    } else if (tagFamily == "tag25h9") {
        tag25h9_destroy(tf);
    } else if (tagFamily == "tag16h5") {
        tag16h5_destroy(tf);
    } else if (tagFamily == "tagCircle21h7") {
        tagCircle21h7_destroy(tf);
    } else if (tagFamily == "tagCircle49h12") {
        tagCircle49h12_destroy(tf);
    } else if (tagFamily == "tagCustom48h12") {
        tagCustom48h12_destroy(tf);
    } else if (tagFamily == "tagStandard41h12") {
        tagStandard41h12_destroy(tf);
    } else if (tagFamily == "tagStandard52h13") {
        tagStandard52h13_destroy(tf);
    }
}

void mytag::detect(const cv::Mat& gray) {
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
        cv::Scalar color = (i == closestTagIndex) ? cv::Scalar(0, 0xff, 0) : cv::Scalar(0, 0, 0xff);
        apriltag_detection_t *det;
        zarray_get(detections, i, &det);

        // 在框架上绘制检测到的标签
        line(frame,cv::Point((int)det->p[0][0], (int)det->p[0][1]),
                 cv::Point((int)det->p[1][0], (int)det->p[1][1]), color, 2);
        line(frame, cv::Point((int)det->p[1][0], (int)det->p[1][1]),
                 cv::Point((int)det->p[2][0], (int)det->p[2][1]), color, 2);
        line(frame,cv::Point((int)det->p[2][0], (int)det->p[2][1]),
                 cv::Point((int)det->p[3][0], (int)det->p[3][1]), color, 2);
        line(frame, cv::Point((int)det->p[3][0], (int)det->p[3][1]),
                 cv::Point((int)det->p[0][0], (int)det->p[0][1]), color, 2);
        putText(frame, std::to_string((int)det->id), cv::Point((int)det->c[0], (int)det->c[1]), cv::FONT_HERSHEY_SIMPLEX, 1, color, 2);
    }
}

void mytag::clean() {
    apriltag_detections_destroy(detections);
}

int mytag::getClosetTagIndex() {
    if(zarray_size(detections)) {
        auto getDistance = [=](const double point1[2], const double point2[2]) {
            return (point1[0] - point2[0]) * (point1[0] - point2[0]) +
                   (point1[1] - point2[1]) * (point1[1] - point2[1]);
        };
        auto getTagLongestLine = [=](apriltag_detection_t *det) {
            double line1 = getDistance(det->p[0], det->p[1]);
            double line2 = getDistance(det->p[1], det->p[2]);
            double line3 = getDistance(det->p[2], det->p[3]);
            double line4 = getDistance(det->p[3], det->p[0]);
            return std::max(std::max(line1, line2), std::max(line3, line4));
        };
        auto TagLongestLine = 0.0;
        closestTagIndex = 0;
        for (int index = 0; index < zarray_size(detections); index++) {
            apriltag_detection_t *det;
            zarray_get(detections, index, &det);
            if (getTagLongestLine(det) > TagLongestLine) {
                closestTagIndex = index;
                TagLongestLine = getTagLongestLine(det);
            }
        }
        return closestTagIndex;
    }
    return -1;
}

int mytag::getClosetTagID() {
    if (zarray_size(detections)) {
        apriltag_detection_t *det;
        zarray_get(detections, closestTagIndex, &det);
        return det->id;
    }
    return -1;
}

// t是距离系数，自行测量实际距离调整t
double mytag::getClosetTagDistance(double t) {
    if (zarray_size(detections)) {
        apriltag_detection_t *det;
        zarray_get(detections, closestTagIndex, &det);
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
            108, 0, 80,
            0, 108, 60,
            0, 0, 1);

        // 畸变系数
//        cv::Mat disCoeffs = (cv::Mat_<double>(5, 1) << -0.03814017,  0.29592995, -0.00294403,  0.00292109, -0.47267434);
        cv::Mat disCoeffs = (cv::Mat_<double>(5, 1) << 0,0,0,0,0);
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