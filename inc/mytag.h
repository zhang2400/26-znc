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
    mytag(const char *tagFamily, float decimate, float sigma, int threads, bool debug, bool refine);
    ~mytag();
    void detect(cv::Mat &gray);
    void draw(cv::Mat &frame);
    void clean();
    int getTag0ID();
    double getTag0Distance(double t);
private:
    apriltag_detector_t *td;
    apriltag_family_t *tf;
    zarray_t *detections;
};

#endif //MYTAG_H
