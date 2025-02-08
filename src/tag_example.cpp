//
// Created by EiveLL on 25-1-17.
//

#include <iostream>
#include <opencv2/opencv.hpp>
#include <mytag.h>

int tag_test() {
    auto cap = cv::VideoCapture();
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 160);
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 120);
    cap.open(0);
    cv::Mat frame;
    cv::Mat gray;

    auto atag = mytag("tag36h11", 3.0, 0.0, 1, false, false);
    while (true) {
        cap >> frame;
        double start = cv::getTickCount();
        cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        atag.detect(gray);
        atag.getClosetTagIndex();
        atag.draw(frame);
        auto id = atag.getClosetTagID();
        auto distance = atag.getClosetTagDistance(1000);
        double end = cv::getTickCount();
        auto duration = (end - start) / cv::getTickFrequency();
        std::string text = "process: " + std::to_string(duration*1000) + "ms";
        cv::putText(frame, text, cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 2);
        imshow("Camera", frame);
        std::cout << "id:" << id << " distance:" << distance << std::endl;
        if (cv::waitKey(1) == 'q') {
            break;
        }
    }
    return 0;
}
