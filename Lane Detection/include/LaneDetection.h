#ifndef LANE_DETECTION_H
#define LANE_DETECTION_H

#include "constants.h"
#include "NumCpp.hpp"
#include <vector>

class LaneDetection {
public:
    LaneDetection(char*);

    double getFrameWidth();
    double getFrameHeight();

    void begin();
    void processFrame(cv::Mat &image, cv::Mat &output);
    cv::Mat cannyEdge(cv::Mat &input);
    void applyROI(cv::Mat &input);
    cv::Mat houghLine(cv::Mat &input);
private:
    double frameWidth;
    double frameHeight;
    cv::Mat inputFrame;
    cv::Mat outputFrame;
    cv::VideoCapture video;

    void nextFrame();
};

#endif