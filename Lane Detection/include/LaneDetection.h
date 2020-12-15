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

    // test functions
    void grayBinary(cv::Mat &input, cv::Mat &output);
    void rChannel(cv::Mat &input, cv::Mat &output);
    void gChannel(cv::Mat &input, cv::Mat &output);
    void bChannel(cv::Mat &input, cv::Mat &output);
    void rBinary(cv::Mat &input, cv::Mat &output);
    void hChannel(cv::Mat &input, cv::Mat &output);
    void lChannel(cv::Mat &input, cv::Mat &output);
    void sChannel(cv::Mat &input, cv::Mat &output);
    void sBinary(cv::Mat &input, cv::Mat &output);
    void hBinary(cv::Mat &input, cv::Mat &output);
    void combineSChannelAndGradient(cv::Mat &input, cv::Mat &output);
private:
    double frameWidth;
    double frameHeight;
    cv::Mat inputFrame;
    cv::Mat outputFrame;
    cv::VideoCapture video;

    void nextFrame();
};

#endif