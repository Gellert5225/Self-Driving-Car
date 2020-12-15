#ifndef CAMERA_H
#define CAMERA_H

#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>

class Camera {
public:
    static cv::Mat cameraMatrix;
    static cv::Mat distortionCoeffecient;

    Camera();

    // to calibrate a frame
    void calibrateFrame(cv::Mat &input, cv::Mat &output);
private:
    int CHECKERBOARD[2]{9, 6};
    std::vector<cv::Mat> chessboards;

    // read chess board patterns
    void readChessboardImages();

    // to generate camera matrix and distortion coeffecient
    void calibrate();
};

#endif