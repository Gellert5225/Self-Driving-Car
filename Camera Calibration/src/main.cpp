#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "../include/Camera.h"
#include <iostream>
using namespace cv;

int main(int argc, char** argv ) {
    Mat test = imread("/home/gellert/Documents/Self Driving Car/Camera Calibration/test_images/test2.jpg");
    Mat dst;

    Camera *camera = new Camera();
    camera->calibrateFrame(test, dst);

    imshow("original", test);
    imshow("calibrated", dst);
    waitKey(0);

    return 0;
}