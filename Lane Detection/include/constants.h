#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <opencv2/opencv.hpp>

const int GAUSSIAN_BLUR_KERNEL_SIZE = 5;

const double CANNY_EDGE_THRESHOLD_1 = 50.0;
const double CANNY_EDGE_THRESHOLD_2 = 50.0;

const double HOUGH_LINE_RHO = 2.0;
const double HOUGH_LINE_THETA = CV_PI / 180.0;
const double HOUGH_LINE_THRESHOLD = 50.0;
const double HOUGH_LINE_MIN_LINE_LENGTH = 20.0;
const double HOUGH_LINE_MAX_LINE_GAP = 200.0;

#endif