#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "NumCpp.hpp"
using namespace cv;

void processFrame(Mat &image, Mat &output) {
    Mat gray;
    Mat cannyDst;
    Mat bluredGray;
    Mat color_dst;
    std::vector<Vec4i> lines;

    double rho = 2;
    double theta = CV_PI / 180;
    double threshold = 50;
    double minLineLength = 20;
    double maxLineGap = 200;

    double width = image.size().width;
    double height = image.size().height;

    // canny edge detection
    cvtColor(image, gray, COLOR_BGR2GRAY);
    GaussianBlur(gray, bluredGray, Size(5,5), 0);
    Canny(bluredGray, cannyDst, 50, 150);

    // apply mask
    Mat mask = cv::Mat::zeros(cannyDst.size(), cannyDst.type());

    Point pts[1][4];
    pts[0][0] = Point(width / 2 - width / 10, height / 2 + height / 8);
    pts[0][1] = Point(width / 10, height);
    pts[0][2] = Point(width - width / 10, height);
    pts[0][3] = Point(width / 2 + width / 10, height / 2 + height / 8);
    int npoints = 4;
    const Point* points[1] = {pts[0]};

    fillPoly(mask, points, &npoints, 1, Scalar(255));
    bitwise_and(mask, cannyDst, cannyDst);

    // hough line
    cvtColor(cannyDst, color_dst, COLOR_GRAY2BGR);
    HoughLinesP(cannyDst, lines, rho, theta, threshold, minLineLength, maxLineGap);

    Mat line_img = cv::Mat::zeros(color_dst.size(), color_dst.type());

    std::vector<int> left_x;
    std::vector<int> left_y;
    std::vector<int> right_x;
    std::vector<int> right_y;

    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        double slope = double(l[3] - l[1]) / double(l[2] - l[0]);
        if (abs(slope) < 0.5) continue;
        if (slope < 0) {
            left_x.push_back(l[0]);
            left_x.push_back(l[2]);
            left_y.push_back(l[1]);
            left_y.push_back(l[3]);
        } else {
            right_x.push_back(l[0]);
            right_x.push_back(l[2]);
            right_y.push_back(l[1]);
            right_y.push_back(l[3]);
        }
    }

    nc::NdArray<int> left_line_x(left_x.size(), 1);
    nc::NdArray<int> left_line_y(left_y.size(), 1);
    nc::NdArray<int> right_line_x(right_x.size(), 1);
    nc::NdArray<int> right_line_y(right_y.size(), 1);

    for (size_t i = 0; i < left_x.size(); i++) {
        left_line_x(i, 0) = left_x[i];
    }

    for (size_t i = 0; i < left_y.size(); i++) {
        left_line_y(i, 0) = left_y[i];
    }

    for (size_t i = 0; i < right_x.size(); i++) {
        right_line_x(i, 0) = right_x[i];
    }

    for (size_t i = 0; i < right_y.size(); i++) {
        right_line_y(i, 0) = right_y[i];
    }
    

    auto min_y = height * 3 / 5;
    auto max_y = height;

    auto poly_left = nc::polynomial::Poly1d<int>::fit(left_line_y, left_line_x, 1);
    auto left_x_start = int(poly_left(min_y));
    auto left_x_end = int(poly_left(max_y));

    auto poly_right = nc::polynomial::Poly1d<int>::fit(right_line_y, right_line_x, 1);
    auto right_x_start = int(poly_right(min_y));
    auto right_x_end = int(poly_right(max_y));

    line( line_img, Point(left_x_start, min_y), Point(left_x_end, max_y), Scalar(0,0,255), 3, LINE_AA, 0);
    line( line_img, Point(right_x_start, min_y), Point(right_x_end, max_y), Scalar(0,0,255), 3, LINE_AA, 0);
    
    // combine
    Mat bgra;
    cvtColor(line_img, bgra, cv::COLOR_BGR2BGRA);
    cvtColor(image, image, cv::COLOR_BGR2BGRA);

    addWeighted(bgra, 1, image, 1, 1, output);
}

int main(int argc, char** argv ) {
    if ( argc != 2 ) {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }

    VideoCapture cap(argv[1]);
    if(!cap.isOpened()){
        std::cout << "Error opening video stream or file" << std::endl;
        return -1;
    }

    while(1) {
        Mat frame, result;
        cap >> frame;

        if (frame.empty()) {
            break;
        }

        processFrame(frame, result);

        imshow("Lane Detection", result);

        char c=(char)waitKey(25);
        if(c==27)
            break;

    }

    cap.release();
    destroyAllWindows();

    return 0;
}