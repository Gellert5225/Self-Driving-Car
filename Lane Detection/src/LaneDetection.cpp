#include "../include/LaneDetection.h"

LaneDetection::LaneDetection(char* path) {
    cv::VideoCapture cap(path);
    if(!cap.isOpened()){
        throw "Error opening video stream or file\n";
    }
    video = cap;
    cap >> inputFrame;
    this->frameWidth = inputFrame.size().width;
    this->frameHeight = inputFrame.size().height;
}

void LaneDetection::nextFrame() {
    video >> inputFrame;
}

double LaneDetection::getFrameWidth() {
    return this->frameWidth;
}

double LaneDetection::getFrameHeight() {
    return this->frameHeight;
}

void LaneDetection::begin() {
    while(1) {
        cv::Mat frame, result;

        if (inputFrame.empty())
            break;

        processFrame(inputFrame, result);

        cv::imshow("Lane Detection", result);

        char c=(char)cv::waitKey(25);
        if(c==27)
            break;
        nextFrame();
    }
}

void LaneDetection::processFrame(cv::Mat &image, cv::Mat &output) {
    cv::Mat cannyEdgeFrame, cp;
    this->combineSChannelAndGradient(image, cannyEdgeFrame);
    applyROI(cannyEdgeFrame);

    output = houghLine(cannyEdgeFrame);

    cv::Mat bgra;
    cvtColor(output, bgra, cv::COLOR_BGR2BGRA);
    cvtColor(image, image, cv::COLOR_BGR2BGRA);

    addWeighted(bgra, 1, image, 1, 1, output);
}

cv::Mat LaneDetection::cannyEdge(cv::Mat &input) {
    cv::Mat gray;
    cv::Mat cannyDst;
    cv::Mat bluredGray;

    cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    GaussianBlur(gray, bluredGray, cv::Size(GAUSSIAN_BLUR_KERNEL_SIZE, GAUSSIAN_BLUR_KERNEL_SIZE), 0);
    Canny(bluredGray, cannyDst, 50, 150);

    return cannyDst;
}

void LaneDetection::applyROI(cv::Mat &input) {
    cv::Mat mask = cv::Mat::zeros(input.size(), input.type());

    cv::Point pts[1][4];
    pts[0][0] = cv::Point(frameWidth / 2 - frameWidth / 10, frameHeight / 2 + frameHeight / 10);
    pts[0][1] = cv::Point(frameWidth / 10, frameHeight);
    pts[0][2] = cv::Point(frameWidth - frameWidth / 10, frameHeight);
    pts[0][3] = cv::Point(frameWidth / 2 + frameWidth / 10, frameHeight / 2 + frameHeight / 10);
    int npoints = 4;
    const cv::Point* points[1] = {pts[0]};

    fillPoly(mask, points, &npoints, 1, cv::Scalar(255));
    bitwise_and(mask, input, input);
}

cv::Mat LaneDetection::houghLine(cv::Mat &input){
    cv::Mat color_dst;
    std::vector<cv::Vec4i> lines;

    cvtColor(input, color_dst, cv::COLOR_GRAY2BGR);
    HoughLinesP(input, lines, HOUGH_LINE_RHO, HOUGH_LINE_THETA, HOUGH_LINE_THRESHOLD, HOUGH_LINE_MIN_LINE_LENGTH, HOUGH_LINE_MAX_LINE_GAP);

    cv::Mat line_img = cv::Mat::zeros(color_dst.size(), color_dst.type());

    std::vector<int> left_x;
    std::vector<int> left_y;
    std::vector<int> right_x;
    std::vector<int> right_y;

    for (size_t i = 0; i < lines.size(); i++) {
        cv::Vec4i l = lines[i];
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

    auto min_y = frameHeight * 3 / 5;
    auto max_y = frameHeight;

    auto poly_left = nc::polynomial::Poly1d<int>::fit(left_line_y, left_line_x, 1);
    auto left_x_start = int(poly_left(min_y));
    auto left_x_end = int(poly_left(max_y));

    auto poly_right = nc::polynomial::Poly1d<int>::fit(right_line_y, right_line_x, 1);
    auto right_x_start = int(poly_right(min_y));
    auto right_x_end = int(poly_right(max_y));

    line(line_img, cv::Point(left_x_start, min_y), cv::Point(left_x_end, max_y), cv::Scalar(0,0,255), 3, cv::LINE_AA, 0);
    line(line_img, cv::Point(right_x_start, min_y), cv::Point(right_x_end, max_y), cv::Scalar(0,0,255), 3, cv::LINE_AA, 0);

    return line_img;
}

// test functions
void LaneDetection::grayBinary(cv::Mat &input, cv::Mat &output) {
    cv::Mat gray;
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    cv::imshow("gray", gray);
    cv::threshold(gray, output, 180, 255, cv::THRESH_BINARY);
}

void LaneDetection::rChannel(cv::Mat &input, cv::Mat &output) {
    std::vector<cv::Mat> channels;
    cv::split(input, channels);
    output = channels[2];
}

void LaneDetection::gChannel(cv::Mat &input, cv::Mat &output) {
    std::vector<cv::Mat> channels;
    cv::split(input, channels);
    output = channels[1];
}

void LaneDetection::bChannel(cv::Mat &input, cv::Mat &output) {
    std::vector<cv::Mat> channels;
    cv::split(input, channels);
    output = channels[0];
}

void LaneDetection::rBinary(cv::Mat &input, cv::Mat &output) {
    rChannel(input, output);
    cv::threshold(output, output, 200, 255, cv::THRESH_BINARY);
}

void LaneDetection::hChannel(cv::Mat &input, cv::Mat &output) {
    cv::Mat hls;
    cv::cvtColor(input, hls, cv::COLOR_BGR2HLS);
    std::vector<cv::Mat> channels;
    cv::split(hls, channels);
    output = channels[0];
}

void LaneDetection::lChannel(cv::Mat &input, cv::Mat &output) {
    cv::Mat hls;
    cv::cvtColor(input, hls, cv::COLOR_BGR2HLS);
    std::vector<cv::Mat> channels;
    cv::split(hls, channels);
    output = channels[1];
}

void LaneDetection::sChannel(cv::Mat &input, cv::Mat &output) {
    cv::Mat hls;
    cv::cvtColor(input, hls, cv::COLOR_BGR2HLS);
    std::vector<cv::Mat> channels;
    cv::split(hls, channels);
    output = channels[2];
}

void LaneDetection::sBinary(cv::Mat &input, cv::Mat &output) {
    sChannel(input, output);
    cv::threshold(output, output, 170, 255, cv::THRESH_BINARY);
}

void LaneDetection::hBinary(cv::Mat &input, cv::Mat &output) {
    hChannel(input, output);
    cv::threshold(output, output, 15, 100, cv::THRESH_BINARY);
}

void LaneDetection::combineSChannelAndGradient(cv::Mat &input, cv::Mat &output) {
    cv::Mat s_channel;
    cv::Mat s_binary;
    cv::Mat gray;
    cv::Mat sobel_x;
    cv::Mat scaled_sobel;
    cv::Mat sx_binary;

    sChannel(input, s_channel);
    cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);

    // sobel x
    cv::Sobel(gray, sobel_x, CV_64F, 1, 0);
    cv::abs(sobel_x);
    double minVal, maxVal;
    cv::minMaxLoc(sobel_x, &minVal, &maxVal);

    sobel_x.convertTo(scaled_sobel, CV_8U);
    scaled_sobel /= (maxVal / 255.0);

    cv::threshold(scaled_sobel, sx_binary, 20, 200, cv::THRESH_BINARY);
    sBinary(input, s_binary);
    cv::bitwise_or(sx_binary, s_binary, output);
}
