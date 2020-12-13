#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
using namespace cv;

void processFrame(Mat &image) {
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

    for (size_t i = 0; i < lines.size(); i++) {
        Vec4i l = lines[i];
        double slope = double(l[3] - l[1]) / double(l[2] - l[0]);
        if (abs(slope) < 0.5) continue;
        line( line_img, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 6, 1, 0);
    }
    
    // combine
    addWeighted(line_img, 0.8, image, 1, 0, image);
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
        Mat frame;
        cap >> frame;

        if (frame.empty()) {
            break;
        }

        processFrame(frame);

        imshow("Lane Detection", frame);

        char c=(char)waitKey(25);
        if(c==27)
            break;

    }

    cap.release();
    destroyAllWindows();

    return 0;
}