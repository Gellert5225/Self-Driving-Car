#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
using namespace cv;

int main(int argc, char** argv ) {
    if ( argc != 2 ) {
        printf("usage: DisplayImage.out <Image_Path>\n");
        return -1;
    }

    int CHECKERBOARD[2]{8,6};

    std::vector<std::vector<Point3f> > objPoints;
    std::vector<std::vector<Point2f> > imgPoints; 
    std::vector<Point3f> objp;

    for(int i = 0; i<CHECKERBOARD[1]; i++) {
        for(int j = 0; j<CHECKERBOARD[0]; j++)
            objp.push_back(Point3f(j, i, 0));
    }

    Mat image;
    image = imread( argv[1], 1 );
    if ( !image.data ) {
        printf("No image data \n");
        return -1;
    }
    Size patternsize(8,6); //interior number of corners
    std::vector<Point2f> corners; //this will be filled by the detected corners
    Mat gray;
    cvtColor(image, gray, COLOR_BGR2GRAY);

    bool patternfound = findChessboardCorners(gray, patternsize, corners);

    if(patternfound){

        drawChessboardCorners(image, patternsize, Mat(corners), patternfound);
        objPoints.push_back(objp);
        imgPoints.push_back(corners);

        imshow("chess board", image);
    }

    cv::Mat cameraMatrix, distCoeffs, R, T, dst;
    cv::calibrateCamera(objPoints, imgPoints, cv::Size(image.rows, image.cols), cameraMatrix, distCoeffs, R, T);
    undistort(image, dst, cameraMatrix, distCoeffs);
    imshow("calibrated", dst);
    waitKey(0);

    return 0;
}