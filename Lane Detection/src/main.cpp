#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include "NumCpp.hpp"
#include "../include/LaneDetection.h"

int main(int argc, char** argv ) {
    if ( argc != 2 ) {
        printf("usage: LaneDetection <Video_Path>\n");
        return -1;
    }

    LaneDetection *laneDetection = new LaneDetection(argv[1]);
    laneDetection->begin();

    cv::destroyAllWindows();

    // cv::Mat image = cv::imread(argv[1]);
    // cv::Mat grayBin;
    // cv::Mat rChannel;
    // cv::Mat gChannel;
    // cv::Mat bChannel;
    // cv::Mat rBinary;
    // cv::Mat hChannel;
    // cv::Mat lChannel;
    // cv::Mat sChannel;
    // cv::Mat sBinary;
    // cv::Mat hBinary;
    // cv::Mat sobel_x;

    // laneDetection->grayBinary(image, grayBin);
    // laneDetection->rChannel(image, rChannel);
    // laneDetection->gChannel(image, gChannel);
    // laneDetection->bChannel(image, bChannel);
    // laneDetection->rBinary(image, rBinary);
    // laneDetection->hChannel(image, hChannel);
    //laneDetection->lChannel(image, lChannel);
    // laneDetection->sChannel(image, sChannel);
    // laneDetection->sBinary(image, sBinary);
    // laneDetection->hBinary(image, hBinary);
    // laneDetection->combineSChannelAndGradient(image, sobel_x);

    // cv::imshow("gray binary", grayBin);
    // cv::imshow("R Channel", rChannel);
    // cv::imshow("G Channel", gChannel);
    // cv::imshow("B Channel", bChannel);
    // cv::imshow("R Binary", rBinary);
    // cv::imshow("H Channel", hChannel);
    //cv::imshow("L Channel", lChannel);
    // cv::imshow("S Channel", sChannel);
    // cv::imshow("H Binary", hBinary);
    // cv::imshow("S Binary", sBinary);
    // cv::imshow("Sobel X", sobel_x);

    cv::waitKey(0);

    return 0;
}