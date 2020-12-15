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

    return 0;
}