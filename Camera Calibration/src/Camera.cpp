#include "../include/Camera.h"

cv::Mat Camera::cameraMatrix = cv::Mat();
cv::Mat Camera::distortionCoeffecient = cv::Mat();

// public
Camera::Camera() {
    readChessboardImages();
    calibrate();
}

void Camera::calibrateFrame(cv::Mat &input, cv::Mat &output) {
    cv::undistort(input, output, cameraMatrix, distortionCoeffecient);
}

// private
void Camera::readChessboardImages() {
    std::vector<cv::String> fileNames;
    cv::glob("/home/gellert/Documents/Self Driving Car/Camera Calibration/chessboards/*.jpg", fileNames, false);

    for(auto i : fileNames) {
        chessboards.push_back(cv::imread(i));
    }
}

void Camera::calibrate() {
    std::vector<std::vector<cv::Point3f>> objPoints;
    std::vector<std::vector<cv::Point2f>> imgPoints; 
    std::vector<cv::Point3f> objp;

    for(int i = 0; i<CHECKERBOARD[1]; i++) {
        for(int j = 0; j<CHECKERBOARD[0]; j++)
            objp.push_back(cv::Point3f(j, i, 0));
    }

    cv::Size patternsize(9,6); //interior number of corners
    std::vector<cv::Point2f> corners; //this will be filled by the detected corners
    cv::Mat gray;

    for(auto i = 0; i < chessboards.size(); i++) {
        cv::Mat image = chessboards[i];
        
        if ( !image.data ) {
            printf("No image data \n");
            break;
        }
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);

        bool patternfound = findChessboardCorners(gray, patternsize, corners, cv::CALIB_CB_ADAPTIVE_THRESH | cv::CALIB_CB_FAST_CHECK | cv::CALIB_CB_NORMALIZE_IMAGE);

        if(patternfound){
            cv::TermCriteria criteria(cv::TermCriteria::EPS | cv::TermCriteria::MAX_ITER, 30, 0.001);
            cv::cornerSubPix(gray, corners, cv::Size(11, 11), cv::Size(-1, -1), criteria);
            drawChessboardCorners(image, patternsize, cv::Mat(corners), patternfound);
            objPoints.push_back(objp);
            imgPoints.push_back(corners);
        }
    }

    cv::Mat camMatrix, distCoeffs, R, T, dst, dst2;
    cv::calibrateCamera(objPoints, imgPoints, cv::Size(gray.rows, gray.cols), camMatrix, distCoeffs, R, T);
    for (size_t i = 0; i < chessboards.size(); i++) {
        undistort(chessboards[i], dst, camMatrix, distCoeffs);
        std::cout << camMatrix << std::endl;
        std::string filename = "/home/gellert/Documents/Self Driving Car/Camera Calibration/calibrated_chessboards/" + std::to_string(i) + ".jpg";
        imwrite(filename.c_str(), dst);
    }

    Camera::cameraMatrix = camMatrix;
    Camera::distortionCoeffecient = distCoeffs;
}