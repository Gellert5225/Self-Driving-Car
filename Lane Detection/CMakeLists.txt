cmake_minimum_required(VERSION 2.8)
set (CMAKE_CXX_STANDARD 11)

project( LaneDetection )
find_package( OpenCV REQUIRED )
find_package(NumCpp 2.1.0 REQUIRED)

FILE(GLOB source src/*.cpp src/*.c)

include_directories( ${OpenCV_INCLUDE_DIRS} include)
add_executable( LaneDetection ${source} )

set(EXECUTABLE_OUTPUT_PATH ./bin)
target_link_libraries( LaneDetection ${OpenCV_LIBS} NumCpp::NumCpp)