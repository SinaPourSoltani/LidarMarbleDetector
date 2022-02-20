cmake_minimum_required(VERSION 3.12)
project(LidarMarbleDetector)

set(CMAKE_CXX_STANDARD 17)

add_executable(${CMAKE_PROJECT_NAME} main.cpp LidarMarbleDetector.h LidarMarbleDetector.cpp)

find_package( OpenCV REQUIRED )
include_directories(OpenCV_INCLUDE_DIRS)
target_link_libraries(${CMAKE_PROJECT_NAME} ${OpenCV_LIBS})