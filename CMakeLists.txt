cmake_minimum_required(VERSION 3.25)
project(Kalman)

find_package(Eigen3 REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(include ${EIGEN3_INCLUDE_DIR})

add_library(Kalman src/filters/KalmanFilter.cpp src/filters/ExtendedKalmanFilter.cpp)
add_executable(KalmanTest src/main.cpp)

target_link_libraries(Kalman Eigen3::Eigen)
target_link_libraries(KalmanTest Kalman)
