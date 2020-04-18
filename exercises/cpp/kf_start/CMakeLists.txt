cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
find_package(Eigen3 REQUIRED)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_library(KalmanFilter STATIC kalman_filter.cpp kalman_filter.hpp)
target_link_libraries(KalmanFilter PUBLIC Eigen3::Eigen)

add_executable(KalmanFilterExample01 example1.cpp)
target_link_libraries(KalmanFilterExample01 PUBLIC Eigen3::Eigen KalmanFilter)

