cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_INSTALL_PREFIX "${CMAKE_CURRENT_SOURCE_DIR}/build/")

project(lidar_tracking)

find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME} main.cpp tracking.cpp kalman_filter.cpp)
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)

install(
    FILES "${CMAKE_CURRENT_SOURCE_DIR}/dat/obj_pose-laser-radar-synthetic-input.txt"
    DESTINATION dat
)