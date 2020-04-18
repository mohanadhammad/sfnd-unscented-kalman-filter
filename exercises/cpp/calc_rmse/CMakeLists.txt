cmake_minimum_required(VERSION 3.5 FATAL_ERROR)
project(calc_rmse)

find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME} main.cpp)
target_link_libraries(${PROJECT_NAME} PUBLIC Eigen3::Eigen)
