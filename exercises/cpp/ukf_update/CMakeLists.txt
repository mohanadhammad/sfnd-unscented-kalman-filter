cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

set(CMAKE_INCLUDE_CURRENT_DIR ON)

project(ukf_update)

find_package(Eigen3 REQUIRED)

add_executable(${PROJECT_NAME} main.cpp ukf.cpp)

target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
