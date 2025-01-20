cmake_minimum_required(VERSION 3.10)
project(KalmanFilter)

# Specify the C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# Include the Eigen headers (use full path and correct case)
include_directories("/home/bandapear/Documents/source/eigen")

# Add the executable
add_executable(KalmanFilter main.cpp)


