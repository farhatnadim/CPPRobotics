# @copyright Copyright (C) 2024-2025, JBT
# All right reserved. Property of JBT.
# Restricted rights to use, duplicate or disclose
# of this file are granted through contract.

cmake_minimum_required(VERSION 3.10)
project(trajectory_planning)
# set the c++ standard to 17
set(CMAKE_CXX_STANDARD 20)
# find package
find_package(Eigen3 REQUIRED NO_MODULE)
#recurse into directories on the same level
add_subdirectory(control/lqr_control/src)
add_subdirectory(planning/path_fitting/clothoid/src/)
# create the executable
add_executable(trajectory_planning main.cpp)
# include eigen
target_include_directories(trajectory_planning PRIVATE ${EIGEN_INCLUDE_DIRS})
# link the libraries
target_link_libraries(trajectory_planning lqr_speed_control clothoid_path)
# if macos
if(APPLE)
    include_directories(/opt/homebrew/include)
endif()
