cmake_minimum_required(VERSION 3.10)
project(lqr_speed_control)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Find Eigen3 package
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# Build the library
add_library(lqr_speed_control
    lqr_speed_control.cpp
)
# Include the Eigen directory
target_include_directories(lqr_speed_control
    PUBLIC ${EIGEN_INCLUDE_DIRS}
)
# ... existing code ...
# if macos
if(APPLE)
    include_directories(/opt/homebrew/include)
endif()
# ... existing code ...
target_link_libraries(lqr_speed_control
    PUBLIC Eigen3::Eigen
)


# Optional: Enable optimization for Release builds
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")