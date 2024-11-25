cmake_minimum_required(VERSION 3.10)
project(ClothoidPathPlanner)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Add the library target
add_library(clothoid_path
    clothoid_path.cpp
)

# Include directories for the library
target_include_directories(clothoid_path
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
)


# Link the library to the executable
