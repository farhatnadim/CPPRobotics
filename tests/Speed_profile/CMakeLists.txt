cmake_minimum_required(VERSION 3.10)
project(SpeedProfileTests)

# GoogleTest
find_package(GTest REQUIRED)

include_directories(${GTEST_INCLUDE_DIRS})


add_executable(test_speed_profile
    test_speed_profile.cpp
    speed_profile.cpp
    my_test_main.cpp    # <-- Our custom main
)
target_link_libraries(test_speed_profile
    GTest::gtest        # use gtest (no _main)
    pthread
)
