// File: test_speed_profile.cpp

#include <gtest/gtest.h>
#include <fstream>     // for std::ofstream
#include <array>
#include <iostream>

// Include your header with the calc_speed_profile function
#include "speed_profile.hpp"

TEST(SpeedProfileTest, BasicDecelerationTest)
{
    // 1. Define the size of the array (e.g., 100).
    //    Suppose your constant is:  #define PI_MAX_TRAJECTORY_PTS 100
    //    or a static constexpr int in a header, etc.

    const int SIZE = PI_MAX_TRAJECTORY_PTS;
    std::array<double, SIZE> speed_profile_1{};
    std::array<double, SIZE> speed_profile_2{};
    speed_profile_1.fill(0.0);
    speed_profile_2.fill(0.0);


    // 2. Define test parameters
    int start_index_1 = 0;
    int end_index_1   = 100;     // We'll set half the array in this test
    double max_speed_1 = 2;  // example: 10 m/s
    double stop_speed_1 = 0.6;  // fully stop
    double goal_dis_1   = 10.0; // number of points over which to decelerate

    // 3. Call the function being tested
    calc_speed_profile(start_index_1,
                       end_index_1,
                       max_speed_1,
                       stop_speed_1,
                       goal_dis_1,
                       speed_profile_1);
    int start_index_2 = 100;
    int end_index_2   = 200;     // We'll set half the array in this test
    double max_speed_2 = 0.6;  // example: 10 m/s
    double stop_speed_2 = 0.6;  // fully stop
    double goal_dis_2   = 10.0; // number of points over which to decelerate
    calc_speed_profile(start_index_2,
                       end_index_2,
                       max_speed_2,
                       stop_speed_2,
                       goal_dis_2,
                       speed_profile_1);
    int start_index_3 = 200;
    int end_index_3   = 300;     // We'll set half the array in this test
    double max_speed_3 = 0.6;  // example: 10 m/s
    double stop_speed_3 = 1.5;  // fully stop
    double goal_dis_3   = 10.0; // number of points over which to decelerate
    calc_speed_profile(start_index_3,
                       end_index_3,
                       max_speed_3,
                       stop_speed_3,
                       goal_dis_3,
                       speed_profile_1);
                       
                       
    int start_index_4 = 0;
    int end_index_4   = 100;     // We'll set half the array in this test
    double max_speed_4 = 1.5;  // example: 10 m/s
    double stop_speed_4 = 1.5   ;  // fully stop
    double goal_dis_4   = 10.0; // number of points over which to decelerate
    calc_speed_profile(start_index_4,
                       end_index_4,
                       max_speed_4,
                       stop_speed_4,
                       goal_dis_4,
                       speed_profile_2);
    int start_index_5 = 100;
    int end_index_5   = 200;     // We'll set half the array in this test
    double max_speed_5 = 1.5;  // example: 10 m/s
    double stop_speed_5 = 0.6   ;  // fully stop
    double goal_dis_5   = 10.0; // number of points over which to decelerate
    calc_speed_profile(start_index_5,
                       end_index_5,
                       max_speed_5,
                       stop_speed_5,
                       goal_dis_5,
                       speed_profile_2);
    int start_index_6 = 200;
    int end_index_6   = 300;     // We'll set half the array in this test
    double max_speed_6 = 0.6;  // example: 10 m/s
    double stop_speed_6 = 0   ;  // fully stop
    double goal_dis_6   = 10.0; // number of points over which to decelerate
    calc_speed_profile(start_index_6,
                       end_index_6,
                       max_speed_6,
                       stop_speed_6,
                       goal_dis_6,
                       speed_profile_2);
    // 4. Basic sanity checks
    //    We expect first part of the segment to be near max_speed, 
    //    and the last 'goal_dis' points to have decreased.
    //    This is just an example; you can add more thorough checks.
    EXPECT_NEAR(speed_profile_1[start_index_1], max_speed_1, 1e-5);
    EXPECT_NEAR(speed_profile_1[end_index_1 - 1], stop_speed_1, 1e-5);

    // 5. Write output to a CSV file for external plotting
    std::ofstream ofs("speed_profile_output_part1.csv");
    std::ofstream ofs2("speed_profile_output_part2.csv");
    if (!ofs.is_open()) {
        std::cerr << "Error: Could not open file for writing.\n";
        return;
    }

    // Write CSV header
    ofs << "Index,Speed\n";
    // Write each point in speed_profile
    for (int i = 0; i < SIZE; ++i) {
        ofs << i << "," << speed_profile_1[i] << "\n";
    }
    ofs.close();
    ofs2 << "Index,Speed\n";
    for (int i = 0; i < SIZE; ++i) {
        ofs2 << i << "," << speed_profile_2[i] << "\n";
    }
    ofs2.close();

    // Test completes successfully if it reaches this point.
    SUCCEED();
}

// You might have additional test cases here for boundary checks, etc.

