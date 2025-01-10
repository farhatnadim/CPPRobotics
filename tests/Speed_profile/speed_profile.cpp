
#include "speed_profile.hpp"
#include <array>
#include <cmath>
#include <algorithm>
#include <iostream>

void calc_speed_profile(const int start_index,
                        const int end_index,
                        double max_speed,
                        double stop_speed,
                        int goal_dis,
                        std::array<double, PI_MAX_TRAJECTORY_PTS>& speed_profile)
{
    // Basic validation
    if (start_index < 0 || end_index <= start_index ||
        end_index > static_cast<int>(speed_profile.size()))
    {
        std::cerr << "[calc_speed_profile] Invalid start/end indices.\n";
        return;
    }

    // Convert goal_dis to an integer index range
    int decel_count = goal_dis;
    if (decel_count < 0) {
        // If goal_dis is negative for some reason, just reset
        decel_count = 0;
    }

    // 1. Fill [start_index, end_index) with max_speed
    std::fill(speed_profile.begin() + start_index,
              speed_profile.begin() + end_index,
              max_speed);

    // 2. Apply smooth deceleration from the back
    //    decelerate over min(decel_count, end_index - start_index) points
    decel_count = std::min(decel_count, end_index - start_index);

    for (int i = 0; i < decel_count; ++i) {
        double ratio = static_cast<double>(i) / decel_count;  // from 0 to 1
        double deceleration_ratio = 3 * ratio * ratio - 2 * ratio * ratio * ratio;

        int index_from_rear = end_index - i - 1;  // e.g., if i=0 -> last point in [start_index, end_index)
        // Check boundary again (though it should be safe)
        if (index_from_rear < start_index || index_from_rear >= end_index) {
            continue;
        }

        double speed_from_rear = std::max(stop_speed, max_speed * deceleration_ratio);
        speed_profile[index_from_rear] = speed_from_rear;
    }

    
}
