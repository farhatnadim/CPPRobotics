#pragma once

#include <array>
constexpr int PI_MAX_TRAJECTORY_PTS = 300;
void calc_speed_profile(const int start_index, const int end_index, double max_speed, double stop_speed, int goal_dis, std::array<double, PI_MAX_TRAJECTORY_PTS>& speed_profile);
