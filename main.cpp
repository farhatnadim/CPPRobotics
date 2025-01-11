#include "planning/path_fitting/clothoid/src/clothoid_path.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include "control/lqr_control/src/lqr_speed_control.hpp"

struct PathNode {
  std::string node_id;
  double x;
  double y;
  double heading;
  std::string end_node_id;
  double max_steer_angle;
  double max_speed;
  std::string trajectory_type;
};

struct PathData {
  std::unordered_map<std::string, PathNode> nodes;

  PathData() {
    // Initialize with your path data
    nodes["500"] = {"500", 190.61, 183.0, 180, "501", 26.64, 0.600, "Clothoid"};
    nodes["501"] = {"501", 187.25, 187.609, 90.0, "502", 0.00, 2.000, "Clothoid"};
    nodes["502"] = {"502", 187.25, 194.0, 90.0, "503", 0.00, 1.500, "Clothoid"};
    nodes["503"] = {"503", 187.25, 195.5, 90.0, "504", 48.37, 0.600, "Clothoid"};
    nodes["504"] = {"504", 189.453, 199.005, 0.0, "505", 0.00, 2.000, "Clothoid"};
    nodes["505"] = {"505", 195.5, 199.005, 0.0, "509", 0.00, 1.500, "Clothoid"};
    nodes["506"] = {"506", 195.151, 183, 180, "507", 0.00, 1.500, "Clothoid"};
  }
};

std::vector<double> trapezoidal_speed_profile(int path_size, double v_max, double v_end, double a_accel, double a_decel) 
{
    // Initialize speed profile
    std::vector<double> speed_profile(path_size, 0.0);

    // 1. Accelerate from start
    // speed at step i assuming start from 0 and constant accel: v_i = sqrt(2*a_accel*distance)
    // We'll fill this incrementally and consider partial distances as increments of "1" unit.
    for (int i = 1; i < path_size; ++i) {
        double v_prev = speed_profile[i-1];
        double v_candidate = std::sqrt(v_prev*v_prev + 2*a_accel*1.0); // assuming unit distance between points
        speed_profile[i] = std::min(v_candidate, v_max);
    }

    // 2. Decelerate from the end to meet v_end exactly
    // Starting from the goal backwards, ensure we can stop at v_end
    speed_profile.back() = v_end;
    for (int i = path_size-2; i >= 0; --i) {
        double v_prev = speed_profile[i+1];
        double v_candidate = std::sqrt(v_prev*v_prev + 2*a_decel*1.0); // from the end backwards
        // The actual allowed speed at i must be the minimum of what we set during acceleration and what we can achieve ensuring we can slow down in time
        speed_profile[i] = std::min(speed_profile[i], v_candidate);
    }

    // After this backward pass, we have a trapezoidal shape. However, the backward pass might reduce speeds below what was set by the forward pass.
    // To ensure consistency, you might do one or two passes forward again:
    for (int i = 1; i < path_size; ++i) {
        double v_prev = speed_profile[i-1];
        double v_candidate = std::sqrt(v_prev*v_prev + 2*a_accel*1.0);
        speed_profile[i] = std::min(speed_profile[i], v_candidate);
    }

    return speed_profile;
}

void calc_speed_profile(const int &path_size, double max_speed, double stop_speed, double goal_dis, std::vector<double> &speed_profile) 
{
    std::fill(speed_profile.begin(), speed_profile.end(), max_speed);

    // Apply smooth deceleration
    for (int i = 0; i < goal_dis && i < path_size; ++i) {
        double ratio = static_cast<double>(i) / goal_dis;
        // Cubic smoothing function
        // Calculate speed with minimum speed limit
        speed_profile[speed_profile.size() - 1 - i] = std::max(
            stop_speed,
            max_speed * (3*ratio*ratio - 2*ratio*ratio*ratio)
        );
    }
    speed_profile.push_back(stop_speed);
}
void do_simulation(const ClothoidPath& path) 
{
    // if path is empty, return
    if (path.points.empty() || path.yaws.empty() || path.curvatures.empty()) {
        std::cerr << "Path has empty points, yaws, or curvatures" << std::endl;
        return;
    }
    std::vector<double> max_steer_angles = {0, 26.64*M_PI/180.0, 0.0, 0.0, 48.37*M_PI/180.0  , 0.0};
    std::vector<double> max_speeds = {0.6, 2.0, 1.5, 0.6, 2.0, 1.5};
    std::vector<double> stop_speeds = {0.6, 1.5, 0.6, 2.0, 1.5, 0.001};
    std::vector<double> goal_dis = {10, 10, 10, 10, 10, 10};
    // do these in a constructor for a the class motion controller
    // private variables
    constexpr double max_speed {2.0};
    constexpr double stop_speed {0.6};
    constexpr int goal_distance {10};

    SimulationParams params;
    params.max_speed = max_speed;
    params.stop_speed = stop_speed;
    params.goal_dis = goal_distance;
    State state;
    Matrix<double, STATE_DIM, STATE_DIM> lqr_Q;
    Matrix<double, CONTROL_DIM, CONTROL_DIM> lqr_R;
   
    // LQR matrices
    lqr_Q = Matrix<double, STATE_DIM, STATE_DIM>::Identity() * WEIGHT_STATE;  // Increase weights
    lqr_R = Matrix<double, CONTROL_DIM, CONTROL_DIM>::Identity();
    state = State(path.points[0].x, path.points[0].y, path.yaws[0], 0.0);
    double time = 0.0;
    // convert these
    std::vector<double> x, y, yaw, v, t;
    x.push_back(state.x);
    y.push_back(state.y);
    yaw.push_back(state.yaw);
    v.push_back(state.v);
    t.push_back(0.0);
    std::vector<double> speed_profile(path.yaws.size(), 0.0);
    calc_speed_profile(path.yaws.size(), params.max_speed, params.stop_speed, params.goal_dis, speed_profile);
    //auto speed_profile = trapezoidal_speed_profile(path.yaws.size(), params.max_speed, params.stop_speed, 1.0, 1.0);
    // write the speed profile to a file
    std::ofstream speed_profile_file("speed_profile.csv");
    for ( auto &speed : speed_profile)
    {
        speed_profile_file << speed << std::endl;
    }
    speed_profile_file.close();
    double e = 0.0, e_th = 0.0;

    std::vector<double> path_x;
    std::vector<double> path_y;
    for (const auto& point : path.points) 
   {
        path_x.push_back(point.x);
        path_y.push_back(point.y);
    }
    
    std::ofstream path_file("data.txt");
    for (int i = 0; i < path.yaws.size(); i++)
    {
        path_file << path.points[i].x << "," << path.points[i].y << "," << path.yaws[i] << "," << path.curvatures[i] << std::endl;
    }

    path_file.close();
  
    std::ofstream state_file("state.txt");
    while (params.T >= time) 
    {
        double dl, ai;
        int target_ind;
        
        std::tie(dl, target_ind, e, e_th, ai) = lqr_speed_steering_control(
            state, path_x, path_y, path.yaws, path.curvatures, 
            e, e_th, speed_profile, lqr_Q, lqr_R, 
            params
        );    
     
        state = update(state, ai, dl, params);
        // write to file
    
        if (abs(state.v) <= params.stop_speed) 
        {
            target_ind++;
        }

        time += params.dt;
        double dx = state.x - path.points.back().x;
        double dy = state.y - path.points.back().y;
        std::cout << "Distance: " << hypot(dx, dy) << std::endl;
        std::cout << "Goal distance: " << path.points.back().x << "," << path.points.back().y << std::endl;
        double distance_tolerance = 0.2;
        if (hypot(dx, dy) <= distance_tolerance) 
        {
            std::cout << "Goal" << std::endl;
            break;
        }

        x.push_back(state.x);
        y.push_back(state.y);
        yaw.push_back(state.yaw);
        v.push_back(state.v);
        t.push_back(time);
    }
    // write x, y, yaw to a csv file
    static int counter = 0;
    std::ofstream file("lqr_speed_control_path.csv");
    for (size_t i = 0; i < x.size(); i++) 
    { 
        file << t[i] << "," << x[i] << "," << y[i] << "," << yaw[i] << "," << v[i] << std::endl;
    }

    file.close();
    
}


int main() {
  double wheelbase = 3.0;
  int n_path_points = 100;
  ClothoidPathGenerator generator(n_path_points, wheelbase);
  PathData path_data;
  // steering angles vector
  
  const auto& start_node = path_data.nodes["506"];

  // Define waypoints
  const std::vector<PathNode> way_nodes = {
      path_data.nodes["500"], path_data.nodes["501"], path_data.nodes["502"],
      path_data.nodes["503"], path_data.nodes["504"]};

  const auto& end_node = path_data.nodes["505"];
  Point start_point = {start_node.x, start_node.y,
                       start_node.heading * M_PI / 180.0};

  std::vector<Point> all_points;
  all_points.reserve(way_nodes.size() + 2);

  all_points.push_back(start_point);
  for (const auto& node : way_nodes) {
    all_points.push_back({node.x, node.y, node.heading * M_PI / 180.0});
  }
  Point goal_point = {end_node.x, end_node.y,
                      end_node.heading * M_PI / 180.0};
  all_points.push_back(goal_point);

  ClothoidPath full_path;
  generator.generateClothoidPaths(all_points, full_path);
  
  // Add error checking
  if (full_path.points.empty()) {
    std::cerr << "Failed to generate any valid clothoid paths" << std::endl;
    return 1;
  }
  
  // full path size
  int path_size = full_path.yaws.size();
  // print the path size
  std::cout << "Path size: " << path_size << std::endl;
  // write the path to a csv file
  std::ofstream path_file("path.csv");
  for (size_t i = 0; i < full_path.points.size(); i++) {
    path_file << full_path.points[i].x << "," << full_path.points[i].y << "," << full_path.yaws[i] << "," << full_path.curvatures[i] << std::endl;
  }
  do_simulation(full_path);
  return 0;
}

