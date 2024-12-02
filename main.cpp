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
std::vector<double> calc_speed_profile(const int &path_size, double max_speed, double stop_speed, int goal_dis)
{
    std::vector<double> speed_profile(path_size, max_speed);
    // Apply smooth deceleration
    for (int i = 0; i < goal_dis && i < path_size; ++i) {
        double ratio = static_cast<double>(i) / goal_dis;
        // Cubic smoothing function
        double smooth_ratio = ratio * ratio * (3.0 - 2.0 * ratio);
        // Calculate speed with minimum speed limit
        speed_profile[speed_profile.size() - 1 - i] = std::max(
            stop_speed,
            max_speed * (1.0 - smooth_ratio)
        );
    }
    
    return speed_profile;
}
void do_simulation(const ClothoidPath& path) 
{
    // if path is empty, return
    if (path.points.empty() || path.yaws.empty() || path.curvatures.empty()) {
        std::cerr << "Path has empty points, yaws, or curvatures" << std::endl;
        return;
    }
    // do these in a constructor for a the class motion controller
    // private variables
    constexpr double max_speed {2.0};
    constexpr double stop_speed {0.001};
    constexpr int goal_distance {10};

    SimulationParams params;
    params.max_speed = max_speed;
    params.stop_speed = stop_speed;
    params.goal_dis = goal_distance;
    State state;
    Matrix<double, STATE_DIM, STATE_DIM> lqr_Q;
    Matrix<double, CONTROL_DIM, CONTROL_DIM> lqr_R;
    params.max_steer = 0.0 ;
    // LQR matrices
    lqr_Q = Matrix<double, STATE_DIM, STATE_DIM>::Identity() * WEIGHT_STATE;  // Increase weights
    lqr_R = Matrix<double, CONTROL_DIM, CONTROL_DIM>::Identity();
    state = State(path.points[0].x, path.points[0].y, path.yaws[0], 0.0);
 
    // convert these
    std::vector<double> x, y, yaw, v, t;
    x.push_back(state.x);
    y.push_back(state.y);
    yaw.push_back(state.yaw);
    v.push_back(state.v);
    t.push_back(0.0);

    double e = 0.0, e_th = 0.0;

    std::vector<double> path_x;
    std::vector<double> path_y;
    for (const auto& point : path.points)
    {
        path_x.push_back(point.x);
        path_y.push_back(point.y);
    }
    // size of the path
    int path_size = path.yaws.size();

    auto speed_profile = calc_speed_profile(path_size, params.max_speed, params.stop_speed, params.goal_dis);
    // instead of using the time, use the distance to the goal
    constexpr int number_of_iterations = 10000;
    int i = 0;
    while (true && i < number_of_iterations) 
    {
        double dl, ai;
        int target_ind;
        
        std::tie(dl, target_ind, e, e_th, ai) = lqr_speed_steering_control(
            state, path_x, path_y, path.yaws, path.curvatures, 
            e, e_th, speed_profile, lqr_Q, lqr_R, 
            params
        );    

        state = update(state, ai, dl, params);

        if (abs(state.v) <= params.stop_speed) 
        {
            target_ind++;
        }

        double dx = path.points.back().x - state.x;
        double dy = path.points.back().y - state.y;
        // need to change this to stop when the distance is less than the goal distance
        double distance_to_goal = hypot(dx, dy);
        std::cout << "Distance to goal: " << distance_to_goal << std::endl;
        if (distance_to_goal <= 0.00001) 
        {
            std::cout << "Reached Goal" << std::endl;
            break;
        }
        
        if (i > number_of_iterations)
        {
            std::cout << "Max iterations reached" << std::endl;
            break;
        }
       
        x.push_back(state.x);
        y.push_back(state.y);
        yaw.push_back(state.yaw);
        v.push_back(state.v);
        i++;
    }
    // write x, y, yaw to a csv file
    std::ofstream file("lqr_speed_control_path.csv");
    for (size_t i = 0; i < x.size(); i++) {
        file << x[i] << "," << y[i] << "," << yaw[i] << "," << v[i] << std::endl;
    }

    file.close();
    
}


int main() {
  double wheelbase = 3.0;
  int n_path_points = 300;
  ClothoidPathGenerator generator(n_path_points, wheelbase);
  PathData path_data;
  // steering angles vector
  std::vector<double> max_steer_angles = {0, 26.64*M_PI/180.0, 0.0, 0.0, 48.37*M_PI/180.0  , 0.0};
  std::vector<double> max_speeds = {0.6, 2.0, 1.5, 0.6, 2.0, 1.5};
  std::vector<double> stop_speeds = {0.6, 1.5, 0.6, 2.0, 1.5, 0.001};
  std::vector<double> goal_dis = {10, 10, 10, 10, 10, 10};
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
    path_file << full_path.points[i].x << "," << full_path.points[i].y << std::endl;
  }
  do_simulation(full_path);
  return 0;
}

