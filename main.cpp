#include "clothoid_path.hpp"
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>
#include "lqr_control/lqr_speed_control.hpp"

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

void do_simulation(const ClothoidPath& path, const std::vector<double>& speed_profile, const std::vector<double>& goal) 
{
    // if path is empty, return
    if (path.points.empty()) {
        std::cerr << "Path contains no points" << std::endl;
        return;
    }
    SimulationParams params;
    // LQR matrices
    Matrix<double, STATE_DIM, STATE_DIM> lqr_Q = Matrix<double, STATE_DIM, STATE_DIM>::Identity() * WEIGHT_STATE;  // Increase weights
    Matrix<double, CONTROL_DIM, CONTROL_DIM> lqr_R = Matrix<double, CONTROL_DIM, CONTROL_DIM>::Identity();
    State state(path.points[0].x, path.points[0].y, path.yaws[0], 0.0);
    double time = 0.0;

    std::vector<double> x, y, yaw, v, t;
    x.push_back(state.x);
    y.push_back(state.y);
    yaw.push_back(state.yaw);
    v.push_back(state.v);
    t.push_back(0.0);

    double e = 0.0, e_th = 0.0;

    std::vector<double> path_x;
    std::vector<double> path_y;
    for (const auto& point : path.points) {
        path_x.push_back(point.x);
        path_y.push_back(point.y);
    }
    
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

        if (abs(state.v) <= params.stop_speed) 
        {
            target_ind++;
        }

        time += params.dt;

        double dx = state.x - goal[0];
        double dy = state.y - goal[1];
        if (hypot(dx, dy) <= params.goal_dis) 
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
    std::ofstream file("lqr_speed_control.csv");
    for (size_t i = 0; i < x.size(); i++) {
        file << t[i] << "," << x[i] << "," << y[i] << "," << yaw[i] << "," << v[i] << std::endl;
    }
    file.close();
    
}


int main() {
  double wheelbase = 3.0;
  int n_path_points = 300;
  ClothoidPathGenerator generator(n_path_points, wheelbase);
  PathData path_data;
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

  std::vector<ClothoidPath> paths;
  generator.generateClothoidPaths(all_points, paths);
  
  // Add error checking
  if (paths.empty()) {
    std::cerr << "Failed to generate any valid clothoid paths" << std::endl;
    return 1;
  }

  for (const auto& path : paths) 
  {
    // Add size checking
    if (path.points.empty()) {
      std::cerr << "Path contains no points" << std::endl;
      continue;
    }

    std::vector<double> speed_profile(path.points.size(), 0.6);
    std::vector<double> goal = {end_node.x, end_node.y};
    do_simulation(path, speed_profile, goal);
  }

  return 0;
}

