#pragma once

#include <functional>
#include <stdexcept>
#include <string>
#include <tuple>
#include <vector>
#include <algorithm>

struct Point {
  double x, y;
  double heading;
};

struct ClothoidPath {
  std::vector<Point> points;
  std::vector<double> steering_angles;
  std::vector<double> yaws;
  std::vector<double> curvatures;
  std::vector<double> s;
};

class ClothoidPathGenerator {
 public:
  ClothoidPathGenerator(int n_path_points, double wheelbase);

  void generateClothoidPaths(const std::vector<Point>& all_points,
                             ClothoidPath & full_path);

  void generateClothoidPath(const Point& start_point, const Point& goal_point,
                            ClothoidPath& path);

 private:
  int n_path_points;
  double wheelbase;

  void generateStraightPath(const Point& start_point, const Point& goal_point,
                            ClothoidPath& path);

  void constructPath(const Point& start_point, double start_yaw, double L,
                     double curvature, double curvature_rate,
                     ClothoidPath& path);

  double X(double a, double b, double c, double lim_a, double lim_b);
  double Y(double a, double b, double c, double lim_a, double lim_b);
  double integrate(const std::function<double(double)>& f, double a, double b,
                   int n = 1000);
  double solveForRoot(double theta1, double theta2, double delta);
  double fsolve(const std::function<double(double)>& func, double x0,
                double tol = 1e-6, int max_iter = 100);
  double computePathLength(double r, double theta1, double delta, double A);
  double computeCurvature(double delta, double A, double L);
  double computeCurvatureRate(double A, double L);
  double normalizeAngle(double angle_rad);
};

