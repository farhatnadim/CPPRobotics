#include "clothoid_path.hpp"
#include <cmath>
#include <functional>
#include <iostream>
#include <cassert>
ClothoidPathGenerator::ClothoidPathGenerator(int n_path_points, double wheelbase)
    : n_path_points(n_path_points), wheelbase(wheelbase) {
      //assert that n_path_points is greater than 2
       if (n_path_points < 2) {
        throw std::invalid_argument("n_path_points must be at least 2.");
       }
    }

void ClothoidPathGenerator::generateClothoidPaths(const std::vector<Point>& all_points,
                                                  std::vector<ClothoidPath>& paths) {
  for (size_t i = 0; i < all_points.size() - 1; i++) {
    ClothoidPath path;
    generateClothoidPath(all_points[i], all_points[i + 1], path);
    if (!path.points.empty()) {
      paths.push_back(path);
    }
  }
}

void ClothoidPathGenerator::generateClothoidPath(const Point& start_point,
                                                 const Point& goal_point,
                                                 ClothoidPath& path) {
  double dx = goal_point.x - start_point.x;
  double dy = goal_point.y - start_point.y;
  
  // Check for straight-line case
  if (start_point.heading == goal_point.heading && (dx == 0 || dy == 0)) {
    generateStraightPath(start_point, goal_point, path);
    return;
  }

  double r = std::hypot(dx, dy);
  double phi = atan2(dy, dx);
  double phi1 = normalizeAngle(start_point.heading - phi);
  double phi2 = normalizeAngle(goal_point.heading - phi);
  double delta = phi2 - phi1;

  double A;
  try {
    A = solveForRoot(phi1, phi2, delta);
  } catch (const std::exception& e) {
    std::cerr << "Failed to generate clothoid points: " << e.what() << std::endl;
    return;
  }

  double path_length = computePathLength(r, phi1, delta, A);
  double curvature = computeCurvature(delta, A, path_length);
  double curvature_rate = computeCurvatureRate(A, path_length);

  constructPath(start_point, start_point.heading, path_length, curvature,
                curvature_rate, path);
}

void ClothoidPathGenerator::generateStraightPath(const Point& start_point,
                                                 const Point& goal_point,
                                                 ClothoidPath& path) {
  for (int i = 0; i < n_path_points; ++i) {
    double t = static_cast<double>(i) / (n_path_points - 1);
    path.points.push_back({start_point.x + t * (goal_point.x - start_point.x),
                           start_point.y + t * (goal_point.y - start_point.y)});
    path.steering_angles.push_back(0);
    path.yaws.push_back(start_point.heading);
    path.curvatures.push_back(0);

  }
}

void ClothoidPathGenerator::constructPath(const Point& start_point, double start_yaw,
                                          double path_length, double curvature,
                                          double curvature_rate,
                                          ClothoidPath& path) 
{
  double s_step = path_length / (n_path_points - 1);

  for (int i = 0; i < n_path_points; ++i) 
  {
    double s = s_step * i;
    double x = start_point.x +
               s * X(curvature_rate * s * s, curvature * s, start_yaw, 0, 1);
    double y = start_point.y +
               s * Y(curvature_rate * s * s, curvature * s, start_yaw, 0, 1);
    path.points.push_back({x, y});
    path.curvatures.push_back(curvature+curvature_rate*s);
    path.steering_angles.push_back(atan(curvature*wheelbase));

  }

  // Compute yaws for each point
  path.yaws.push_back(start_yaw);
  for (size_t i = 0; i < path.points.size() - 1 ; ++i) 
  {
    double ds = hypot(path.points[i+1].x - path.points[i].x, path.points[i+1].y - path.points[i].y);
    double yaw = path.yaws[i] + (path.curvatures[i+1] + path.curvatures[i]) / 2 * ds;
    path.yaws.push_back(normalizeAngle(yaw));
  }

  // Append the last yaw
  if (!path.yaws.empty()) {
    path.yaws.push_back(path.yaws.back());
  }
}

double ClothoidPathGenerator::X(double a, double b, double c, double lim_a, double lim_b) {
  return integrate(
      [a, b, c](double t) { return std::cos((a / 2) * t * t + b * t + c); },
      lim_a, lim_b);
}

double ClothoidPathGenerator::Y(double a, double b, double c, double lim_a, double lim_b) {
  return integrate(
      [a, b, c](double t) { return std::sin((a / 2) * t * t + b * t + c); },
      lim_a, lim_b);
}

double ClothoidPathGenerator::integrate(const std::function<double(double)>& f,
                                        double a, double b, int n) {
  double h = (b - a) / n;
  double sum = 0.0;
  for (int i = 0; i <= n; ++i) {
    double x = a + i * h;
    double weight = (i == 0 || i == n) ? 0.5 : 1.0;
    sum += weight * f(x);
  }
  return sum * h;
}

double ClothoidPathGenerator::solveForRoot(double theta1, double theta2, double delta) {
  double initial_guess = 3 * (theta1 + theta2);
  double A = fsolve([this, delta, theta1](double A) {
                      return Y(2 * A, delta - A, theta1, 0, 1);
                    },
                    initial_guess);
  return A;
}

double ClothoidPathGenerator::fsolve(const std::function<double(double)>& func,
                                     double x0, double tol, int max_iter) {
  double x = x0;
  for (int i = 0; i < max_iter; ++i) {
    double y = func(x);
    double y_prime = (func(x + tol) - y) / tol;  // Numerical derivative
    if (std::abs(y_prime) < tol)
      throw std::runtime_error("Derivative too small");
    double x_new = x - y / y_prime;
    if (std::abs(x_new - x) < tol)
      return x_new;
    x = x_new;
  }
  throw std::runtime_error("fsolve did not converge");
}

double ClothoidPathGenerator::computePathLength(double r, double theta1, double delta, double A) {
  return r / X(2 * A, delta - A, theta1, 0, 1);
}

double ClothoidPathGenerator::computeCurvature(double delta, double A, double path_length) {
  return (delta - A) / path_length;
}

double ClothoidPathGenerator::computeCurvatureRate(double A, double path_length) {
  return 2 * A / (path_length * path_length);
}

double ClothoidPathGenerator::normalizeAngle(double angle_rad) {
  return std::fmod(angle_rad + M_PI, 2 * M_PI) - M_PI;
}

