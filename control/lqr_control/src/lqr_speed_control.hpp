#pragma once

#include <eigen3/Eigen/Dense>
#include <vector>
#include <tuple>
#include <algorithm>

using namespace Eigen;


constexpr int STATE_DIM = 5;
constexpr int CONTROL_DIM = 2;
constexpr double WEIGHT_STATE = 100.0;



struct LQRParams {
    LQRParams(int max_iter = 100, double eps = 1e-3)
        : max_iter(max_iter), eps(eps) {}
    int max_iter;
    double eps;
};


struct SimulationParams {
    SimulationParams(double L = 1.0, double max_steer = 0.5, double max_speed = 2.0, double T = 10.0, double goal_dis = 0.5, double stop_speed = 0.1, double dt = 0.03, LQRParams lqr_params = LQRParams())
        : L(L), max_steer(max_steer), max_speed(max_speed), T(T), goal_dis(goal_dis), stop_speed(stop_speed), dt(dt), lqr_params(lqr_params) {}
    double L;
    double max_steer;
    double max_speed;
    double T;
    double goal_dis;
    double stop_speed;
    double dt;
    LQRParams lqr_params;
};



struct State {
    double x, y, yaw, v;
    State(double x = 0.0, double y = 0.0, double yaw = 0.0, double v = 0.0)
        : x(x), y(y), yaw(yaw), v(v) {}
};


State update(State state, double a, double delta, const SimulationParams& params);
double pi_2_pi(double angle);
std::tuple<int, double> calc_nearest_index(const State& state, const std::vector<double>& cx,
                                         const std::vector<double>& cy, const std::vector<double>& cyaw);
MatrixXd solve_dare(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R);
std::tuple<MatrixXd, MatrixXd, VectorXd> dlqr(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R);
std::tuple<double, int, double, double, double> lqr_speed_steering_control(const State& state, 
    const std::vector<double>& cx, const std::vector<double>& cy, const std::vector<double>& cyaw, 
    const std::vector<double>& ck, double pe, double pth_e, const std::vector<double>& sp, 
    const MatrixXd& Q, const MatrixXd& R, const SimulationParams& params);
