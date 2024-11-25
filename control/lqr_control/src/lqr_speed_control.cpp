#include <iostream>
#include <cmath>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <algorithm>
#include <tuple>
#include <fstream>
#include <sstream>
#include "lqr_speed_control.hpp"



using namespace std;
using namespace Eigen;

// Parameters

// Add this function prototype after the State struct definition
std::tuple<int, double> calc_nearest_index(const State& state, const std::vector<double>& cx,
                                           const std::vector<double>& cy, const std::vector<double>& cyaw);

State update(State state, double a, double delta, const SimulationParams& params) 
{
    delta = std::clamp(delta, -params.max_steer, params.max_steer); // Clamp steering angle

    state.x += state.v * cos(state.yaw) * params.dt;
    state.y += state.v * sin(state.yaw) * params.dt;
    state.yaw += state.v / params.L * tan(delta) * params.dt;
    state.v += a * params.dt;
    // clamp the speed
    state.v = std::clamp(state.v, 0.0, params.max_speed);
    return state;
}

double pi_2_pi(double angle) {
    while (angle >= M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

tuple<int, double> calc_nearest_index(const State& state, const vector<double>& cx, const vector<double>& cy, const vector<double>& cyaw) 
{
    vector<double> dx, dy, d;
    for (size_t i = 0; i < cx.size(); i++) {
        dx.push_back(state.x - cx[i]);
        dy.push_back(state.y - cy[i]);
        d.push_back(dx[i] * dx[i] + dy[i] * dy[i]);
    }

    auto mind_iter = min_element(d.begin(), d.end());
    int ind = distance(d.begin(), mind_iter);
    double mind = sqrt(*mind_iter);

    double dxl = cx[ind] - state.x;
    double dyl = cy[ind] - state.y;
    double angle = pi_2_pi(cyaw[ind] - atan2(dyl, dxl));
    if (angle < 0) {
        mind *= -1;
    }

    return make_tuple(ind, mind);
}

MatrixXd solve_dare(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R, const LQRParams& params) 
{
    MatrixXd X = Q;
    MatrixXd X_next = Q;
   
    for (int i = 0; i < params.max_iter; ++i) {
        X_next = A.transpose() * X * A - A.transpose() * X * B *
                 (R + B.transpose() * X * B).inverse() * B.transpose() * X * A + Q;
        if ((X_next - X).cwiseAbs().maxCoeff() < params.eps) break;
        X = X_next;
    }

    return X_next;
}

tuple<MatrixXd, MatrixXd, VectorXd> dlqr(const MatrixXd& A, const MatrixXd& B, const MatrixXd& Q, const MatrixXd& R, const LQRParams& params) {
    MatrixXd X = solve_dare(A, B, Q, R, params);
    MatrixXd K = (B.transpose() * X * B + R).inverse() * (B.transpose() * X * A);
    VectorXd eigvals = (A - B * K).eigenvalues().real();

    return make_tuple(K, X, eigvals);
}

tuple<double, int, double, double, double> lqr_speed_steering_control(const State& state, const vector<double>& cx, const vector<double>& cy, const vector<double>& cyaw, const vector<double>& ck, double pe, double pth_e, const vector<double>& sp, const MatrixXd& Q, const MatrixXd& R, const SimulationParams& params) {
    int ind;
    double e;
    try {
        tie(ind, e) = calc_nearest_index(state, cx, cy, cyaw);
    } catch (const std::exception& e) {
        std::cerr << "Error calculating nearest index: " << e.what() << std::endl;
        return make_tuple(0.0, 0, 0.0, 0.0, 0.0);
    }

    double tv = sp[ind];
    double k = ck[ind];
    double v = state.v;
    double th_e = pi_2_pi(state.yaw - cyaw[ind]);

    Matrix<double, STATE_DIM, STATE_DIM> A;
    A << 1.0, params.dt, 0.0, 0.0, 0.0,
         0.0, 0.0, v, 0.0, 0.0,
         0.0, 0.0, 1.0, params.dt, 0.0,
         0.0, 0.0, 0.0, 0.0, 0.0,
         0.0, 0.0, 0.0, 0.0, 1.0;

    Matrix<double, STATE_DIM, CONTROL_DIM> B;
    B << 0.0, 0.0,
         0.0, 0.0,
         0.0, 0.0,
         v / params.L, 0.0,
         0.0, params.dt;

    MatrixXd K;
    MatrixXd X, eigvals;
    tie(K, X, eigvals) = dlqr(A, B, Q, R, params.lqr_params);
    // print the K matrix  for each iteration
    //cout << "K: " << K << endl;
    VectorXd x(5);
    x << e, (e - pe) / params.dt, th_e, (th_e - pth_e) / params.dt, v - tv;

    VectorXd ustar = -K * x;

    double ff = atan2(params.L * k, 1.0);
    double fb = pi_2_pi(ustar(0));
    double delta = ff + fb;
    double accel = ustar(1);

    return make_tuple(delta, ind, e, th_e, accel);
}

