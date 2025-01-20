#include <iostream>
#include <math.h>
#include <tuple>
#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Dense>

using namespace std;
using namespace Eigen;

float measurements[3] = { 1, 2, 3 };

tuple<MatrixXf, MatrixXf> kalman_filter(MatrixXf x, MatrixXf P, MatrixXf u, MatrixXf F, MatrixXf H, MatrixXf R, MatrixXf I)
{
    for (int n = 0; n < sizeof(measurements) / sizeof(measurements[0]); n++) {
        // Measurement Update
        // Initialize and Compute Z, y, S, K, x, and P

        VectorXf Z(1);
        // I Grab the current measurement
        Z(0) = measurements[n];

        VectorXf y(1);
        // II Compute the residuals
        y = Z - H * x;
        // III Compute the measurement covariance
        MatrixXf S = H * P * H.transpose() + R;

        // III Compute the Kalman Gain
        MatrixXf K = P * H.transpose() * S.inverse();
        // IV Update the state
        x = x + K *y;
        // The state covariance is updated
        P = (I- K* H)* P;
        // Prediction
        // Perform prediction calculations here
        x = F * x +u;
        P = F*P*F.transpose();
    
    }

    return make_tuple(x, P);
}

int main()
{
    MatrixXf x(2, 1); // Initial state (location and velocity) 
    x << 0,
         0; 

    MatrixXf P(2, 2); // Initial Uncertainty
    P << 100, 0, 
         0, 100; 

    MatrixXf u(2, 1); // External Motion
    u << 0,
         0; 

    MatrixXf F(2, 2); // Next State Function
    F << 1, 1,
         0, 1; 

    MatrixXf H(1, 2); // Measurement Function
    H << 1, 0; 

    MatrixXf R(1, 1); // Measurement Uncertainty
    R << 1;

    MatrixXf I = MatrixXf::Identity(2, 2); // Identity Matrix

    tie(x, P) = kalman_filter(x, P, u, F, H, R, I);
    cout << "x= \n" << x << endl;
    cout << "P= \n" << P << endl;

    return 0;
}