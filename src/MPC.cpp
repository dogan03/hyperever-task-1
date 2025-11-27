#include "MPC.h"
#include <iostream>
#include <algorithm>

MPC::MPC(int horizon, double dt)
    : horizon(horizon)
    , dt(dt)
    , max_force(DEFAULT_MAX_FORCE)
    , cart_mass(1.0)
    , pole_mass(0.2)
    , pole_length(0.5)
    , gravity(9.81)
{
    setupSystemMatrices();
    setupCostMatrices();
}

MPC::MPC(double cart_mass, double pole_mass, double pole_length, double gravity,
         int horizon, double dt)
    : horizon(horizon)
    , dt(dt)
    , max_force(DEFAULT_MAX_FORCE)
    , cart_mass(cart_mass)
    , pole_mass(pole_mass)
    , pole_length(pole_length)
    , gravity(gravity)
{
    setupSystemMatrices();
    setupCostMatrices();
}

void MPC::setupSystemMatrices() {
    double total_mass = cart_mass + pole_mass;
    double inertia_term = 4.0 / 3.0 - pole_mass / total_mass;

    Eigen::Matrix4d Ac;
    Ac << 0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, -pole_mass * gravity / total_mass, 0.0,
          0.0, 0.0, 0.0, 1.0,
          0.0, 0.0, total_mass * gravity / (pole_length * total_mass * inertia_term), 0.0;

    Eigen::Matrix<double, 4, 1> Bc;
    Bc << 0.0,
          1.0 / total_mass,
          0.0,
          -1.0 / (pole_length * total_mass * inertia_term);

    Ad = Eigen::Matrix4d::Identity() + Ac * dt;
    Bd = Bc * dt;
}

void MPC::setupCostMatrices() {
    Q = Eigen::Matrix4d::Zero();
    Q(0, 0) = 10.0;    // cart position
    Q(1, 1) = 1.0;     // cart velocity
    Q(2, 2) = 500.0;   // pole angle (high priority)
    Q(3, 3) = 50.0;    // pole angular velocity

    R(0, 0) = 0.01;    // control effort penalty

    Qf = Q * 2.0;      // terminal cost
}

void MPC::setCostWeights(double q_x, double q_vx, double q_theta, double q_vtheta, double r) {
    Q(0, 0) = q_x;
    Q(1, 1) = q_vx;
    Q(2, 2) = q_theta;
    Q(3, 3) = q_vtheta;
    R(0, 0) = r;
    Qf = Q * 2.0;
}

Eigen::Matrix4d MPC::solveRiccatiEquation(const Eigen::Matrix4d& A,
                                           const Eigen::Matrix<double, 4, 1>& B,
                                           const Eigen::Matrix4d& Q,
                                           const Eigen::Matrix<double, 1, 1>& R) const {
    Eigen::Matrix4d P = Qf;

    for (int i = 0; i < horizon; ++i) {
        Eigen::Matrix<double, 1, 4> BtP = B.transpose() * P;
        Eigen::Matrix<double, 1, 1> temp = R + BtP * B;

        if (temp(0, 0) < 1e-10) {
            std::cerr << "[MPC Warning] Riccati equation ill-conditioned at iteration " << i << std::endl;
            break;
        }

        Eigen::Matrix<double, 1, 4> K = temp.inverse() * BtP * A;
        P = Q + A.transpose() * P * (A - B * K);
    }

    return P;
}

Eigen::Matrix<double, 1, 4> MPC::computeOptimalGain(const Eigen::Matrix4d& P) const {
    Eigen::Matrix<double, 1, 4> BtP = Bd.transpose() * P;
    Eigen::Matrix<double, 1, 1> temp = R + BtP * Bd;

    if (temp(0, 0) < 1e-10) {
        std::cerr << "[MPC Warning] Gain computation ill-conditioned" << std::endl;
        return Eigen::Matrix<double, 1, 4>::Zero();
    }

    return temp.inverse() * BtP * Ad;
}

double MPC::computeControl(double x, double vx, double theta, double vtheta) const {
    Eigen::Vector4d state;
    state << x, vx, theta, vtheta;

    Eigen::Matrix4d P = solveRiccatiEquation(Ad, Bd, Q, R);
    Eigen::Matrix<double, 1, 4> K = computeOptimalGain(P);

    double control = -(K * state)(0);
    control = std::clamp(control, -max_force, max_force);

    return control;
}