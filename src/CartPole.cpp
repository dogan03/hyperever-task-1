#include "CartPole.h"

CartPole::CartPole(const CartPoleParams& p) 
    : params(p)
    , x(0.0)
    , vx(0.0)
    , theta(p.init_theta)
    , vtheta(0.0)
{}

void CartPole::update(double force, double dt) {
    if ((x <= -250.0 && force < 0.0) || (x >= 250.0 && force > 0.0)) {
        force = 0.0;
    }
    
    double theta_accel = computeThetaAcceleration(force);
    double x_accel = computeXAcceleration(theta_accel, force);
    
    updateVelocities(x_accel, theta_accel, dt);
    updatePositions(dt);
}

double CartPole::computeThetaAcceleration(double force) const {
    double sin_theta = std::sin(theta);
    double cos_theta = std::cos(theta);
    double total_mass = params.M + params.m;
    
    double numerator = params.g * sin_theta + 
                      cos_theta * (-force - params.m * params.l * vtheta * vtheta * sin_theta) / total_mass;
    
    double denominator = params.l * (4.0/3.0 - params.m * cos_theta * cos_theta / total_mass);
    
    return numerator / denominator;
}

double CartPole::computeXAcceleration(double theta_accel, double force) const {
    double sin_theta = std::sin(theta);
    double cos_theta = std::cos(theta);
    
    double numerator = force + params.m * params.l * (vtheta * vtheta * sin_theta - theta_accel * cos_theta);
    double denominator = params.M + params.m;
    
    return numerator / denominator;
}

void CartPole::updateVelocities(double x_accel, double theta_accel, double dt) {
    vx += x_accel * dt;
    vtheta += theta_accel * dt;
}

void CartPole::updatePositions(double dt) {
    x += vx * dt;
    theta += vtheta * dt;
    normalizeTheta();
}

void CartPole::normalizeTheta() {
    while (theta > M_PI) {
        theta -= 2.0 * M_PI;
    }
    while (theta < -M_PI) {
        theta += 2.0 * M_PI;
    }
}