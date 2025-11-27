#include "CartPoleSimulation.h"
#include <random>
#include <iostream>
#include <iomanip>
#include <stdexcept>

CartPoleSimulation::CartPoleSimulation()
    : mpc(MPC_HORIZON, PHYSICS_DT)
    , elapsed_time(0.0f)
{
    initializeCartPole();
    initializeLogging();
}

CartPoleSimulation::~CartPoleSimulation() {
    if (logfile.is_open()) {
        logfile.close();
    }
}

double CartPoleSimulation::generateRandomInitialAngle() const {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<double> dist(-INITIAL_ANGLE_RANGE, INITIAL_ANGLE_RANGE);
    return dist(gen);
}

void CartPoleSimulation::initializeCartPole() {
    double initial_angle_deg = generateRandomInitialAngle();
    double initial_angle_rad = initial_angle_deg * M_PI / 180.0;

    window.cartpole.theta = initial_angle_rad;
    window.cartpole.x = 0.0;
    window.cartpole.vx = 0.0;
    window.cartpole.vtheta = 0.0;

    std::cout << "Starting with initial angle: "
              << std::fixed << std::setprecision(2)
              << initial_angle_deg << " degrees" << std::endl;
}

void CartPoleSimulation::initializeLogging() {
    logfile.open(LOG_FILENAME);
    if (!logfile.is_open()) {
        throw std::runtime_error("Failed to open log file: " + std::string(LOG_FILENAME));
    }
    logfile << "time,x,theta,vx,vtheta,force\n";
    logfile << std::fixed << std::setprecision(6);
}

void CartPoleSimulation::logData(double force) {
    logfile << elapsed_time << ","
            << window.cartpole.x << ","
            << window.cartpole.theta << ","
            << window.cartpole.vx << ","
            << window.cartpole.vtheta << ","
            << force << "\n";
}

double CartPoleSimulation::computeControl() const {
    return mpc.computeControl(
        window.cartpole.x,
        window.cartpole.vx,
        window.cartpole.theta,
        window.cartpole.vtheta
    );
}

void CartPoleSimulation::applyBoundaryConstraints() {
    if (window.cartpole.x < -POSITION_LIMIT) {
        window.cartpole.x = -POSITION_LIMIT;
        window.cartpole.vx = 0.0;
    } else if (window.cartpole.x > POSITION_LIMIT) {
        window.cartpole.x = POSITION_LIMIT;
        window.cartpole.vx = 0.0;
    }
}

void CartPoleSimulation::render() {
    window.window.clear(window.background_color);
    window.drawTitle();
    window.drawSimulationWindow();
    window.drawTicks();
    window.drawRoad();
    window.drawTheta();
    window.drawX();
    window.drawCart();
    window.drawPole();
    window.window.display();
}

void CartPoleSimulation::handleEvents() {
    sf::Event event;
    while (window.window.pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            window.window.close();
        } else if (event.type == sf::Event::KeyPressed && event.key.code == sf::Keyboard::Escape) {
            window.window.close();
        }
    }
}

bool CartPoleSimulation::shouldContinue() const {
    return window.window.isOpen() && elapsed_time < SIMULATION_DURATION;
}

void CartPoleSimulation::printFinalState() const {
    std::cout << "\nSimulation complete!" << std::endl;
    std::cout << "Final state:" << std::endl;
    std::cout << "  Time: " << std::fixed << std::setprecision(2) << elapsed_time << " seconds" << std::endl;
    std::cout << "  Position: " << std::setprecision(3) << window.cartpole.x << std::endl;
    std::cout << "  Angle: " << std::setprecision(2) << (window.cartpole.theta * 180.0 / M_PI) << " degrees" << std::endl;
    std::cout << "Data saved to " << LOG_FILENAME << std::endl;
}

void CartPoleSimulation::run() {
    while (shouldContinue()) {
        handleEvents();

        double force = computeControl();
        logData(force);

        window.cartpole.update(force, PHYSICS_DT);
        applyBoundaryConstraints();

        render();

        elapsed_time += PHYSICS_DT;
    }

    printFinalState();
}