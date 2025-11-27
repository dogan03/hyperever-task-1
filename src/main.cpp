/**
 * @file main.cpp
 * @brief CartPole MPC Stabilization Simulation - Entry Point
 * 
 * This program demonstrates Model Predictive Control (MPC) for stabilizing
 * an inverted pendulum (cart-pole) system. The simulation:
 * 
 * 1. Initializes a cart-pole with random initial angle (±20 degrees)
 * 2. Uses MPC to compute optimal control forces
 * 3. Visualizes the system in real-time using SFML graphics
 * 4. Logs all state variables and control forces to CSV
 * 5. Runs for 10 seconds or until user closes window
 * 
 * Output:
 * - Real-time graphical display showing cart and pole motion
 * - mpc_data.csv containing time-series data for post-analysis
 * - Console output with initial conditions and final state
 * 
 * Requirements:
 * - SFML library for graphics
 * - Eigen3 library for linear algebra
 * - arial.ttf font file in executable directory
 * 
 * Usage:
 *   ./cartpole_sim
 * 
 * Controls:
 *   ESC - Exit simulation early
 *   Close window - Exit simulation
 * 
 * @author Hyperever Task 1
 * @date 2024
 */

#include <iostream>
#include <exception>
#include "CartPoleSimulation.h"

/**
 * @brief Main entry point for CartPole MPC simulation
 * 
 * Creates and executes a complete CartPole stabilization simulation
 * with MPC control. Handles all exceptions gracefully and reports
 * errors to stderr.
 * 
 * Execution flow:
 * 1. Construct CartPoleSimulation (initializes window, MPC, logging)
 * 2. Run simulation loop until completion or user exit
 * 3. Clean up resources automatically via RAII
 * 
 * @return 0 on successful completion
 * @return 1 if an error occurs (exception caught)
 * 
 * @throws std::exception Caught and reported - program exits gracefully
 * 
 * Common failure modes:
 * - Font file (arial.ttf) not found
 * - Unable to create log file
 * - SFML window creation failure
 * - Graphics driver issues
 */
int main() {
    try {
        // Create simulation with default parameters
        CartPoleSimulation simulation;
        
        // Run until completion or user exit
        simulation.run();
        
        return 0;
    } catch (const std::exception& e) {
        // Report any errors to stderr
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}