#ifndef CARTPOLE_SIMULATION_H
#define CARTPOLE_SIMULATION_H

#include <fstream>
#include "MainWindow.h"
#include "MPC.h"

/**
 * @brief Manages the CartPole MPC stabilization simulation
 * 
 * Orchestrates the complete simulation loop including:
 * - Random initial angle generation within specified range
 * - MPC controller for pole stabilization
 * - Real-time visualization using SFML
 * - Data logging to CSV for post-analysis
 * - Event handling and user interaction
 * 
 * The simulation runs for a fixed duration (default 10s) and logs
 * all state variables and control forces at each timestep.
 */
class CartPoleSimulation {
private:
    static constexpr double PHYSICS_DT = 0.02;              ///< Physics timestep (s) - 50 Hz update rate
    static constexpr double SIMULATION_DURATION = 10.0;      ///< Total simulation time (s)
    static constexpr double POSITION_LIMIT = 250.0;          ///< Cart position limits ±250 units
    static constexpr double INITIAL_ANGLE_RANGE = 20.0;      ///< Initial angle range ±20 degrees
    static constexpr int MPC_HORIZON = 30;                   ///< MPC prediction horizon (timesteps)
    static constexpr const char* LOG_FILENAME = "mpc_data.csv"; ///< Output CSV filename

    MainWindow window;      ///< SFML window for visualization
    MPC mpc;               ///< Model Predictive Controller
    std::ofstream logfile; ///< CSV file stream for data logging
    float elapsed_time;    ///< Current simulation time (s)

    /**
     * @brief Generates random initial pole angle within range
     * @return Random angle in degrees within [-20, 20]
     */
    double generateRandomInitialAngle() const;
    
    /**
     * @brief Initializes CartPole state with random angle
     * 
     * Sets initial conditions:
     * - theta: Random angle from generateRandomInitialAngle()
     * - x, vx, vtheta: All set to zero
     * Prints initial angle to console.
     */
    void initializeCartPole();
    
    /**
     * @brief Opens CSV log file and writes header
     * @throws std::runtime_error if file cannot be opened
     */
    void initializeLogging();
    
    /**
     * @brief Logs current state and control force to CSV
     * @param force Control force applied at current timestep (N)
     * 
     * Writes: time, x, theta, vx, vtheta, force
     */
    void logData(double force);
    
    /**
     * @brief Computes optimal control force using MPC
     * @return Optimal control force (N)
     * 
     * Queries MPC controller with current CartPole state to
     * compute stabilizing control action.
     */
    double computeControl() const;
    
    /**
     * @brief Enforces cart position boundaries
     * 
     * Clamps cart position to [-250, 250] and zeros velocity
     * if boundary is reached to prevent drift.
     */
    void applyBoundaryConstraints();
    
    /**
     * @brief Renders current simulation state to window
     * 
     * Draws all visual elements: title, simulation area, cart,
     * pole, position/angle indicators, and boundary markers.
     */
    void render();
    
    /**
     * @brief Processes window events (close, keyboard input)
     * 
     * Handles:
     * - Window close button
     * - ESC key press (closes simulation)
     */
    void handleEvents();
    
    /**
     * @brief Checks if simulation should continue
     * @return true if window open and time < duration, false otherwise
     */
    bool shouldContinue() const;
    
    /**
     * @brief Prints final simulation statistics to console
     * 
     * Displays:
     * - Final elapsed time
     * - Final cart position
     * - Final pole angle
     * - Output filename confirmation
     */
    void printFinalState() const;

public:
    /**
     * @brief Constructs and initializes the simulation
     * 
     * Sets up:
     * - MPC controller with specified horizon and timestep
     * - Random initial CartPole state
     * - Data logging infrastructure
     * - SFML visualization window
     * 
     * @throws std::runtime_error if initialization fails
     */
    CartPoleSimulation();
    
    /**
     * @brief Destructor - ensures log file is properly closed
     */
    ~CartPoleSimulation();

    /**
     * @brief Executes the main simulation loop
     * 
     * Runs until window closes or duration expires:
     * 1. Process user events
     * 2. Compute MPC control
     * 3. Log data
     * 4. Update dynamics
     * 5. Apply constraints
     * 6. Render visualization
     * 7. Advance time
     * 
     * Prints final state upon completion.
     */
    void run();
};

#endif