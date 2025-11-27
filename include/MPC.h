#ifndef MPC_H
#define MPC_H

#include <Eigen/Dense>

/**
 * @brief Model Predictive Controller for CartPole stabilization
 * 
 * Implements a finite-horizon Linear Quadratic Regulator (LQR) using
 * Model Predictive Control (MPC) strategy. The controller:
 * 
 * 1. Linearizes CartPole dynamics around upright equilibrium (θ=0)
 * 2. Discretizes the continuous-time system using Euler method
 * 3. Solves finite-horizon Riccati equation backwards over prediction horizon
 * 4. Computes optimal state feedback gain K
 * 5. Applies control law: u = -K*x (with saturation)
 * 
 * State vector: x = [cart_position, cart_velocity, pole_angle, pole_angular_velocity]ᵀ
 * Control input: u = horizontal force on cart (N)
 * 
 * Cost function minimizes:
 * J = Σ(xₖᵀQxₖ + uₖᵀRuₖ) + xₙᵀQfxₙ
 * 
 * The controller operates in receding horizon fashion - computes full
 * control sequence but applies only the first action, then re-optimizes.
 */
class MPC {
private:
    static constexpr int DEFAULT_HORIZON = 30;        ///< Default prediction horizon (timesteps)
    static constexpr double DEFAULT_DT = 0.02;        ///< Default timestep (s) - 50 Hz
    static constexpr double DEFAULT_MAX_FORCE = 50.0; ///< Default force saturation limit (N)

    int horizon;        ///< Prediction horizon N (number of timesteps to look ahead)
    double dt;          ///< Discretization timestep (s)
    double max_force;   ///< Control force saturation limit ±max_force (N)

    double cart_mass;   ///< Cart mass M (kg)
    double pole_mass;   ///< Pole mass m (kg)
    double pole_length; ///< Pole length from pivot to COM l (m)
    double gravity;     ///< Gravitational acceleration g (m/s²)

    Eigen::Matrix4d Q;                  ///< State cost matrix (4x4) - diagonal weights for [x, vx, θ, vθ]
    Eigen::Matrix<double, 1, 1> R;      ///< Control cost matrix (1x1) - penalty on control effort
    Eigen::Matrix4d Qf;                 ///< Terminal state cost matrix (4x4) - cost at horizon end

    Eigen::Matrix4d Ad;                 ///< Discrete-time state transition matrix A (4x4)
    Eigen::Matrix<double, 4, 1> Bd;     ///< Discrete-time control input matrix B (4x1)

    /**
     * @brief Linearizes and discretizes CartPole dynamics
     * 
     * Linearizes nonlinear dynamics around θ=0 (upright) to obtain:
     * ẋ = Ac*x + Bc*u (continuous-time)
     * 
     * Then discretizes using first-order Euler approximation:
     * Ad = I + Ac*dt
     * Bd = Bc*dt
     * 
     * Resulting discrete-time model: x[k+1] = Ad*x[k] + Bd*u[k]
     */
    void setupSystemMatrices();
    
    /**
     * @brief Initializes cost function weight matrices
     * 
     * Sets default weights:
     * - Q(0,0) = 10.0   (cart position penalty)
     * - Q(1,1) = 1.0    (cart velocity penalty)
     * - Q(2,2) = 500.0  (pole angle penalty - highest priority)
     * - Q(3,3) = 50.0   (pole angular velocity penalty - damping)
     * - R = 0.01        (control effort penalty)
     * - Qf = 2*Q        (terminal cost - double state penalty)
     */
    void setupCostMatrices();
    
    /**
     * @brief Solves discrete-time finite-horizon Riccati equation
     * 
     * Computes cost-to-go matrix P by iterating backwards from terminal cost Qf:
     * P[N] = Qf
     * P[k] = Q + AᵀP[k+1](A - B*K[k+1]) for k = N-1, ..., 0
     * 
     * where K[k] = (R + BᵀP[k+1]B)⁻¹ * BᵀP[k+1]A
     * 
     * @param A Discrete-time state transition matrix (4x4)
     * @param B Discrete-time control input matrix (4x1)
     * @param Q State cost matrix (4x4)
     * @param R Control cost matrix (1x1)
     * @return Final cost-to-go matrix P (4x4) after backward iteration
     */
    Eigen::Matrix4d solveRiccatiEquation(const Eigen::Matrix4d& A,
                                          const Eigen::Matrix<double, 4, 1>& B,
                                          const Eigen::Matrix4d& Q,
                                          const Eigen::Matrix<double, 1, 1>& R) const;
    
    /**
     * @brief Computes optimal state feedback gain from cost-to-go matrix
     * 
     * Calculates K = (R + BᵀPB)⁻¹ * BᵀPA
     * 
     * @param P Cost-to-go matrix from Riccati equation (4x4)
     * @return Optimal feedback gain K (1x4) for control law u = -K*x
     */
    Eigen::Matrix<double, 1, 4> computeOptimalGain(const Eigen::Matrix4d& P) const;

public:
    /**
     * @brief Constructs MPC controller with default physical parameters
     * 
     * Uses standard CartPole parameters:
     * - Cart mass: 1.0 kg
     * - Pole mass: 0.2 kg
     * - Pole length: 0.5 m
     * - Gravity: 9.81 m/s²
     * 
     * @param horizon Prediction horizon in timesteps (default 30)
     * @param dt Discretization timestep in seconds (default 0.02)
     */
    MPC(int horizon = DEFAULT_HORIZON, double dt = DEFAULT_DT);
    
    /**
     * @brief Constructs MPC controller with custom physical parameters
     * 
     * Allows specification of CartPole physical properties to match
     * the actual system being controlled.
     * 
     * @param cart_mass Cart mass M (kg)
     * @param pole_mass Pole mass m (kg)
     * @param pole_length Pole length from pivot to COM l (m)
     * @param gravity Gravitational acceleration g (m/s²)
     * @param horizon Prediction horizon in timesteps (default 30)
     * @param dt Discretization timestep in seconds (default 0.02)
     */
    MPC(double cart_mass, double pole_mass, double pole_length, double gravity,
        int horizon = DEFAULT_HORIZON, double dt = DEFAULT_DT);

    /**
     * @brief Computes optimal control force for current state
     * 
     * Executes full MPC optimization:
     * 1. Solves Riccati equation over prediction horizon
     * 2. Computes optimal feedback gain K
     * 3. Calculates control: u = -K*x
     * 4. Saturates control to ±max_force
     * 
     * @param x Cart position (m)
     * @param vx Cart velocity (m/s)
     * @param theta Pole angle from vertical (rad, 0 = upright)
     * @param vtheta Pole angular velocity (rad/s)
     * @return Optimal saturated control force (N) in range [-max_force, max_force]
     */
    double computeControl(double x, double vx, double theta, double vtheta) const;
    
    /**
     * @brief Sets maximum control force limit
     * @param force Maximum absolute force (N)
     */
    void setMaxForce(double force) { max_force = force; }
    
    /**
     * @brief Gets current maximum control force limit
     * @return Maximum absolute force (N)
     */
    double getMaxForce() const { return max_force; }
    
    /**
     * @brief Updates cost function weight matrices
     * 
     * Allows runtime tuning of controller behavior by adjusting
     * penalties on state deviations and control effort.
     * 
     * @param q_x Cart position weight (higher = penalize position error more)
     * @param q_vx Cart velocity weight (higher = penalize velocity more)
     * @param q_theta Pole angle weight (higher = prioritize upright pole)
     * @param q_vtheta Pole angular velocity weight (higher = dampen oscillations)
     * @param r Control effort weight (higher = prefer smaller control forces)
     */
    void setCostWeights(double q_x, double q_vx, double q_theta, double q_vtheta, double r);
    
    /**
     * @brief Gets prediction horizon length
     * @return Horizon N (number of timesteps)
     */
    int getHorizon() const { return horizon; }
    
    /**
     * @brief Gets discretization timestep
     * @return Timestep dt (seconds)
     */
    double getTimeStep() const { return dt; }
};

#endif