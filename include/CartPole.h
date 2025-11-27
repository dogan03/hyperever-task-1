#ifndef CARTPOLE_H
#define CARTPOLE_H

#include <cmath>

/**
 * @brief Physical parameters for the CartPole system
 * 
 * Defines the physical properties of the cart-pole inverted pendulum system.
 * All values use SI units (kg, m, m/s²).
 */
struct CartPoleParams {
    double M = 1.0;           ///< Cart mass (kg)
    double m = 0.2;           ///< Pole mass (kg)
    double l = 0.5;           ///< Pole length from pivot to center of mass (m)
    double g = 9.81;          ///< Gravitational acceleration (m/s²)
    double init_theta = 0.0;  ///< Initial pole angle in radians (0 = upright)
};

/**
 * @brief Inverted pendulum (cart-pole) dynamics simulation
 * 
 * Implements the nonlinear dynamics of a cart-pole system using Euler integration.
 * The cart moves horizontally along a track while the pole is attached via a
 * frictionless pivot. The system is underactuated (control force only on cart).
 * 
 * State variables:
 * - x: Cart position (m)
 * - vx: Cart velocity (m/s)
 * - theta: Pole angle from vertical (rad, 0 = upright, positive = clockwise)
 * - vtheta: Pole angular velocity (rad/s)
 * 
 * The dynamics are derived from Lagrangian mechanics:
 * - Cart acceleration: (F + ml(θ̇² sin(θ) - θ̈ cos(θ))) / (M + m)
 * - Pole angular acceleration: (g sin(θ) + cos(θ)(-F - ml θ̇² sin(θ))/(M+m)) / (l(4/3 - m cos²(θ)/(M+m)))
 */
class CartPole {
private:
    CartPoleParams params;  ///< Physical parameters of the system
    
    /**
     * @brief Computes pole angular acceleration from current state
     * @param force Horizontal force applied to cart (N)
     * @return Pole angular acceleration (rad/s²)
     */
    double computeThetaAcceleration(double force) const;
    
    /**
     * @brief Computes cart acceleration from pole dynamics and applied force
     * @param theta_accel Pole angular acceleration (rad/s²)
     * @param force Horizontal force applied to cart (N)
     * @return Cart horizontal acceleration (m/s²)
     */
    double computeXAcceleration(double theta_accel, double force) const;
    
    /**
     * @brief Updates velocities using Euler integration
     * @param x_accel Cart acceleration (m/s²)
     * @param theta_accel Pole angular acceleration (rad/s²)
     * @param dt Time step (s)
     */
    void updateVelocities(double x_accel, double theta_accel, double dt);
    
    /**
     * @brief Updates positions using Euler integration
     * @param dt Time step (s)
     */
    void updatePositions(double dt);
    
    /**
     * @brief Normalizes theta to [-π, π] range
     */
    void normalizeTheta();

public:
    double x;       ///< Cart position (m)
    double vx;      ///< Cart velocity (m/s)
    double theta;   ///< Pole angle from vertical (rad, 0 = upright)
    double vtheta;  ///< Pole angular velocity (rad/s)

    /**
     * @brief Constructs a CartPole system with specified parameters
     * @param p Physical parameters (defaults to standard values)
     */
    explicit CartPole(const CartPoleParams& p = CartPoleParams());
    
    /**
     * @brief Advances the simulation by one time step
     * 
     * Integrates the nonlinear dynamics forward using Euler method.
     * Enforces boundary constraints on cart position ([-250, 250]).
     * 
     * @param force Horizontal control force applied to cart (N)
     * @param dt Time step for integration (s)
     */
    void update(double force, double dt);
    
    /**
     * @brief Gets cart position
     * @return Cart position (m)
     */
    double getX() const { return x; }
    
    /**
     * @brief Gets cart velocity
     * @return Cart velocity (m/s)
     */
    double getVX() const { return vx; }
    
    /**
     * @brief Gets pole angle
     * @return Pole angle from vertical (rad)
     */
    double getTheta() const { return theta; }
    
    /**
     * @brief Gets pole angular velocity
     * @return Pole angular velocity (rad/s)
     */
    double getVTheta() const { return vtheta; }
    
    /**
     * @brief Gets system physical parameters
     * @return Reference to CartPoleParams structure
     */
    const CartPoleParams& getParams() const { return params; }
};

#endif