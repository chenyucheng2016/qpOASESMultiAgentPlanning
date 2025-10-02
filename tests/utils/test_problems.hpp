/*
 *  Real-World MPC Test Problems
 *  
 *  Collection of realistic MPC problems for testing:
 *  - Quadrotor stabilization
 *  - Vehicle path tracking
 *  - Inverted pendulum
 *  
 *  Each provides linearized dynamics and cost matrices
 */

#ifndef TEST_PROBLEMS_HPP
#define TEST_PROBLEMS_HPP

#include <qpOASES.hpp>
#include <cmath>

USING_NAMESPACE_QPOASES

namespace RealWorldProblems {

// ============================================================================
// QUADROTOR HOVER STABILIZATION
// ============================================================================

class QuadrotorHover {
public:
    static constexpr int nx = 12;  // [x,y,z, vx,vy,vz, roll,pitch,yaw, p,q,r]
    static constexpr int nu = 4;   // [T1, T2, T3, T4] - rotor thrusts
    
    static void getSystemMatrices(real_t* A, real_t* B, real_t dt = 0.1) {
        // Linearized quadrotor dynamics around hover
        // From: Beard & McLain, "Small Unmanned Aircraft"
        
        real_t m = 0.5;      // mass (kg)
        real_t g = 9.81;     // gravity (m/s²)
        real_t Ixx = 0.01;   // moment of inertia
        real_t Iyy = 0.01;
        real_t Izz = 0.02;
        real_t L = 0.25;     // arm length (m)
        
        // Continuous-time dynamics: ẋ = Ac*x + Bc*u
        // Position dynamics: ẍ = (1/m)*[roll*T_total, pitch*T_total, 0]
        // Attitude dynamics: Euler equations
        
        // Simplified linearized continuous-time A matrix (12x12)
        real_t Ac[144] = {0};
        
        // Position to velocity (identity in upper-right 3x3)
        Ac[0*12 + 3] = 1.0;  // x_dot = vx
        Ac[1*12 + 4] = 1.0;  // y_dot = vy
        Ac[2*12 + 5] = 1.0;  // z_dot = vz
        
        // Velocity to attitude coupling
        Ac[3*12 + 7] = g;    // vx_dot = g*pitch
        Ac[4*12 + 6] = -g;   // vy_dot = -g*roll
        
        // Attitude to angular velocity
        Ac[6*12 + 9] = 1.0;   // roll_dot = p
        Ac[7*12 + 10] = 1.0;  // pitch_dot = q
        Ac[8*12 + 11] = 1.0;  // yaw_dot = r
        
        // Continuous-time B matrix (12x4)
        real_t Bc[48] = {0};
        
        // Thrust affects vertical acceleration
        Bc[5*4 + 0] = 1.0/m;  // z_ddot from T1
        Bc[5*4 + 1] = 1.0/m;  // z_ddot from T2
        Bc[5*4 + 2] = 1.0/m;  // z_ddot from T3
        Bc[5*4 + 3] = 1.0/m;  // z_ddot from T4
        
        // Roll torque (differential thrust)
        Bc[9*4 + 0] = L/(Ixx);
        Bc[9*4 + 2] = -L/(Ixx);
        
        // Pitch torque
        Bc[10*4 + 1] = L/(Iyy);
        Bc[10*4 + 3] = -L/(Iyy);
        
        // Yaw torque (diagonal motors)
        Bc[11*4 + 0] = 0.1/Izz;
        Bc[11*4 + 1] = -0.1/Izz;
        Bc[11*4 + 2] = 0.1/Izz;
        Bc[11*4 + 3] = -0.1/Izz;
        
        // Discretize: A = I + dt*Ac, B = dt*Bc (Euler approximation)
        for (int i = 0; i < 144; ++i) {
            A[i] = (i % 13 == 0) ? 1.0 : 0.0;  // Identity
            A[i] += dt * Ac[i];
        }
        
        for (int i = 0; i < 48; ++i) {
            B[i] = dt * Bc[i];
        }
    }
    
    static void getCostMatrices(real_t* Q, real_t* R) {
        // Cost: penalize position error and attitude error
        for (int i = 0; i < 144; ++i) Q[i] = 0.0;
        
        // Position cost
        Q[0*12 + 0] = 10.0;  // x
        Q[1*12 + 1] = 10.0;  // y
        Q[2*12 + 2] = 10.0;  // z
        
        // Velocity cost
        Q[3*12 + 3] = 1.0;   // vx
        Q[4*12 + 4] = 1.0;   // vy
        Q[5*12 + 5] = 1.0;   // vz
        
        // Attitude cost
        Q[6*12 + 6] = 5.0;   // roll
        Q[7*12 + 7] = 5.0;   // pitch
        Q[8*12 + 8] = 1.0;   // yaw
        
        // Angular velocity cost
        Q[9*12 + 9] = 0.1;   // p
        Q[10*12 + 10] = 0.1; // q
        Q[11*12 + 11] = 0.1; // r
        
        // Input cost (control effort)
        for (int i = 0; i < 16; ++i) R[i] = 0.0;
        R[0*4 + 0] = 0.1;
        R[1*4 + 1] = 0.1;
        R[2*4 + 2] = 0.1;
        R[3*4 + 3] = 0.1;
    }
};

// ============================================================================
// VEHICLE PATH TRACKING (KINEMATIC BICYCLE MODEL)
// ============================================================================

class VehicleTracking {
public:
    static constexpr int nx = 4;   // [x, y, heading, velocity]
    static constexpr int nu = 2;   // [steering, acceleration]
    
    static void getSystemMatrices(real_t* A, real_t* B, real_t dt = 0.1, real_t v0 = 5.0) {
        // Kinematic bicycle model linearized around straight-line motion
        // From: Rajamani, "Vehicle Dynamics and Control"
        
        real_t L = 2.7;  // wheelbase (m)
        
        // Continuous-time linearization around (v=v0, δ=0)
        // ẋ = v*cos(θ)     ≈ v
        // ẏ = v*sin(θ)     ≈ v*θ
        // θ̇ = v/L * tan(δ) ≈ v/L * δ
        // v̇ = a
        
        // A matrix (4x4)
        for (int i = 0; i < 16; ++i) A[i] = 0.0;
        
        // Identity diagonal
        A[0*4 + 0] = 1.0;
        A[1*4 + 1] = 1.0;
        A[2*4 + 2] = 1.0;
        A[3*4 + 3] = 1.0;
        
        // Coupling terms
        A[0*4 + 2] = -dt * v0;        // x depends on heading
        A[1*4 + 2] = dt * v0;         // y depends on heading
        A[1*4 + 3] = dt;              // y depends on velocity
        
        // B matrix (4x2)
        for (int i = 0; i < 8; ++i) B[i] = 0.0;
        
        B[2*2 + 0] = dt * v0 / L;     // heading from steering
        B[3*2 + 1] = dt;              // velocity from acceleration
    }
    
    static void getCostMatrices(real_t* Q, real_t* R) {
        // Track a straight path: minimize lateral error (y) and heading error
        for (int i = 0; i < 16; ++i) Q[i] = 0.0;
        
        Q[0*4 + 0] = 1.0;    // x position (progress)
        Q[1*4 + 1] = 100.0;  // y position (lateral error) - HIGH WEIGHT
        Q[2*4 + 2] = 10.0;   // heading error
        Q[3*4 + 3] = 1.0;    // velocity tracking
        
        // Input cost
        for (int i = 0; i < 4; ++i) R[i] = 0.0;
        R[0*2 + 0] = 1.0;    // steering effort
        R[1*2 + 1] = 0.1;    // acceleration effort
    }
};

// ============================================================================
// INVERTED PENDULUM (CART-POLE)
// ============================================================================

class InvertedPendulum {
public:
    static constexpr int nx = 4;   // [cart_pos, cart_vel, pole_angle, pole_angular_vel]
    static constexpr int nu = 1;   // [force on cart]
    
    static void getSystemMatrices(real_t* A, real_t* B, real_t dt = 0.02) {
        // Linearized cart-pole around upright position
        // From: Ogata, "Modern Control Engineering"
        
        real_t M = 1.0;      // cart mass (kg)
        real_t m = 0.1;      // pole mass (kg)
        real_t L = 0.5;      // pole length (m)
        real_t g = 9.81;     // gravity
        
        // Continuous-time A matrix
        real_t Ac[16] = {0};
        Ac[0*4 + 1] = 1.0;                                    // cart_pos_dot = cart_vel
        Ac[1*4 + 2] = -m*g / M;                               // cart_acc from pole angle
        Ac[2*4 + 3] = 1.0;                                    // pole_angle_dot = angular_vel
        Ac[3*4 + 2] = (M + m)*g / (M*L);                      // pole_angular_acc
        
        // Continuous-time B matrix
        real_t Bc[4] = {0};
        Bc[1] = 1.0 / M;                                      // cart acceleration
        Bc[3] = -1.0 / (M*L);                                 // pole angular acceleration
        
        // Discretize (Euler)
        for (int i = 0; i < 16; ++i) {
            A[i] = (i % 5 == 0) ? 1.0 : 0.0;
            A[i] += dt * Ac[i];
        }
        
        for (int i = 0; i < 4; ++i) {
            B[i] = dt * Bc[i];
        }
    }
    
    static void getCostMatrices(real_t* Q, real_t* R) {
        // Goal: keep pole upright (angle = 0) and cart near origin
        for (int i = 0; i < 16; ++i) Q[i] = 0.0;
        
        Q[0*4 + 0] = 1.0;     // cart position
        Q[1*4 + 1] = 1.0;     // cart velocity
        Q[2*4 + 2] = 100.0;   // pole angle - HIGH WEIGHT!
        Q[3*4 + 3] = 10.0;    // pole angular velocity
        
        R[0] = 0.1;           // control effort
    }
};

// ============================================================================
// MASS-SPRING-DAMPER CHAIN
// ============================================================================

class MassSpringDamper {
public:
    static constexpr int n_masses = 5;
    static constexpr int nx = 2 * n_masses;  // [positions, velocities]
    static constexpr int nu = 1;              // Force on first mass
    
    static void getSystemMatrices(real_t* A, real_t* B, real_t dt = 0.05) {
        real_t m = 1.0;    // mass
        real_t k = 10.0;   // spring constant
        real_t c = 1.0;    // damping
        
        // Continuous-time dynamics
        // M*q̈ + C*q̇ + K*q = F
        
        real_t Ac[100] = {0};
        
        // Position to velocity (upper-right block = I)
        for (int i = 0; i < n_masses; ++i) {
            Ac[i*10 + (n_masses + i)] = 1.0;
        }
        
        // Velocity equations (lower-left block = -K/m, lower-right = -C/m)
        for (int i = 0; i < n_masses; ++i) {
            int row = n_masses + i;
            
            // Spring forces
            if (i > 0) Ac[row*10 + (i-1)] = k/m;           // from left spring
            Ac[row*10 + i] = -2.0*k/m;                      // self
            if (i < n_masses-1) Ac[row*10 + (i+1)] = k/m;  // from right spring
            
            // Damping
            Ac[row*10 + row] = -c/m;
        }
        
        // Input: force on first mass
        real_t Bc[10] = {0};
        Bc[n_masses] = 1.0/m;
        
        // Discretize
        for (int i = 0; i < 100; ++i) {
            A[i] = (i % 11 == 0) ? 1.0 : 0.0;
            A[i] += dt * Ac[i];
        }
        
        for (int i = 0; i < 10; ++i) {
            B[i] = dt * Bc[i];
        }
    }
    
    static void getCostMatrices(real_t* Q, real_t* R) {
        // Regulate all masses to origin
        for (int i = 0; i < 100; ++i) Q[i] = 0.0;
        
        for (int i = 0; i < 10; ++i) {
            Q[i*10 + i] = (i < n_masses) ? 1.0 : 0.1;  // Position: 1.0, Velocity: 0.1
        }
        
        R[0] = 0.1;
    }
};

} // namespace RealWorldProblems

#endif // TEST_PROBLEMS_HPP
