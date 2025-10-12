/*
 *  This file is part of qpOASES.
 *
 *  qpOASES -- An Implementation of the Online Active Set Strategy.
 *  Copyright (C) 2007-2024 by Hans Joachim Ferreau, Andreas Potschka,
 *  Christian Kirches et al. All rights reserved.
 *
 *  TurboADMM: Multi-Agent MPC using ADMM with MPC-aware qpOASES
 *
 *  Author: Multi-Agent MPC Team
 *  Version: 1.0
 *  Date: 2025
 */

#ifndef QPOASES_TURBO_ADMM_HPP
#define QPOASES_TURBO_ADMM_HPP

#include <qpOASES/QProblem.hpp>
#include <qpOASES/Options.hpp>
#include <vector>
#include <cmath>

BEGIN_NAMESPACE_QPOASES


/**
 * Agent data structure for multi-agent MPC
 * Contains all MPC parameters for a single agent
 */
struct AgentData
{
    int N;              /**< Prediction horizon */
    int nx;             /**< State dimension */
    int nu;             /**< Control dimension */
    int nV;             /**< Decision variable dimension: N*nx + (N-1)*nu */
    int nC;             /**< Constraint dimension: (N-1)*nx (dynamics) */
    
    real_t* A;          /**< State transition matrix (nx × nx) */
    real_t* B;          /**< Control input matrix (nx × nu) */
    real_t* Q;          /**< State cost matrix (nx × nx), assumed diagonal */
    real_t* R;          /**< Control cost matrix (nu × nu), assumed diagonal */
    
    real_t* Q_diag;     /**< Diagonal of Q (nx × 1), for efficient operations */
    real_t* R_diag;     /**< Diagonal of R (nu × 1), for efficient operations */
    
    real_t* x_ref;      /**< Reference state trajectory (N+1 × nx) */
    real_t* u_ref;      /**< Reference control trajectory (N × nu) */
    
    real_t* lb;         /**< Lower bounds on decision variables (nV × 1) */
    real_t* ub;         /**< Upper bounds on decision variables (nV × 1) */
    real_t* lbA;        /**< Lower bounds on constraints (nC × 1) */
    real_t* ubA;        /**< Upper bounds on constraints (nC × 1) */
    
    // Static inequality constraints: c <= G*x <= d
    int nG;             /**< Number of static inequality constraints */
    real_t* G;          /**< Static constraint matrix (nG × nV) */
    real_t* c;          /**< Lower bounds on static constraints (nG × 1) */
    real_t* d;          /**< Upper bounds on static constraints (nG × 1) */
    
    /** Constructor */
    AgentData();
    
    /** Copy constructor (deep copy) */
    AgentData(const AgentData& other);
    
    /** Assignment operator (deep copy) */
    AgentData& operator=(const AgentData& other);
    
    /** Destructor */
    ~AgentData();
    
    /** Allocate memory for agent data */
    returnValue allocate(int N_, int nx_, int nu_);
    
    /** Set diagonal elements of Q and R */
    returnValue extractDiagonals();
    
    /** Free all allocated memory */
    returnValue clear();
};


/**
 * Rectangular obstacle definition
 * Supports arbitrary orientation via rotation angle theta
 */
struct RectangularObstacle
{
    real_t x_center;    /**< Center x position (m) */
    real_t y_center;    /**< Center y position (m) */
    real_t width;       /**< Width along local x-axis (m) */
    real_t height;      /**< Height along local y-axis (m) */
    real_t theta;       /**< Orientation angle (radians, counter-clockwise from x-axis) */
    int id;             /**< Unique obstacle identifier */
    
    /** Constructor */
    RectangularObstacle() : x_center(0), y_center(0), width(0), height(0), theta(0), id(-1) {}
    
    RectangularObstacle(real_t x, real_t y, real_t w, real_t h, real_t angle = 0.0, int obstacle_id = -1)
        : x_center(x), y_center(y), width(w), height(h), theta(angle), id(obstacle_id) {}
};


/**
 * Bypass side selection for obstacle avoidance
 * Specifies which side of the obstacle the agent should pass
 */
enum BypassSide
{
    BYPASS_LEFT = 0,    /**< Stay on left side of obstacle (in obstacle's local frame) */
    BYPASS_RIGHT = 1,   /**< Stay on right side of obstacle (in obstacle's local frame) */
    BYPASS_ABOVE = 2,   /**< Stay above obstacle (in obstacle's local frame) */
    BYPASS_BELOW = 3    /**< Stay below obstacle (in obstacle's local frame) */
};


/**
 * PointMass agent with 2D point mass dynamics
 * 
 * State: [x, y, vx, vy] (position and velocity in 2D)
 * Control: [ax, ay] (acceleration in 2D)
 * Dynamics: x_dot = vx, y_dot = vy, vx_dot = ax, vy_dot = ay
 * 
 * Discretization (Euler): x[k+1] = x[k] + dt*vx[k]
 *                         y[k+1] = y[k] + dt*vy[k]
 *                         vx[k+1] = vx[k] + dt*ax[k]
 *                         vy[k+1] = vy[k] + dt*ay[k]
 */
class PointMass : public AgentData
{
public:
    real_t dt;          /**< Time step for discretization */
    real_t a_max;       /**< Maximum acceleration magnitude (box constraint) */
    real_t v_max;       /**< Maximum velocity magnitude (box constraint, optional) */
    
    /**
     * Constructor
     * @param N_ Prediction horizon
     * @param dt_ Time step
     * @param a_max_ Maximum acceleration (default: 10.0 m/s^2)
     * @param v_max_ Maximum velocity (default: 20.0 m/s, negative means no limit)
     */
    PointMass(int N_ = 10, real_t dt_ = 0.2, real_t a_max_ = 10.0, real_t v_max_ = 20.0);
    
    /**
     * Setup dynamics matrices A and B for 2D point mass
     * A = [1  0  dt  0]    B = [0   0 ]
     *     [0  1  0  dt]        [0   0 ]
     *     [0  0  1   0]        [dt  0 ]
     *     [0  0  0   1]        [0  dt ]
     */
    returnValue setupDynamics();
    
    /**
     * Setup bounds on states and controls
     * Control bounds: -a_max <= ax, ay <= a_max
     * Velocity bounds: -v_max <= vx, vy <= v_max (if v_max > 0)
     * Position bounds: unbounded (can be set externally via lb, ub)
     */
    returnValue setupBounds();
    
    /**
     * Setup cost matrices Q and R (diagonal)
     * @param Q_pos Position cost weight (for x, y)
     * @param Q_vel Velocity cost weight (for vx, vy)
     * @param R_ctrl Control cost weight (for ax, ay)
     */
    returnValue setupCostMatrices(real_t Q_pos = 10.0, real_t Q_vel = 1.0, real_t R_ctrl = 0.01);
    
    /**
     * Define a rectangular obstacle (stores geometry, does not add constraints)
     * 
     * Step 1 of 2-step process:
     * 1. defineObstacle() - stores obstacle geometry
     * 2. addObstacleConstraint() - adds constraints for specific bypass side
     * 
     * @param obstacle Rectangular obstacle definition (center, size, orientation)
     * @return Obstacle ID for later reference
     */
    int defineObstacle(const RectangularObstacle& obstacle);
    
    /**
     * Add constraint for bypassing obstacle on specified side
     * 
     * Step 2 of 2-step process. This adds ONE linear inequality constraint
     * for the specified bypass side. Call this after defineObstacle().
     * 
     * The constraint is in the obstacle's local frame:
     * - BYPASS_LEFT:  agent stays on left side (local y > height/2 + r_agent)
     * - BYPASS_RIGHT: agent stays on right side (local y < -height/2 - r_agent)
     * - BYPASS_ABOVE: agent stays above (local x > width/2 + r_agent)
     * - BYPASS_BELOW: agent stays below (local x < -width/2 - r_agent)
     * 
     * Constraint is applied at all stages k=0..N.
     * 
     * @param obstacle_id ID returned by defineObstacle()
     * @param side Which side to bypass (LEFT, RIGHT, ABOVE, BELOW)
     * @param r_agent Agent safety radius (default: 0.5)
     * @return SUCCESSFUL_RETURN or error code
     */
    returnValue addObstacleConstraint(int obstacle_id, BypassSide side, real_t r_agent = 0.5);
    
    /**
     * Add time-varying obstacle constraint based on reference trajectory
     * 
     * This method adds stage-dependent constraints that vary over the prediction horizon.
     * It is designed to work with global path planners (A*, RRT*, etc.) that provide
     * a reference trajectory around obstacles.
     * 
     * At each stage k, a different bypass side can be specified, allowing the agent
     * to transition from one side of an obstacle to another (e.g., from LEFT to ABOVE
     * when going around a corner).
     * 
     * This maintains convexity at each stage (only one constraint per stage) while
     * allowing the agent to follow complex reference trajectories around obstacles.
     * 
     * Usage:
     * 1. Run global planner (A*, RRT*) to get reference trajectory
     * 2. Analyze reference to determine which side of obstacle at each stage
     * 3. Create bypass schedule array
     * 4. Call this method to add time-varying constraints
     * 
     * Example:
     *   BypassSide schedule[N+1] = {
     *       BYPASS_LEFT, BYPASS_LEFT, BYPASS_LEFT,    // k=0,1,2: left of obstacle
     *       BYPASS_ABOVE, BYPASS_ABOVE, BYPASS_ABOVE  // k=3,4,5: above obstacle
     *   };
     *   agent.addTimeVaryingObstacleConstraint(obs_id, schedule, 0.5);
     * 
     * @param obstacle_id ID returned by defineObstacle()
     * @param bypass_schedule Array of bypass sides, one per stage k=0..N (length N+1)
     * @param r_agent Agent safety radius (default: 0.5)
     * @return SUCCESSFUL_RETURN or error code
     */
    returnValue addTimeVaryingObstacleConstraint(int obstacle_id, const BypassSide* bypass_schedule, real_t r_agent = 0.5);
    
    /**
     * Update obstacle constraints based on current trajectory
     * 
     * For Sequential Convex Programming (SCP), obstacle constraints need to be
     * updated at each iteration based on the newly computed trajectory. This is
     * critical for:
     * - Circular obstacles (re-linearize around new trajectory)
     * - Rotated rectangular obstacles (improve constraint accuracy)
     * - Non-convex constraints (iterative convexification)
     * 
     * This method should be called at each ADMM iteration BEFORE solving the QP.
     * 
     * Usage in ADMM loop:
     *   for (int iter = 0; iter < max_admm_iter; ++iter) {
     *       // Update constraints based on current trajectory
     *       agent.updateObstacleConstraints(z_current, bypass_schedule);
     *       
     *       // Solve QP with updated constraints
     *       solveAgentQP(i);
     *   }
     * 
     * @param z_current Current trajectory (length nV = N*(nx+nu) + nx)
     * @param bypass_schedule Bypass schedule for time-varying constraints (length N+1)
     * @return SUCCESSFUL_RETURN or error code
     */
    returnValue updateObstacleConstraints(const real_t* z_current, const BypassSide* bypass_schedule);
    
    /**
     * Add rectangular workspace bounds
     * @param x_min Minimum x position
     * @param x_max Maximum x position
     * @param y_min Minimum y position
     * @param y_max Maximum y position
     */
    returnValue addWorkspaceBounds(real_t x_min, real_t x_max, real_t y_min, real_t y_max);

private:
    /**
     * Internal storage for defined obstacles
     */
    std::vector<RectangularObstacle> obstacles_;
    
    /**
     * Bypass schedule storage for SCP constraint updates
     * Stores bypass side for each stage k=0..N
     */
    std::vector<BypassSide> bypass_schedule_;
    
    /**
     * Helper: Transform point from global frame to obstacle's local frame
     * @param x_global Global x coordinate
     * @param y_global Global y coordinate
     * @param obstacle Obstacle definition
     * @param x_local Output: local x coordinate
     * @param y_local Output: local y coordinate
     */
    void transformToLocal(real_t x_global, real_t y_global, 
                          const RectangularObstacle& obstacle,
                          real_t& x_local, real_t& y_local) const;
    
    /**
     * Helper: Get constraint coefficients for bypass side in global frame
     * @param obstacle Obstacle definition
     * @param side Bypass side
     * @param r_agent Safety margin
     * @param a Output: constraint coefficient for x (a*x + b*y <= c or >= d)
     * @param b Output: constraint coefficient for y
     * @param bound Output: constraint bound value
     * @param is_upper Output: true if upper bound (<=), false if lower bound (>=)
     */
    void getConstraintCoefficients(const RectangularObstacle& obstacle, BypassSide side, real_t r_agent,
                                   real_t& a, real_t& b, real_t& bound, bool& is_upper) const;
};


/**
 * Coupling constraint data structure
 * Represents constraints between agents (e.g., collision avoidance)
 */
struct CouplingData
{
    real_t d_safe;      /**< Safety distance for collision avoidance */
    real_t gamma;       /**< Coupling penalty weight for preconditioner */
    
    /** Constructor */
    CouplingData();
};


/**
 * ADMM algorithm parameters
 */
struct ADMMParameters
{
    real_t rho;             /**< Penalty parameter (adaptive) */
    real_t alpha_relax;     /**< Over-relaxation parameter (1.0-2.0, typically 1.7) */
    
    real_t eps_primal;      /**< Primal residual tolerance */
    real_t eps_dual;        /**< Dual residual tolerance */
    
    int max_admm_iter;      /**< Maximum ADMM iterations per MPC step */
    int max_qp_iter;        /**< Maximum iterations per QP solve */
    
    BooleanType enable_adaptive_rho;   /**< Enable adaptive penalty parameter */
    BooleanType enable_over_relax;     /**< Enable over-relaxation */
    BooleanType enable_collision_avoidance;  /**< Enable collision avoidance (soft penalty) */
    
    /** Constructor with default values */
    ADMMParameters();
};


/**
 * ADMM statistics for performance monitoring
 */
struct ADMMStatistics
{
    int admm_iterations;        /**< Number of ADMM iterations */
    int total_qp_iterations;    /**< Total QP iterations across all agents */
    
    real_t solve_time_ms;       /**< Total solve time in milliseconds */
    real_t primal_residual;     /**< Final primal residual */
    real_t dual_residual;       /**< Final dual residual */
    
    BooleanType converged;      /**< Did ADMM converge? */
    
    /** Constructor */
    ADMMStatistics();
    
    /** Reset statistics */
    void reset();
};


/**
 * Main TurboADMM class for multi-agent MPC
 * 
 * Implements ADMM-based distributed optimization leveraging:
 * - MPC-aware qpOASES (Riccati warm start, O(N) TQ factorization)
 * - Sherman-Morrison preconditioner (rank-1 updates)
 * - Adaptive penalty parameter
 * - Over-relaxation
 * 
 * Design: Sequential implementation for testing, parallelization later
 */
class TurboADMM
{
public:
    /** Default constructor */
    TurboADMM();
    
    /** Constructor with agent data */
    TurboADMM(int n_agents);
    
    /** Destructor */
    ~TurboADMM();
    
    
    /** 
     * Setup: Initialize all agents and ADMM variables
     * 
     * @param agents        Array of agent data (size: n_agents)
     * @param n_agents      Number of agents
     * @param coupling      Coupling constraint data
     * @param params        ADMM parameters
     * @param neighbors     Neighbor graph: neighbors[i] = array of neighbor indices
     * @param num_neighbors Number of neighbors per agent
     * @return              SUCCESSFUL_RETURN or error code
     */
    returnValue setup(
        const AgentData* agents,
        int n_agents,
        const CouplingData* coupling,
        const ADMMParameters* params,
        const int* const* neighbors = 0,
        const int* num_neighbors = 0
    );
    
    
    /**
     * Solve: Cold start for first MPC step
     * 
     * @param x_init        Initial states for all agents (array of nx vectors)
     * @param converged_out Output: did ADMM converge?
     * @return              SUCCESSFUL_RETURN or error code
     */
    returnValue solveColdStart(
        const real_t* const* x_init,
        BooleanType* converged_out = 0
    );
    
    
    /**
     * Solve: Hotstart for subsequent MPC steps
     * 
     * @param x_init        Updated initial states
     * @param converged_out Output: did ADMM converge?
     * @return              SUCCESSFUL_RETURN or error code
     */
    returnValue solveHotstart(
        const real_t* const* x_init,
        BooleanType* converged_out = 0
    );
    
    
    /**
     * Get solution for a specific agent
     * 
     * @param agent_id      Agent index (0 to n_agents-1)
     * @param z_out         Output: primal solution (nV × 1)
     * @return              SUCCESSFUL_RETURN or error code
     */
    returnValue getSolution(int agent_id, real_t* z_out) const;
    
    
    /**
     * Get ADMM statistics
     * 
     * @param stats_out     Output: statistics structure
     * @return              SUCCESSFUL_RETURN or error code
     */
    returnValue getStatistics(ADMMStatistics* stats_out) const;
    
    
protected:
    // Core setup functions
    returnValue allocateADMMVariables();
    returnValue setupAgentSolvers();
    returnValue computePreconditionerData();
    
    // ADMM iteration components
    returnValue solveAgentSubproblems();
    returnValue updateConsensus();
    returnValue updateDualVariables();
    returnValue adaptPenaltyParameter();
    returnValue applyOverRelaxation();
    
    // Convergence check
    BooleanType checkConvergence(real_t* r_primal_out, real_t* r_dual_out);
    
    // Adaptive penalty parameter
    returnValue adaptPenaltyParameter(real_t r_primal_max, real_t r_dual_max);
    
    // Collision coupling (coupling-constraint ADMM)
    returnValue projectCollisionConstraints();
    returnValue updateCollisionDuals();
    returnValue addCollisionCouplingGradient(int agent_i, real_t* g);
    
    // Warm start utilities
    returnValue initializeWithRiccati(const real_t* const* x_init);
    returnValue shiftTrajectories();
    
    // Helper functions
    returnValue computeTrackingGradient(int agent_id, real_t* g_out);
    returnValue computeAugmentedGradient(int agent_id, real_t* g_aug_out);
    returnValue applyPreconditioner(int agent_id, int stage, 
                                     const real_t* v, real_t* result);
    returnValue detectCoupling(int agent_id, int stage, real_t** c_out);
    
    // Sherman-Morrison utilities
    returnValue applyRank1PreconditionerInverse(
        const real_t* Q_diag,
        const real_t* c,
        real_t gamma,
        const real_t* v,
        int nx,
        real_t* result
    );
    
    
private:
    // Configuration
    int n_agents_;              /**< Number of agents */
    AgentData* agents_;         /**< Array of agent data */
    CouplingData coupling_;     /**< Coupling constraints */
    ADMMParameters params_;     /**< ADMM parameters */
    
    // Agent solvers (layer 1: MPC-aware qpOASES)
    QProblem** agent_solvers_;  /**< Array of QProblem instances */
    
    // ADMM variables
    real_t** z_local_;          /**< Local primal variables (per agent) */
    real_t** z_prev_;           /**< Previous solutions (for warm start) */
    
    // Collision coupling variables (coupling-constraint ADMM)
    int** neighbors_;           /**< Neighbor graph: neighbors_[i] = list of neighbors for agent i */
    int* num_neighbors_;        /**< Number of neighbors per agent */
    real_t**** v_collision_;    /**< Auxiliary vars: v_collision_[i][j][k][state_idx] */
    real_t**** lambda_collision_; /**< Dual vars: lambda_collision_[i][j][k][state_idx] */
    
    int nV_total_;              /**< Total decision variables: Σ nV_i */
    
    // Preconditioner data (coupling vectors per agent per stage)
    real_t*** coupling_vectors_;  /**< coupling_vectors[agent][stage] = c vector */
    
    // Statistics
    ADMMStatistics stats_;
    
    // Internal state
    BooleanType is_setup_;
    BooleanType is_first_solve_;
    
    // No copy constructor or assignment operator
    TurboADMM(const TurboADMM&);
    TurboADMM& operator=(const TurboADMM&);
};


END_NAMESPACE_QPOASES

#endif  // QPOASES_TURBO_ADMM_HPP
