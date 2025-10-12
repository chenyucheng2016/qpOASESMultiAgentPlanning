/*
 *  This file is part of qpOASES.
 *
 *  MultiAgentScenario: Setup and management for multi-agent planning scenarios
 *
 *  Author: Multi-Agent MPC Team
 *  Date: October 2025
 */

#ifndef QPOASES_MULTIAGENTSCENARIO_HPP
#define QPOASES_MULTIAGENTSCENARIO_HPP

#include <qpOASES/Environment.hpp>
#include <qpOASES/TurboADMM.hpp>
#include <vector>

BEGIN_NAMESPACE_QPOASES

/**
 * @brief Multi-agent scenario configuration
 * 
 * Manages:
 * - Multiple PointMass agents
 * - Start and goal positions
 * - Neighbor graph (collision checking)
 * - Environment integration
 * - ADMM setup and execution
 */
class MultiAgentScenario {
public:
    /**
     * @brief Constructor
     * @param env Environment reference
     * @param n_agents Number of agents
     * @param N Planning horizon
     * @param dt Time step
     */
    MultiAgentScenario(Environment& env, int n_agents, int N, real_t dt);
    
    /**
     * @brief Destructor
     */
    ~MultiAgentScenario();
    
    /**
     * @brief Set start position for an agent
     * @param agent_id Agent index
     * @param start Start position
     */
    void setStart(int agent_id, Point2D start);
    
    /**
     * @brief Set goal position for an agent
     * @param agent_id Agent index
     * @param goal Goal position
     */
    void setGoal(int agent_id, Point2D goal);
    
    /**
     * @brief Setup agents with default parameters
     * @param a_max Maximum acceleration (default: 10.0)
     * @param v_max Maximum velocity (default: 20.0)
     * @param Q_pos Position tracking weight (default: 10.0)
     * @param Q_vel Velocity tracking weight (default: 1.0)
     * @param R_ctrl Control effort weight (default: 0.01)
     */
    returnValue setupAgents(real_t a_max = 10.0, real_t v_max = 20.0,
                           real_t Q_pos = 10.0, real_t Q_vel = 1.0, real_t R_ctrl = 0.01);
    
    /**
     * @brief Setup neighbor graph (who can collide with whom)
     * @param neighbor_radius Maximum distance for collision checking (default: 0 = all agents)
     */
    returnValue setupNeighborGraph(real_t neighbor_radius = 0.0);
    
    /**
     * @brief Add environment obstacles to all agents
     * @param r_agent Agent safety radius (default: 0.5)
     */
    returnValue addEnvironmentObstacles(real_t r_agent = 0.5);
    
    /**
     * @brief Setup ADMM solver
     * @param d_safe Safety distance between agents (default: 1.0)
     * @param rho ADMM penalty parameter (default: 50.0)
     * @param max_admm_iter Maximum ADMM iterations (default: 10)
     */
    returnValue setupADMM(real_t d_safe = 1.0, real_t rho = 50.0, int max_admm_iter = 10);
    
    /**
     * @brief Solve multi-agent planning problem
     * @param converged Output: convergence flag
     * @return SUCCESSFUL_RETURN or error code
     */
    returnValue solve(BooleanType* converged = 0);
    
    /**
     * @brief Get solution trajectory for an agent
     * @param agent_id Agent index
     * @param trajectory Output: trajectory (length nV)
     * @return SUCCESSFUL_RETURN or error code
     */
    returnValue getSolution(int agent_id, real_t* trajectory);
    
    /**
     * @brief Get current position for an agent at stage k
     * @param agent_id Agent index
     * @param k Stage index (0 to N)
     * @return Position
     */
    Point2D getPosition(int agent_id, int k);
    
    /**
     * @brief Print scenario summary
     */
    void printSummary() const;
    
    /**
     * @brief Visualize scenario (environment + agents)
     */
    void visualize() const;
    
    /**
     * @brief Save trajectories to file
     * @param filename Output filename (CSV format)
     */
    returnValue saveTrajectories(const char* filename);
    
    /**
     * @brief Get number of agents
     */
    int getNumAgents() const { return n_agents_; }
    
    /**
     * @brief Get agent reference
     */
    PointMass& getAgent(int agent_id) { return *agents_[agent_id]; }
    
    /**
     * @brief Get environment reference
     */
    Environment& getEnvironment() { return env_; }
    
private:
    // Environment
    Environment& env_;
    
    // Scenario parameters
    int n_agents_;
    int N_;
    real_t dt_;
    
    // Agents
    std::vector<PointMass*> agents_;
    std::vector<Point2D> starts_;
    std::vector<Point2D> goals_;
    
    // Neighbor graph
    std::vector<std::vector<int>> neighbors_;
    std::vector<int> num_neighbors_;
    
    // ADMM solver
    TurboADMM* admm_;
    bool admm_setup_;
    
    // Solution storage
    std::vector<real_t*> solutions_;
    
    /**
     * @brief Helper: Generate dynamically feasible reference trajectory
     * @param agent_id Agent index
     */
    void generateReference(int agent_id);
    
    /**
     * @brief Helper: Check if two agents are neighbors
     * @param i Agent i
     * @param j Agent j
     * @param radius Neighbor radius (0 = always neighbors)
     */
    bool areNeighbors(int i, int j, real_t radius) const;
};

END_NAMESPACE_QPOASES

#endif  // QPOASES_MULTIAGENTSCENARIO_HPP
