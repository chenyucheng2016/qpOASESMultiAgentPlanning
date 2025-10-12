/*
 *  This file is part of qpOASES.
 *
 *  MultiAgentScenario: Setup and management for multi-agent planning scenarios
 *
 *  Implementation
 */

#include <qpOASES/MultiAgentScenario.hpp>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>

BEGIN_NAMESPACE_QPOASES

MultiAgentScenario::MultiAgentScenario(Environment& env, int n_agents, int N, real_t dt)
    : env_(env), n_agents_(n_agents), N_(N), dt_(dt), admm_(0), admm_setup_(false)
{
    // Allocate agents
    agents_.resize(n_agents_);
    for (int i = 0; i < n_agents_; ++i) {
        agents_[i] = new PointMass(N, dt, 10.0, 20.0);  // Default params
    }
    
    // Allocate start/goal positions
    starts_.resize(n_agents_);
    goals_.resize(n_agents_);
    
    // Allocate neighbor graph
    neighbors_.resize(n_agents_);
    num_neighbors_.resize(n_agents_);
    
    // Allocate solution storage
    solutions_.resize(n_agents_);
    for (int i = 0; i < n_agents_; ++i) {
        solutions_[i] = 0;  // Will be allocated after agent setup
    }
}

MultiAgentScenario::~MultiAgentScenario()
{
    // Delete agents
    for (int i = 0; i < n_agents_; ++i) {
        if (agents_[i]) {
            delete agents_[i];
        }
    }
    
    // Delete ADMM solver
    if (admm_) {
        delete admm_;
    }
    
    // Delete solution storage
    for (int i = 0; i < n_agents_; ++i) {
        if (solutions_[i]) {
            delete[] solutions_[i];
        }
    }
}

void MultiAgentScenario::setStart(int agent_id, Point2D start)
{
    if (agent_id >= 0 && agent_id < n_agents_) {
        starts_[agent_id] = start;
    }
}

void MultiAgentScenario::setGoal(int agent_id, Point2D goal)
{
    if (agent_id >= 0 && agent_id < n_agents_) {
        goals_[agent_id] = goal;
    }
}

returnValue MultiAgentScenario::setupAgents(real_t a_max, real_t v_max,
                                            real_t Q_pos, real_t Q_vel, real_t R_ctrl)
{
    for (int i = 0; i < n_agents_; ++i) {
        PointMass* agent = agents_[i];
        
        // Setup dynamics, bounds, and cost
        agent->setupDynamics();
        agent->setupBounds();
        agent->setupCostMatrices(Q_pos, Q_vel, R_ctrl);
        
        // Generate reference trajectory from start to goal
        generateReference(i);
        
        // Allocate solution storage
        if (solutions_[i]) {
            delete[] solutions_[i];
        }
        solutions_[i] = new real_t[agent->nV];
    }
    
    return SUCCESSFUL_RETURN;
}

returnValue MultiAgentScenario::setupNeighborGraph(real_t neighbor_radius)
{
    // Clear existing neighbor graph
    for (int i = 0; i < n_agents_; ++i) {
        neighbors_[i].clear();
    }
    
    // Build neighbor graph
    for (int i = 0; i < n_agents_; ++i) {
        for (int j = i + 1; j < n_agents_; ++j) {
            if (areNeighbors(i, j, neighbor_radius)) {
                neighbors_[i].push_back(j);
                neighbors_[j].push_back(i);
            }
        }
        num_neighbors_[i] = (int)neighbors_[i].size();
    }
    
    return SUCCESSFUL_RETURN;
}

returnValue MultiAgentScenario::addEnvironmentObstacles(real_t r_agent)
{
    // Get obstacles from environment
    const std::vector<RectangularObstacle>& obstacles = env_.getObstacles();
    
    if (obstacles.empty()) {
        return SUCCESSFUL_RETURN;  // No obstacles to add
    }
    
    // For each agent, add all obstacles
    for (int i = 0; i < n_agents_; ++i) {
        PointMass* agent = agents_[i];
        
        for (size_t obs_idx = 0; obs_idx < obstacles.size(); ++obs_idx) {
            // Define obstacle in agent
            int obs_id = agent->defineObstacle(obstacles[obs_idx]);
            
            // For now, add simple bypass constraint (can be improved with RRT*)
            // Use BYPASS_LEFT as default (will be updated by RRT* later)
            BypassSide schedule[N_ + 1];
            for (int k = 0; k <= N_; ++k) {
                schedule[k] = BYPASS_LEFT;  // Default bypass side
            }
            
            // Add time-varying constraint
            agent->addTimeVaryingObstacleConstraint(obs_id, schedule, r_agent);
        }
    }
    
    return SUCCESSFUL_RETURN;
}

returnValue MultiAgentScenario::setupADMM(real_t d_safe, real_t rho, int max_admm_iter)
{
    // Create ADMM solver
    if (admm_) {
        delete admm_;
    }
    admm_ = new TurboADMM(n_agents_);
    
    // Setup coupling data
    CouplingData coupling;
    coupling.d_safe = d_safe;
    
    // Setup ADMM parameters
    ADMMParameters params;
    params.max_admm_iter = max_admm_iter;
    params.max_qp_iter = 100;
    params.rho = rho;
    params.eps_primal = 1e-3;
    params.eps_dual = 1e-3;
    params.enable_collision_avoidance = BT_TRUE;
    
    // Convert agents to AgentData array
    AgentData* agent_data = new AgentData[n_agents_];
    for (int i = 0; i < n_agents_; ++i) {
        agent_data[i] = *agents_[i];  // Copy agent data
    }
    
    // Convert neighbor graph to array format
    int** neighbors_array = new int*[n_agents_];
    for (int i = 0; i < n_agents_; ++i) {
        if (num_neighbors_[i] > 0) {
            neighbors_array[i] = new int[num_neighbors_[i]];
            for (int j = 0; j < num_neighbors_[i]; ++j) {
                neighbors_array[i][j] = neighbors_[i][j];
            }
        } else {
            neighbors_array[i] = 0;
        }
    }
    
    // Setup ADMM
    returnValue ret = admm_->setup(agent_data, n_agents_, &coupling, &params,
                                   neighbors_array, num_neighbors_.data());
    
    // Cleanup temporary arrays
    delete[] agent_data;
    for (int i = 0; i < n_agents_; ++i) {
        if (neighbors_array[i]) {
            delete[] neighbors_array[i];
        }
    }
    delete[] neighbors_array;
    
    if (ret == SUCCESSFUL_RETURN) {
        admm_setup_ = true;
    }
    
    return ret;
}

returnValue MultiAgentScenario::solve(BooleanType* converged)
{
    if (!admm_setup_) {
        printf("ERROR: ADMM not setup. Call setupADMM() first.\n");
        return RET_INIT_FAILED;
    }
    
    // Setup initial conditions
    real_t** x_init = new real_t*[n_agents_];
    for (int i = 0; i < n_agents_; ++i) {
        x_init[i] = new real_t[agents_[i]->nx];
        x_init[i][0] = starts_[i].x;   // x
        x_init[i][1] = starts_[i].y;   // y
        x_init[i][2] = 0.0;            // vx
        x_init[i][3] = 0.0;            // vy
    }
    
    // Solve
    BooleanType conv = BT_FALSE;
    returnValue ret = admm_->solveColdStart(x_init, &conv);
    
    // Get solutions
    if (ret == SUCCESSFUL_RETURN) {
        for (int i = 0; i < n_agents_; ++i) {
            admm_->getSolution(i, solutions_[i]);
        }
    }
    
    // Cleanup
    for (int i = 0; i < n_agents_; ++i) {
        delete[] x_init[i];
    }
    delete[] x_init;
    
    if (converged) {
        *converged = conv;
    }
    
    return ret;
}

returnValue MultiAgentScenario::getSolution(int agent_id, real_t* trajectory)
{
    if (agent_id < 0 || agent_id >= n_agents_) {
        return RET_INDEX_OUT_OF_BOUNDS;
    }
    
    if (!solutions_[agent_id]) {
        return RET_INIT_FAILED;
    }
    
    memcpy(trajectory, solutions_[agent_id], agents_[agent_id]->nV * sizeof(real_t));
    return SUCCESSFUL_RETURN;
}

Point2D MultiAgentScenario::getPosition(int agent_id, int k)
{
    if (agent_id < 0 || agent_id >= n_agents_ || !solutions_[agent_id]) {
        return Point2D(0.0, 0.0);
    }
    
    PointMass* agent = agents_[agent_id];
    int idx = (k == N_) ? (N_ * (agent->nx + agent->nu)) : (k * (agent->nx + agent->nu));
    
    return Point2D(solutions_[agent_id][idx + 0], solutions_[agent_id][idx + 1]);
}

void MultiAgentScenario::printSummary() const
{
    printf("\n=== Multi-Agent Scenario Summary ===\n");
    printf("Number of agents: %d\n", n_agents_);
    printf("Planning horizon: N=%d, dt=%.2f s (total: %.2f s)\n", N_, dt_, N_ * dt_);
    printf("Environment obstacles: %d\n", (int)env_.getObstacles().size());
    
    printf("\nAgent Start/Goal Positions:\n");
    for (int i = 0; i < n_agents_; ++i) {
        printf("  Agent %d: (%.1f, %.1f) → (%.1f, %.1f)\n",
               i, starts_[i].x, starts_[i].y, goals_[i].x, goals_[i].y);
    }
    
    printf("\nNeighbor Graph:\n");
    for (int i = 0; i < n_agents_; ++i) {
        printf("  Agent %d: %d neighbors [", i, num_neighbors_[i]);
        for (int j = 0; j < num_neighbors_[i]; ++j) {
            printf("%d", neighbors_[i][j]);
            if (j < num_neighbors_[i] - 1) printf(", ");
        }
        printf("]\n");
    }
    
    printf("\nADMM Setup: %s\n", admm_setup_ ? "YES" : "NO");
}

void MultiAgentScenario::visualize() const
{
    // Get agent positions at final stage
    Point2D* positions = new Point2D[n_agents_];
    for (int i = 0; i < n_agents_; ++i) {
        if (solutions_[i]) {
            PointMass* agent = agents_[i];
            int idx = N_ * (agent->nx + agent->nu);
            positions[i].x = solutions_[i][idx + 0];
            positions[i].y = solutions_[i][idx + 1];
        } else {
            positions[i] = starts_[i];  // Use start if no solution
        }
    }
    
    env_.printASCII(positions, n_agents_);
    
    delete[] positions;
}

returnValue MultiAgentScenario::saveTrajectories(const char* filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        printf("ERROR: Failed to open file %s for writing\n", filename);
        return RET_UNKNOWN_BUG;
    }
    
    // Write header
    file << "# Multi-Agent Trajectories\n";
    file << "# N=" << N_ << ", dt=" << dt_ << ", n_agents=" << n_agents_ << "\n";
    file << "# Format: agent_id, k, t, x, y, vx, vy, ax, ay\n\n";
    
    // Write trajectories
    for (int i = 0; i < n_agents_; ++i) {
        if (!solutions_[i]) continue;
        
        PointMass* agent = agents_[i];
        
        for (int k = 0; k <= N_; ++k) {
            int idx = (k == N_) ? (N_ * (agent->nx + agent->nu)) : (k * (agent->nx + agent->nu));
            
            real_t t = k * dt_;
            real_t x = solutions_[i][idx + 0];
            real_t y = solutions_[i][idx + 1];
            real_t vx = solutions_[i][idx + 2];
            real_t vy = solutions_[i][idx + 3];
            
            real_t ax = 0.0, ay = 0.0;
            if (k < N_) {
                ax = solutions_[i][idx + 4];
                ay = solutions_[i][idx + 5];
            }
            
            file << i << ", " << k << ", " << t << ", "
                 << x << ", " << y << ", " << vx << ", " << vy << ", "
                 << ax << ", " << ay << "\n";
        }
        
        file << "\n";  // Blank line between agents
    }
    
    file.close();
    return SUCCESSFUL_RETURN;
}

void MultiAgentScenario::generateReference(int agent_id)
{
    PointMass* agent = agents_[agent_id];
    Point2D start = starts_[agent_id];
    Point2D goal = goals_[agent_id];
    
    // Generate dynamically feasible reference using forward simulation
    // Constant acceleration to reach goal
    real_t dx = goal.x - start.x;
    real_t dy = goal.y - start.y;
    real_t T = N_ * dt_;
    
    // Constant acceleration: a = 2*(x_f - x_0) / T^2
    real_t ax_const = 2.0 * dx / (T * T);
    real_t ay_const = 2.0 * dy / (T * T);
    
    // Forward simulate
    real_t x = start.x, y = start.y;
    real_t vx = 0.0, vy = 0.0;
    
    for (int k = 0; k <= N_; ++k) {
        // Store state
        agent->x_ref[k * agent->nx + 0] = x;
        agent->x_ref[k * agent->nx + 1] = y;
        agent->x_ref[k * agent->nx + 2] = vx;
        agent->x_ref[k * agent->nx + 3] = vy;
        
        // Store control (zero for last stage)
        if (k < N_) {
            agent->u_ref[k * agent->nu + 0] = ax_const;
            agent->u_ref[k * agent->nu + 1] = ay_const;
            
            // Forward step
            vx += ax_const * dt_;
            vy += ay_const * dt_;
            x += vx * dt_;
            y += vy * dt_;
        }
    }
}

bool MultiAgentScenario::areNeighbors(int i, int j, real_t radius) const
{
    if (radius <= 0.0) {
        return true;  // All agents are neighbors
    }
    
    // Check distance between start positions
    real_t dx = starts_[i].x - starts_[j].x;
    real_t dy = starts_[i].y - starts_[j].y;
    real_t dist = sqrt(dx * dx + dy * dy);
    
    return dist <= radius;
}

END_NAMESPACE_QPOASES
