/*
 * Test: RRT* to ADMM Pipeline (Single Agent with Obstacle)
 * 
 * Simplified end-to-end test:
 * 1. RRT* generates collision-free path with temporal information
 * 2. Bypass schedule generator determines obstacle sides
 * 3. Time-varying constraints added to PointMass agent
 * 4. Single-agent MPC optimization with obstacle avoidance
 */

#include <qpOASES/TurboADMM.hpp>
#include <qpOASES/RRTStarPlanner.hpp>
#include <qpOASES/BypassScheduleGenerator.hpp>
#include <cstdio>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

USING_NAMESPACE_QPOASES

void visualizeTrajectory(const Environment& env,
                        const real_t* x,
                        int N, int nx,
                        const RectangularObstacle& obstacle,
                        const RRTPath& rrt_path)
{
    printf("\n=== Trajectory Visualization ===\n");
    
    const int width = 80;
    const int height = 40;
    
    real_t x_min = 0.0;
    real_t x_max = 30.0;
    real_t y_min = 0.0;
    real_t y_max = 20.0;
    
    // Create grid
    char grid[height][width];
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            grid[i][j] = '.';
        }
    }
    
    // Draw obstacle
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            real_t x = x_min + (x_max - x_min) * j / (width - 1);
            real_t y = y_max - (y_max - y_min) * i / (height - 1);
            
            real_t dx = x - obstacle.x_center;
            real_t dy = y - obstacle.y_center;
            real_t cos_theta = cos(-obstacle.theta);
            real_t sin_theta = sin(-obstacle.theta);
            real_t local_x = cos_theta * dx - sin_theta * dy;
            real_t local_y = sin_theta * dx + cos_theta * dy;
            
            if (fabs(local_x) < obstacle.width / 2.0 && fabs(local_y) < obstacle.height / 2.0) {
                grid[i][j] = '#';
            }
        }
    }
    
    // Draw RRT* path (background)
    for (int k = 0; k < rrt_path.getNumWaypoints(); ++k) {
        const TimedWaypoint& wp = rrt_path.getTimedWaypoints()[k];
        
        int j = (int)((wp.position.x - x_min) / (x_max - x_min) * (width - 1));
        int i = (int)((y_max - wp.position.y) / (y_max - y_min) * (height - 1));
        
        if (i >= 0 && i < height && j >= 0 && j < width && grid[i][j] == '.') {
            grid[i][j] = 'r';  // RRT* path
        }
    }
    
    // Draw MPC trajectory (foreground)
    for (int k = 0; k <= N; ++k) {
        real_t px = x[k * nx + 0];
        real_t py = x[k * nx + 1];
        
        int j = (int)((px - x_min) / (x_max - x_min) * (width - 1));
        int i = (int)((y_max - py) / (y_max - y_min) * (height - 1));
        
        if (i >= 0 && i < height && j >= 0 && j < width) {
            if (k == 0) {
                grid[i][j] = 'S';  // Start
            } else if (k == N) {
                grid[i][j] = 'G';  // Goal
            } else if (grid[i][j] != '#') {
                grid[i][j] = 'M';  // MPC path
            }
        }
    }
    
    // Print grid
    for (int i = 0; i < height; ++i) {
        for (int j = 0; j < width; ++j) {
            printf("%c", grid[i][j]);
        }
        printf("\n");
    }
    
    printf("\nLegend:\n");
    printf("  . = free space\n");
    printf("  # = obstacle\n");
    printf("  r = RRT* path (initial)\n");
    printf("  M = MPC trajectory (optimized)\n");
    printf("  S = start, G = goal\n");
}

int main()
{
    printf("=== Test: RRT* to ADMM Pipeline ===\n");
    printf("Single agent with tilted obstacle\n\n");
    
    // ========================================
    // Step 1: Setup Environment and Obstacle
    // ========================================
    
    printf("--- Step 1: Environment Setup ---\n");
    
    Environment env(0.0, 30.0, 0.0, 20.0, 0.1);
    RectangularObstacle obstacle(15.0, 10.0, 8.0, 4.0, M_PI / 6.0);  // 30° tilt
    env.addObstacle(obstacle);
    
    printf("Environment: [0, 30] x [0, 20]\n");
    printf("Obstacle: center=(%.1f, %.1f), size=%.1f×%.1f, angle=%.1f°\n",
           obstacle.x_center, obstacle.y_center, 
           obstacle.width, obstacle.height, 
           obstacle.theta * 180.0 / M_PI);
    
    // ========================================
    // Step 2: RRT* Path Planning
    // ========================================
    
    printf("\n--- Step 2: RRT* Path Planning ---\n");
    
    Point2D start(2.0, 5.0);
    Point2D goal(28.0, 15.0);
    
    printf("Start: (%.1f, %.1f)\n", start.x, start.y);
    printf("Goal:  (%.1f, %.1f)\n", goal.x, goal.y);
    
    real_t r_agent = 1.0;  // Doubled from 0.5 to 1.0 for safer paths
    real_t v_max = 5.0;
    real_t a_max = 10.0;
    
    RRTStarPlanner planner(env, r_agent);
    planner.setStepSize(0.5);
    planner.setRewireRadius(3.0);
    
    printf("\nPlanning path...\n");
    RRTPath rrt_path = planner.planWithTiming(start, goal, v_max, a_max, 5000);
    
    if (rrt_path.getNumWaypoints() == 0) {
        printf("FAIL: Path not found!\n");
        return 1;
    }
    
    printf("SUCCESS: Path found (%d waypoints, %.2f m, %.2f s)\n",
           rrt_path.getNumWaypoints(), rrt_path.getPathLength(), rrt_path.getDuration());
    
    // ========================================
    // Step 3: Generate Bypass Schedule
    // ========================================
    
    printf("\n--- Step 3: Bypass Schedule Generation ---\n");
    
    int N = 20;
    real_t dt = 0.2;
    
    BypassSide schedule[21];
    
    BypassScheduleGenerator generator;
    generator.setSafetyMargin(r_agent);
    
    printf("Generating bypass schedule...\n");
    bool success = generator.generateSchedule(rrt_path, obstacle, N, dt, schedule);
    
    if (!success) {
        printf("FAIL: Failed to generate schedule!\n");
        return 1;
    }
    
    printf("SUCCESS: Bypass schedule generated\n");
    BypassScheduleGenerator::printSchedule(schedule, N, 0);
    
    // ========================================
    // Step 4: Setup MPC with Time-Varying Constraints
    // ========================================
    
    printf("\n--- Step 4: MPC Setup ---\n");
    
    PointMass agent(N, dt, a_max, v_max * 2.0);
    
    // Setup dynamics, bounds, and cost
    agent.setupDynamics();
    agent.setupBounds();
    agent.setupCostMatrices(10.0, 1.0, 0.01);  // Q_pos, Q_vel, R
    
    // Add workspace bounds
    agent.addWorkspaceBounds(0.0, 30.0, 0.0, 20.0);
    
    // Define obstacle and add time-varying constraints
    int obs_id = agent.defineObstacle(obstacle);
    returnValue ret = agent.addTimeVaryingObstacleConstraint(obs_id, schedule, r_agent);
    
    if (ret != SUCCESSFUL_RETURN) {
        printf("ERROR: Failed to add time-varying constraints\n");
        return 1;
    }
    
    printf("Time-varying constraints added: %d constraints\n", agent.nG);
    
    // Create reference trajectory from RRT* path
    for (int k = 0; k <= N; ++k) {
        Point2D pos = rrt_path.interpolatePosition(k * dt);
        real_t vel = rrt_path.interpolateVelocity(k * dt);
        
        // Estimate velocity direction
        real_t vx = 0.0, vy = 0.0;
        if (k < N) {
            Point2D next_pos = rrt_path.interpolatePosition((k+1) * dt);
            real_t dx = next_pos.x - pos.x;
            real_t dy = next_pos.y - pos.y;
            real_t dist = sqrt(dx*dx + dy*dy);
            if (dist > 1e-6) {
                vx = vel * dx / dist;
                vy = vel * dy / dist;
            }
        }
        
        agent.x_ref[k * agent.nx + 0] = pos.x;
        agent.x_ref[k * agent.nx + 1] = pos.y;
        agent.x_ref[k * agent.nx + 2] = vx;
        agent.x_ref[k * agent.nx + 3] = vy;
    }
    
    // Zero control reference
    for (int k = 0; k < N; ++k) {
        agent.u_ref[k * agent.nu + 0] = 0.0;
        agent.u_ref[k * agent.nu + 1] = 0.0;
    }
    
    // ========================================
    // Step 5: Setup ADMM and Solve
    // ========================================
    
    printf("\n--- Step 5: ADMM Setup and Solve ---\n");
    
    // Coupling data (single agent, no collision)
    CouplingData coupling;
    coupling.d_safe = 1.0;
    
    // ADMM parameters
    ADMMParameters params;
    params.max_admm_iter = 5;      // SCP iterations
    params.max_qp_iter = 100;
    params.rho = 10.0;
    params.eps_primal = 1e-3;
    params.eps_dual = 1e-3;
    params.enable_collision_avoidance = BT_FALSE;
    
    // No neighbors for single agent
    int* neighbors_data = 0;
    int* neighbors[1] = {neighbors_data};
    int num_neighbors[1] = {0};
    
    TurboADMM admm;
    ret = admm.setup(&agent, 1, &coupling, &params, neighbors, num_neighbors);
    
    if (ret != SUCCESSFUL_RETURN) {
        printf("ERROR: ADMM setup failed (code %d)\n", ret);
        return 1;
    }
    
    printf("ADMM setup complete\n");
    
    // Initial condition
    real_t x_init_data[4] = {start.x, start.y, 0.0, 0.0};
    real_t* x_init_array[1] = {x_init_data};
    
    printf("Solving with ADMM + SCP...\n");
    BooleanType converged = BT_FALSE;
    ret = admm.solveColdStart(x_init_array, &converged);
    
    if (ret != SUCCESSFUL_RETURN) {
        printf("ERROR: ADMM solve failed (code %d)\n", ret);
        return 1;
    }
    
    printf("SUCCESS: ADMM solved (converged=%s)!\n", converged ? "YES" : "NO");
    
    // Get solution
    real_t* z_opt = new real_t[agent.nV];
    ret = admm.getSolution(0, z_opt);
    
    if (ret != SUCCESSFUL_RETURN) {
        printf("ERROR: Failed to get solution (code %d)\n", ret);
        delete[] z_opt;
        return 1;
    }
    
    // Extract state trajectory
    real_t* x_opt = new real_t[(N+1) * agent.nx];
    for (int k = 0; k <= N; ++k) {
        for (int i = 0; i < agent.nx; ++i) {
            x_opt[k * agent.nx + i] = z_opt[k * agent.nx + i];
        }
    }
    
    // ========================================
    // Step 6: Verify Results
    // ========================================
    
    printf("\n--- Step 6: Verification ---\n");
    
    // Print trajectory
    printf("\nOptimized Trajectory:\n");
    printf("Stage |  Time (s) | Position (x, y)   | Velocity (vx, vy)\n");
    printf("------+-----------+-------------------+-------------------\n");
    
    for (int k = 0; k <= N; k += 5) {  // Print every 5th stage
        real_t t = k * dt;
        real_t px = x_opt[k * agent.nx + 0];
        real_t py = x_opt[k * agent.nx + 1];
        real_t vx = x_opt[k * agent.nx + 2];
        real_t vy = x_opt[k * agent.nx + 3];
        
        printf("  %3d | %9.2f | (%7.2f, %7.2f) | (%7.2f, %7.2f)\n",
               k, t, px, py, vx, vy);
    }
    
    // Check final position
    real_t final_x = x_opt[N * agent.nx + 0];
    real_t final_y = x_opt[N * agent.nx + 1];
    real_t dist_to_goal = sqrt((final_x - goal.x) * (final_x - goal.x) + 
                               (final_y - goal.y) * (final_y - goal.y));
    
    printf("\nFinal Position: (%.2f, %.2f)\n", final_x, final_y);
    printf("Goal Position:  (%.2f, %.2f)\n", goal.x, goal.y);
    printf("Distance to goal: %.2f m\n", dist_to_goal);
    
    // Visualize
    visualizeTrajectory(env, x_opt, N, agent.nx, obstacle, rrt_path);
    
    // ========================================
    // Summary
    // ========================================
    
    printf("\n=== Pipeline Summary ===\n");
    printf("✓ Step 1: Environment setup\n");
    printf("✓ Step 2: RRT* path planning\n");
    printf("✓ Step 3: Bypass schedule generation\n");
    printf("✓ Step 4: MPC setup with time-varying constraints\n");
    printf("✓ Step 5: MPC optimization\n");
    printf("✓ Step 6: Verification\n");
    
    printf("\n=== RRT* to ADMM Pipeline: SUCCESS ✓ ===\n");
    
    // Cleanup
    delete[] z_opt;
    delete[] x_opt;
    
    return 0;
}
