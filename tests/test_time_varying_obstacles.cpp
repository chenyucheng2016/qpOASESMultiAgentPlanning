/*
 * Test: Time-Varying Obstacle Constraints with SCP
 * 
 * This test demonstrates:
 * 1. Defining a rotated rectangular obstacle
 * 2. Creating a bypass schedule (simulating RRT* output)
 * 3. Adding time-varying constraints
 * 4. ADMM solving with automatic SCP constraint updates
 */

#include <qpOASES/TurboADMM.hpp>
#include <cstdio>
#include <cmath>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

USING_NAMESPACE_QPOASES

int main()
{
    printf("=== Test: Time-Varying Obstacle Constraints with SCP ===\n\n");
    
    // ========================================
    // 1. Setup: Single agent with obstacle
    // ========================================
    
    int N = 10;           // Horizon
    real_t dt = 0.2;      // Time step (0.2s)
    real_t a_max = 10.0;  // Max acceleration
    real_t v_max = 20.0;  // Max velocity
    
    PointMass agent(N, dt, a_max, v_max);
    
    // Setup dynamics, bounds, and cost matrices
    agent.setupDynamics();
    agent.setupBounds();
    agent.setupCostMatrices(10.0, 1.0, 0.01);  // Q_pos=10, Q_vel=1, R=0.01
    
    printf("Agent setup complete: N=%d, dt=%.2f, nx=%d, nu=%d\n", N, dt, agent.nx, agent.nu);
    
    // ========================================
    // 2. Define rotated rectangular obstacle
    // ========================================
    
    // Obstacle at (10, 5), size 4×2, rotated 30 degrees
    real_t obs_x = 10.0;
    real_t obs_y = 5.0;
    real_t obs_width = 4.0;
    real_t obs_height = 2.0;
    real_t obs_theta = M_PI / 6.0;  // 30 degrees
    
    RectangularObstacle obstacle(obs_x, obs_y, obs_width, obs_height, obs_theta);
    int obs_id = agent.defineObstacle(obstacle);
    
    printf("Obstacle defined: center=(%.1f, %.1f), size=%.1f×%.1f, angle=%.1f°, id=%d\n",
           obs_x, obs_y, obs_width, obs_height, obs_theta * 180.0 / M_PI, obs_id);
    
    // ========================================
    // 3. Create bypass schedule
    // ========================================
    
    // Simulate RRT* output: agent goes around obstacle
    // Stages 0-4: Approach from left (BYPASS_LEFT)
    // Stages 5-7: Transition to above (BYPASS_ABOVE)
    // Stages 8-10: Continue above (BYPASS_ABOVE)
    
    BypassSide schedule[N+1];
    for (int k = 0; k <= 4; ++k) {
        schedule[k] = BYPASS_LEFT;
    }
    for (int k = 5; k <= N; ++k) {
        schedule[k] = BYPASS_ABOVE;
    }
    
    printf("Bypass schedule created:\n");
    printf("  k=0-4:  BYPASS_LEFT\n");
    printf("  k=5-%d: BYPASS_ABOVE\n", N);
    
    // ========================================
    // 4. Add time-varying obstacle constraints
    // ========================================
    
    real_t r_agent = 0.5;  // Safety margin
    returnValue ret = agent.addTimeVaryingObstacleConstraint(obs_id, schedule, r_agent);
    
    if (ret != SUCCESSFUL_RETURN) {
        printf("ERROR: Failed to add time-varying obstacle constraint (code %d)\n", ret);
        return 1;
    }
    
    printf("Time-varying obstacle constraints added: %d constraints (N+1)\n", agent.nG);
    printf("  Safety margin: %.2f m\n", r_agent);
    
    // ========================================
    // 5. Setup reference trajectory
    // ========================================
    
    // Reference: straight line from (0, 0) to (20, 10)
    real_t x_start = 0.0;
    real_t y_start = 0.0;
    real_t x_goal = 20.0;
    real_t y_goal = 10.0;
    
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / (real_t)N;
        agent.x_ref[k * agent.nx + 0] = x_start + alpha * (x_goal - x_start);  // x
        agent.x_ref[k * agent.nx + 1] = y_start + alpha * (y_goal - y_start);  // y
        agent.x_ref[k * agent.nx + 2] = 0.0;  // vx
        agent.x_ref[k * agent.nx + 3] = 0.0;  // vy
    }
    
    // Zero control reference
    for (int k = 0; k < N; ++k) {
        agent.u_ref[k * agent.nu + 0] = 0.0;  // ax
        agent.u_ref[k * agent.nu + 1] = 0.0;  // ay
    }
    
    printf("Reference trajectory: (%.1f, %.1f) → (%.1f, %.1f)\n",
           x_start, y_start, x_goal, y_goal);
    
    // ========================================
    // 6. Setup ADMM solver
    // ========================================
    
    int n_agents = 1;
    
    // Coupling data (no collision avoidance for single agent)
    CouplingData coupling;
    coupling.d_safe = 1.0;
    
    // ADMM parameters
    ADMMParameters params;
    params.max_admm_iter = 5;      // 5 ADMM iterations (SCP updates)
    params.max_qp_iter = 100;
    params.rho = 10.0;
    params.eps_primal = 1e-3;
    params.eps_dual = 1e-3;
    params.enable_collision_avoidance = BT_FALSE;  // Single agent, no collision
    
    // No neighbors for single agent
    int* neighbors_data = 0;
    int* neighbors[1] = {neighbors_data};
    int num_neighbors[1] = {0};
    
    TurboADMM admm;
    ret = admm.setup(&agent, n_agents, &coupling, &params, neighbors, num_neighbors);
    
    if (ret != SUCCESSFUL_RETURN) {
        printf("ERROR: ADMM setup failed (code %d)\n", ret);
        return 1;
    }
    
    printf("ADMM setup complete: max_iter=%d, rho=%.1f\n", params.max_admm_iter, params.rho);
    
    // ========================================
    // 7. Solve with SCP
    // ========================================
    
    printf("\n--- Solving with ADMM + SCP ---\n");
    
    // Initial condition
    real_t x_init_data[4] = {0.0, 0.0, 0.0, 0.0};  // Start at origin
    real_t* x_init_array[1] = {x_init_data};
    
    BooleanType converged = BT_FALSE;
    ret = admm.solveColdStart(x_init_array, &converged);
    
    if (ret != SUCCESSFUL_RETURN) {
        printf("ERROR: ADMM solve failed (code %d)\n", ret);
        return 1;
    }
    
    printf("ADMM solve complete: converged=%s\n", converged ? "YES" : "NO");
    
    // ========================================
    // 8. Extract and display solution
    // ========================================
    
    real_t* z_solution = new real_t[agent.nV];
    ret = admm.getSolution(0, z_solution);
    
    if (ret != SUCCESSFUL_RETURN) {
        printf("ERROR: Failed to get solution (code %d)\n", ret);
        delete[] z_solution;
        return 1;
    }
    
    printf("\n--- Solution Trajectory ---\n");
    printf("Stage |    x    |    y    |   vx   |   vy   |   ax   |   ay   |\n");
    printf("------|---------|---------|--------|--------|--------|--------|\n");
    
    for (int k = 0; k <= N; ++k) {
        int idx = (k == N) ? (N * (agent.nx + agent.nu)) : (k * (agent.nx + agent.nu));
        
        real_t x = z_solution[idx + 0];
        real_t y = z_solution[idx + 1];
        real_t vx = z_solution[idx + 2];
        real_t vy = z_solution[idx + 3];
        
        real_t ax = 0.0, ay = 0.0;
        if (k < N) {
            ax = z_solution[idx + 4];
            ay = z_solution[idx + 5];
        }
        
        printf("  %2d  | %7.3f | %7.3f | %6.2f | %6.2f | %6.2f | %6.2f |\n",
               k, x, y, vx, vy, ax, ay);
    }
    
    // ========================================
    // 9. Verify obstacle avoidance
    // ========================================
    
    printf("\n--- Obstacle Avoidance Verification ---\n");
    
    int violations = 0;
    for (int k = 0; k <= N; ++k) {
        int idx = (k == N) ? (N * (agent.nx + agent.nu)) : (k * (agent.nx + agent.nu));
        
        real_t x = z_solution[idx + 0];
        real_t y = z_solution[idx + 1];
        
        // Transform to obstacle's local frame
        real_t dx = x - obs_x;
        real_t dy = y - obs_y;
        
        real_t cos_theta = cos(-obs_theta);
        real_t sin_theta = sin(-obs_theta);
        
        real_t x_local = cos_theta * dx - sin_theta * dy;
        real_t y_local = sin_theta * dx + cos_theta * dy;
        
        // Check if inside obstacle (with safety margin)
        bool inside = (fabs(x_local) < obs_width/2 + r_agent) &&
                      (fabs(y_local) < obs_height/2 + r_agent);
        
        if (inside) {
            printf("  k=%2d: VIOLATION! Position (%.3f, %.3f) inside obstacle\n", k, x, y);
            violations++;
        }
    }
    
    if (violations == 0) {
        printf("  ✓ All stages satisfy obstacle avoidance constraints!\n");
    } else {
        printf("  ✗ %d constraint violations detected\n", violations);
    }
    
    // ========================================
    // 10. Get statistics
    // ========================================
    
    ADMMStatistics stats;
    admm.getStatistics(&stats);
    
    printf("\n--- ADMM Statistics ---\n");
    printf("  ADMM iterations: %d\n", stats.admm_iterations);
    printf("  Total QP iterations: %d\n", stats.total_qp_iterations);
    printf("  Primal residual: %.6f\n", stats.primal_residual);
    printf("  Dual residual: %.6f\n", stats.dual_residual);
    
    // Cleanup
    delete[] z_solution;
    
    printf("\n=== Test Complete ===\n");
    
    return (violations == 0 && converged) ? 0 : 1;
}
