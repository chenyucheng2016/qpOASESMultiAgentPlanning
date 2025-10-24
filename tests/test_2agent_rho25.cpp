/*
 * Test: 2-Agent with ρ=25 (midpoint between 10 and 50)
 * 
 * Testing if ρ=25 provides better balance between tracking and collision avoidance
 */
 
#include <qpOASES/TurboADMM.hpp>
#include <cstdio>
#include <cmath>
#include <chrono>

double getTime() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double, std::milli>(duration).count();
}

USING_NAMESPACE_QPOASES

int main()
{
    printf("================================================================================\n");
    printf("                    2-AGENT TEST WITH RHO=25\n");
    printf("================================================================================\n\n");
    
    int N = 20;  // Increased from 10 to 20 (total time: 4.0 seconds)
    int nx = 4;  // PointMass state dimension
    int nu = 2;  // PointMass control dimension
    real_t dt = 0.2;
    real_t v_max = 10.0;  // m/s
    
    // Create PointMass agents using polymorphism
    // PointMass inherits from AgentData, so PointMass* can be cast to AgentData*
    PointMass agent0(N, dt, 10.0, v_max);
    PointMass agent1(N, dt, 10.0, v_max);
    
    // Create array of AgentData for TurboADMM::setup()
    // Copy PointMass objects into AgentData array (uses copy constructor)
    AgentData* agents = new AgentData[2];
    agents[0] = agent0;  // Copy PointMass to AgentData (slicing, but OK for data-only class)
    agents[1] = agent1;
    
    // Set cost weights for both agents
    real_t Q_pos = 2.0;   // Position tracking (reduced from 25 to enable y-maneuvering)
    real_t Q_vel = 0.01;   // Velocity penalty
    real_t R_ctrl = 0.1;  // Control cost
    
    // Update Q and R matrices
    for (int i = 0; i < 2; ++i) {
        agents[i].Q[0*nx + 0] = Q_pos;  // x position
        agents[i].Q[1*nx + 1] = Q_pos;  // y position
        agents[i].Q[2*nx + 2] = Q_vel;  // vx velocity
        agents[i].Q[3*nx + 3] = Q_vel;  // vy velocity
        
        agents[i].R[0*nu + 0] = R_ctrl;  // ax control
        agents[i].R[1*nu + 1] = R_ctrl;  // ay control
        
        agents[i].extractDiagonals();
    }
    
    // Set reference trajectory for Agent 0: (0, 4.8) -> (15, 4.8)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[0].x_ref[k*nx + 0] = 0.0 + alpha * 15.0;   // x: 0 -> 15
        agents[0].x_ref[k*nx + 1] = 4.8;                   // y: constant at 4.8
        agents[0].x_ref[k*nx + 2] = 0.0;                   // vx: 0
        agents[0].x_ref[k*nx + 3] = 0.0;                   // vy: 0
    }
    for (int k = 0; k < N; ++k) {
        agents[0].u_ref[k*nu + 0] = 0.0;  // ax
        agents[0].u_ref[k*nu + 1] = 0.0;  // ay
    }
    
    // Set reference trajectory for Agent 1: (15, 5.3) -> (0, 5.3) - COLLISION COURSE
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[1].x_ref[k*nx + 0] = 15.0 + alpha * (-15.0);  // x: 15 -> 0
        agents[1].x_ref[k*nx + 1] = 5.3;                      // y: constant at 5.3
        agents[1].x_ref[k*nx + 2] = 0.0;                      // vx: 0
        agents[1].x_ref[k*nx + 3] = 0.0;                      // vy: 0
    }
    for (int k = 0; k < N; ++k) {
        agents[1].u_ref[k*nu + 0] = 0.0;  // ax
        agents[1].u_ref[k*nu + 1] = 0.0;  // ay
    }
    
    printf("Agent 0: (0, 4.8) -> (15, 4.8)\n");
    printf("Agent 1: (15, 5.3) -> (0, 5.3)\n");
    printf("Time horizon: N=%d, dt=%.1f s, total time=%.1f s\n", N, dt, N*dt);
    printf("Velocity bounds: -10.0 m/s <= v <= 10.0 m/s\n");
    printf("Penalty: rho = 30.0\n\n");
    
    // Setup ADMM with ρ=30
    CouplingData coupling;
    coupling.d_safe = 2.0;
    
    ADMMParameters params;
    params.max_admm_iter = 500;  // Increased to 500 for N=20 with velocity constraints
    params.max_qp_iter = 200;
    params.rho = 30.0;  // Testing with moderate penalty
    params.eps_primal = 1e-4;  // Tightened for strict collision avoidance
    params.eps_dual = 1e-4;
    params.enable_collision_avoidance = BT_TRUE;
    
    int neighbors_0[1] = {1};
    int neighbors_1[1] = {0};
    int* neighbors[2] = {neighbors_0, neighbors_1};
    int num_neighbors[2] = {1, 1};
    
    TurboADMM admm;
    returnValue ret = admm.setup(agents, 2, &coupling, &params, neighbors, num_neighbors);
    
    if (ret != SUCCESSFUL_RETURN) {
        printf("ERROR: ADMM setup failed\n");
        return 1;
    }
    
    real_t x0_init[4] = {0.0, 4.8, 0.0, 0.0};  // Closer spacing (0.5m separation)
    real_t x1_init[4] = {15.0, 5.3, 0.0, 0.0};  // Closer spacing (0.5m separation)
    real_t* x_init[2] = {x0_init, x1_init};
    
    BooleanType converged = BT_FALSE;
    
    // Measure ADMM solve time (excluding printf overhead)
    double start_time = getTime();
    ret = admm.solveColdStart(x_init, &converged);
    double end_time = getTime();
    double solve_time_ms = end_time - start_time;
    
    if (ret != SUCCESSFUL_RETURN) {
        printf("ERROR: ADMM solve failed\n");
        return 1;
    }
    
    printf("ADMM converged: %s\n\n", converged ? "YES" : "NO");
    
    // Get statistics
    ADMMStatistics stats;
    admm.getStatistics(&stats);
    
    printf("ADMM Statistics:\n");
    printf("  Iterations: %d\n", stats.admm_iterations);
    printf("  Total QP iterations: %d\n", stats.total_qp_iterations);
    printf("  Solve time: %.4f ms\n", solve_time_ms);
    printf("  Primal residual: %.6f\n", stats.primal_residual);
    printf("  Dual residual: %.6f\n\n", stats.dual_residual);
    
    // Extract solutions
    real_t* z0 = new real_t[agents[0].nV];
    real_t* z1 = new real_t[agents[1].nV];
    
    admm.getSolution(0, z0);
    admm.getSolution(1, z1);
    
    // Analyze results
    printf("Trajectory Analysis:\n");
    printf("Stage | Agent 0 Pos    | Agent 1 Pos    | Distance | Status\n");
    printf("------|----------------|----------------|----------|--------\n");
    
    real_t min_dist = 1e10;
    int violations = 0;
    
    for (int k = 0; k <= N; k++) {  // Print ALL stages to check for collisions
        int idx0 = (k == N) ? (N * (nx + nu)) : (k * (nx + nu));
        int idx1 = (k == N) ? (N * (nx + nu)) : (k * (nx + nu));
        
        real_t x0 = z0[idx0 + 0];
        real_t y0 = z0[idx0 + 1];
        real_t x1 = z1[idx1 + 0];
        real_t y1 = z1[idx1 + 1];
        
        real_t dist = sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
        
        if (dist < min_dist) min_dist = dist;
        if (dist < coupling.d_safe) violations++;
        
        const char* status = (dist < coupling.d_safe) ? "VIOLATION" : "safe";
        
        printf("  %2d  | (%5.2f, %5.2f) | (%5.2f, %5.2f) | %7.3f  | %s\n",
               k, x0, y0, x1, y1, dist, status);
    }
    
    // Velocity Analysis
    printf("\nVelocity Profile:\n");
    printf("Stage | Agent 0 Vel (vx, vy) | Speed  | Agent 1 Vel (vx, vy) | Speed  |\n");
    printf("------|----------------------|--------|----------------------|--------|\n");
    
    real_t max_speed_0 = 0.0;
    real_t max_speed_1 = 0.0;
    
    for (int k = 0; k <= N; k++) {
        int idx0 = (k == N) ? (N * (nx + nu)) : (k * (nx + nu));
        int idx1 = (k == N) ? (N * (nx + nu)) : (k * (nx + nu));
        
        real_t vx0 = z0[idx0 + 2];
        real_t vy0 = z0[idx0 + 3];
        real_t vx1 = z1[idx1 + 2];
        real_t vy1 = z1[idx1 + 3];
        
        real_t speed0 = sqrt(vx0*vx0 + vy0*vy0);
        real_t speed1 = sqrt(vx1*vx1 + vy1*vy1);
        
        if (speed0 > max_speed_0) max_speed_0 = speed0;
        if (speed1 > max_speed_1) max_speed_1 = speed1;
        
        printf("  %2d  | (%6.2f, %6.2f)   | %6.2f | (%6.2f, %6.2f)   | %6.2f |\n",
               k, vx0, vy0, speed0, vx1, vy1, speed1);
    }
    
    printf("\nMax Speeds:\n");
    printf("  Agent 0: %.2f m/s\n", max_speed_0);
    printf("  Agent 1: %.2f m/s\n", max_speed_1);
    
    // Check tracking
    real_t x0_final = z0[N * (nx + nu) + 0];
    real_t x1_final = z1[N * (nx + nu) + 0];
    
    real_t tracking_error_0 = fabs(x0_final - 15.0);
    real_t tracking_error_1 = fabs(x1_final - 0.0);
    
    printf("\n");
    printf("Summary:\n");
    printf("  Minimum distance: %.3f m (safety: 2.0 m)\n", min_dist);
    printf("  Violations: %d/%d stages\n", violations, N+1);
    printf("  Agent 0 tracking error: %.2f m (final: %.2f, target: 15.0)\n", tracking_error_0, x0_final);
    printf("  Agent 1 tracking error: %.2f m (final: %.2f, target: 0.0)\n", tracking_error_1, x1_final);
    printf("\n");
    
    bool collision_ok = (min_dist >= 2.0);
    bool tracking_ok = (tracking_error_0 < 3.0 && tracking_error_1 < 3.0);
    
    if (collision_ok && tracking_ok) {
        printf("  ✓ SUCCESS: Good collision avoidance AND tracking!\n");
    } else if (collision_ok) {
        printf("  ⚠ PARTIAL: Collision avoided but poor tracking\n");
    } else if (tracking_ok) {
        printf("  ⚠ PARTIAL: Good tracking but collision violation\n");
    } else {
        printf("  ✗ FAILED: Both collision and tracking issues\n");
    }
    
    delete[] z0;
    delete[] z1;
    
    // Cleanup: Delete PointMass array
    delete[] agents;
    
    printf("\n");
    printf("================================================================================\n");
    
    return 0;
}
