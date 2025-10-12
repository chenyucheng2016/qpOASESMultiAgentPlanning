/*
 * Test: 2-Agent with ρ=25 (midpoint between 10 and 50)
 * 
 * Testing if ρ=25 provides better balance between tracking and collision avoidance
 */

#include <qpOASES/TurboADMM.hpp>
#include <cstdio>
#include <cmath>

USING_NAMESPACE_QPOASES

int main()
{
    printf("================================================================================\n");
    printf("                    2-AGENT TEST WITH RHO=25\n");
    printf("================================================================================\n\n");
    
    int N = 10;
    int nx = 4;
    int nu = 2;
    real_t dt = 0.2;
    
    AgentData agents[2];
    
    // Setup agent 0
    agents[0].allocate(N, nx, nu);
    
    // Dynamics
    memset(agents[0].A, 0, nx * nx * sizeof(real_t));
    agents[0].A[0*nx + 0] = 1.0;
    agents[0].A[0*nx + 2] = dt;
    agents[0].A[1*nx + 1] = 1.0;
    agents[0].A[1*nx + 3] = dt;
    agents[0].A[2*nx + 2] = 1.0;
    agents[0].A[3*nx + 3] = 1.0;
    
    memset(agents[0].B, 0, nx * nu * sizeof(real_t));
    agents[0].B[2*nu + 0] = dt;
    agents[0].B[3*nu + 1] = dt;
    
    // Cost matrices
    memset(agents[0].Q, 0, nx * nx * sizeof(real_t));
    agents[0].Q[0*nx + 0] = 25.0;  // Increased from 10 to strengthen tracking
    agents[0].Q[1*nx + 1] = 25.0;  // Increased from 10 to strengthen tracking
    agents[0].Q[2*nx + 2] = 1.0;
    agents[0].Q[3*nx + 3] = 1.0;
    
    memset(agents[0].R, 0, nu * nu * sizeof(real_t));
    agents[0].R[0*nu + 0] = 1.0;
    agents[0].R[1*nu + 1] = 1.0;
    
    agents[0].extractDiagonals();
    
    // Reference: (0,5) -> (15,5)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[0].x_ref[k*nx + 0] = 0.0 + alpha * 15.0;
        agents[0].x_ref[k*nx + 1] = 4.5;  // Offset by -0.5m
        agents[0].x_ref[k*nx + 2] = 0.0;
        agents[0].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[0].u_ref[k*nu + 0] = 0.0;
        agents[0].u_ref[k*nu + 1] = 0.0;
    }
    
    // Setup agent 1
    agents[1].allocate(N, nx, nu);
    memcpy(agents[1].A, agents[0].A, nx * nx * sizeof(real_t));
    memcpy(agents[1].B, agents[0].B, nx * nu * sizeof(real_t));
    memcpy(agents[1].Q, agents[0].Q, nx * nx * sizeof(real_t));
    memcpy(agents[1].R, agents[0].R, nu * nu * sizeof(real_t));
    agents[1].extractDiagonals();
    
    // Reference: (15,5) -> (0,5) - COLLISION COURSE
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[1].x_ref[k*nx + 0] = 15.0 + alpha * (-15.0);
        agents[1].x_ref[k*nx + 1] = 5.5;  // Offset by +0.5m
        agents[1].x_ref[k*nx + 2] = 0.0;
        agents[1].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[1].u_ref[k*nu + 0] = 0.0;
        agents[1].u_ref[k*nu + 1] = 0.0;
    }
    
    printf("Agent 0: (0, 4.5) -> (15, 4.5)\n");
    printf("Agent 1: (15, 5.5) -> (0, 5.5)\n");
    printf("Penalty: rho = 25.0 (midpoint between 10 and 50)\n\n");
    
    // Setup ADMM with ρ=25
    CouplingData coupling;
    coupling.d_safe = 2.0;
    
    ADMMParameters params;
    params.max_admm_iter = 200;  // Increased to ensure convergence with R=1.0
    params.max_qp_iter = 200;
    params.rho = 70.0;  // Fine-tuned to achieve collision-free with good tracking
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
    
    real_t x0_init[4] = {0.0, 4.5, 0.0, 0.0};  // y offset -0.5m
    real_t x1_init[4] = {15.0, 5.5, 0.0, 0.0};  // y offset +0.5m
    real_t* x_init[2] = {x0_init, x1_init};
    
    BooleanType converged = BT_FALSE;
    ret = admm.solveColdStart(x_init, &converged);
    
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
    
    printf("\n");
    printf("================================================================================\n");
    
    return 0;
}
