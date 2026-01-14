/*
 * Test: 16-Agent Crossing Scenario (20×20 workspace)
 *
 * 4 agents from each side (top, right, bottom, left)
 * All agents cross through center, creating maximum interaction
 * 120 collision pairs total
 * 
 * Layout:
 * - Top side (4 agents): Moving down
 * - Right side (4 agents): Moving left
 * - Bottom side (4 agents): Moving up
 * - Left side (4 agents): Moving right
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
    printf("                   14-AGENT TEST WITH CROSSING TRAJECTORIES\n");
    printf("                         20m × 20m Workspace\n");
    printf("================================================================================\n\n");

    const int NUM_AGENTS = 14;
    int N = 20;
    int nx = 4;
    int nu = 2;
    real_t dt = 0.2;
    real_t v_max = 10.0;

    // Create PointMass agents
    PointMass agent0(N, dt, 10.0, v_max);
    PointMass agent1(N, dt, 10.0, v_max);
    PointMass agent2(N, dt, 10.0, v_max);
    PointMass agent3(N, dt, 10.0, v_max);
    PointMass agent4(N, dt, 10.0, v_max);
    PointMass agent5(N, dt, 10.0, v_max);
    PointMass agent6(N, dt, 10.0, v_max);
    PointMass agent7(N, dt, 10.0, v_max);
    PointMass agent8(N, dt, 10.0, v_max);
    PointMass agent9(N, dt, 10.0, v_max);
    PointMass agent10(N, dt, 10.0, v_max);
    PointMass agent11(N, dt, 10.0, v_max);
    PointMass agent12(N, dt, 10.0, v_max);
    PointMass agent13(N, dt, 10.0, v_max);

    // Create array of AgentData for TurboADMM::setup()
    AgentData* agents = new AgentData[NUM_AGENTS];
    agents[0] = agent0;
    agents[1] = agent1;
    agents[2] = agent2;
    agents[3] = agent3;
    agents[4] = agent4;
    agents[5] = agent5;
    agents[6] = agent6;
    agents[7] = agent7;
    agents[8] = agent8;
    agents[9] = agent9;
    agents[10] = agent10;
    agents[11] = agent11;
    agents[12] = agent12;
    agents[13] = agent13;

    // Set cost weights for all agents
    real_t Q_pos = 25.0;
    real_t Q_vel = 0.1;
    real_t R_ctrl = 2.0;

    for (int i = 0; i < NUM_AGENTS; ++i) {
        agents[i].Q[0*nx + 0] = Q_pos;
        agents[i].Q[1*nx + 1] = Q_pos;
        agents[i].Q[2*nx + 2] = Q_vel;
        agents[i].Q[3*nx + 3] = Q_vel;
        agents[i].R[0*nu + 0] = R_ctrl;
        agents[i].R[1*nu + 1] = R_ctrl;
        agents[i].extractDiagonals();
    }

    // Set reference trajectories for 20x20 workspace
    // TOP SIDE (4 agents moving DOWN) - offset left by 0.25m
    // Agent 0: (2.75, 19) -> (2.75, 1)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[0].x_ref[k*nx + 0] = 2.75;
        agents[0].x_ref[k*nx + 1] = 19.0 + alpha * (-18.0);
    }

    // Agent 1: (8.75, 19) -> (8.75, 1)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[1].x_ref[k*nx + 0] = 8.75;
        agents[1].x_ref[k*nx + 1] = 19.0 + alpha * (-18.0);
    }

    // Agent 2: (12.75, 19) -> (12.75, 1)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[2].x_ref[k*nx + 0] = 12.75;
        agents[2].x_ref[k*nx + 1] = 19.0 + alpha * (-18.0);
    }

    // Agent 12: (16.75, 19) -> (16.75, 1)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[12].x_ref[k*nx + 0] = 16.75;
        agents[12].x_ref[k*nx + 1] = 19.0 + alpha * (-18.0);
    }

    // RIGHT SIDE (3 agents moving LEFT) - offset up by 0.25m
    // Agent 3: (19, 15.25) -> (1, 15.25)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[3].x_ref[k*nx + 0] = 19.0 + alpha * (-18.0);
        agents[3].x_ref[k*nx + 1] = 15.25;
    }

    // Agent 4: (19, 11.25) -> (1, 11.25)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[4].x_ref[k*nx + 0] = 19.0 + alpha * (-18.0);
        agents[4].x_ref[k*nx + 1] = 11.25;
    }

    // Agent 5: (19, 7.25) -> (1, 7.25)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[5].x_ref[k*nx + 0] = 19.0 + alpha * (-18.0);
        agents[5].x_ref[k*nx + 1] = 7.25;
    }

    // BOTTOM SIDE (3 agents moving UP) - offset right by 0.25m
    // Agent 6: (17.25, 1) -> (17.25, 19)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[6].x_ref[k*nx + 0] = 17.25;
        agents[6].x_ref[k*nx + 1] = 1.0 + alpha * 18.0;
    }

    // Agent 7: (13.25, 1) -> (13.25, 19)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[7].x_ref[k*nx + 0] = 13.25;
        agents[7].x_ref[k*nx + 1] = 1.0 + alpha * 18.0;
    }

    // Agent 8: (9.25, 1) -> (9.25, 19)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[8].x_ref[k*nx + 0] = 9.25;
        agents[8].x_ref[k*nx + 1] = 1.0 + alpha * 18.0;
    }

    // Agent 13: (3.25, 1) -> (3.25, 19)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[13].x_ref[k*nx + 0] = 3.25;
        agents[13].x_ref[k*nx + 1] = 1.0 + alpha * 18.0;
    }

    // LEFT SIDE (3 agents moving RIGHT) - offset down by 0.25m
    // Agent 9: (1, 11.25) -> (19, 11.25)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[9].x_ref[k*nx + 0] = 1.0 + alpha * 18.0;
        agents[9].x_ref[k*nx + 1] = 11.25;
    }

    // Agent 10: (1, 7.25) -> (19, 7.25)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[10].x_ref[k*nx + 0] = 1.0 + alpha * 18.0;
        agents[10].x_ref[k*nx + 1] = 7.25;
    }

    // Agent 11: (1, 3.25) -> (19, 3.25)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[11].x_ref[k*nx + 0] = 1.0 + alpha * 18.0;
        agents[11].x_ref[k*nx + 1] = 3.25;
    }

    // Set zero velocity and control references for all
    for (int i = 0; i < NUM_AGENTS; ++i) {
        for (int k = 0; k <= N; ++k) {
            agents[i].x_ref[k*nx + 2] = 0.0;
            agents[i].x_ref[k*nx + 3] = 0.0;
        }
        for (int k = 0; k < N; ++k) {
            agents[i].u_ref[k*nu + 0] = 0.0;
            agents[i].u_ref[k*nu + 1] = 0.0;
        }
    }

    // ADMM parameters
    CouplingData coupling;
    coupling.d_safe = 2.05;

    ADMMParameters params;
    params.max_admm_iter = 500;
    params.max_qp_iter = 200;
    params.rho = 80.0;
    params.eps_primal = 1e-5;
    params.eps_dual = 1e-5;
    params.enable_collision_avoidance = BT_TRUE;

    // Setup neighbors for fully connected 14-agent graph (each agent sees all other 13)
    int neighbors_0[13] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
    int neighbors_1[13] = {0, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
    int neighbors_2[13] = {0, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
    int neighbors_3[13] = {0, 1, 2, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13};
    int neighbors_4[13] = {0, 1, 2, 3, 5, 6, 7, 8, 9, 10, 11, 12, 13};
    int neighbors_5[13] = {0, 1, 2, 3, 4, 6, 7, 8, 9, 10, 11, 12, 13};
    int neighbors_6[13] = {0, 1, 2, 3, 4, 5, 7, 8, 9, 10, 11, 12, 13};
    int neighbors_7[13] = {0, 1, 2, 3, 4, 5, 6, 8, 9, 10, 11, 12, 13};
    int neighbors_8[13] = {0, 1, 2, 3, 4, 5, 6, 7, 9, 10, 11, 12, 13};
    int neighbors_9[13] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 10, 11, 12, 13};
    int neighbors_10[13] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 11, 12, 13};
    int neighbors_11[13] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 12, 13};
    int neighbors_12[13] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 13};
    int neighbors_13[13] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    int* neighbors[NUM_AGENTS] = {neighbors_0, neighbors_1, neighbors_2, neighbors_3, 
                                   neighbors_4, neighbors_5, neighbors_6, neighbors_7,
                                   neighbors_8, neighbors_9, neighbors_10, neighbors_11,
                                   neighbors_12, neighbors_13};
    int num_neighbors[NUM_AGENTS] = {13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13, 13};

    printf("Agent Configuration (20m × 20m workspace, 14 agents):\n");
    printf("  TOP SIDE (4 agents moving down):\n");
    printf("    Agent 0: (2.75, 19) -> (2.75, 1)\n");
    printf("    Agent 1: (8.75, 19) -> (8.75, 1)\n");
    printf("    Agent 2: (12.75, 19) -> (12.75, 1)\n");
    printf("    Agent 12: (16.75, 19) -> (16.75, 1)\n");
    printf("  RIGHT SIDE (3 agents moving left):\n");
    printf("    Agent 3: (19, 15.25) -> (1, 15.25)\n");
    printf("    Agent 4: (19, 11.25) -> (1, 11.25)\n");
    printf("    Agent 5: (19, 7.25) -> (1, 7.25)\n");
    printf("  BOTTOM SIDE (4 agents moving up):\n");
    printf("    Agent 6: (17.25, 1) -> (17.25, 19)\n");
    printf("    Agent 7: (13.25, 1) -> (13.25, 19)\n");
    printf("    Agent 8: (9.25, 1) -> (9.25, 19)\n");
    printf("    Agent 13: (3.25, 1) -> (3.25, 19)\n");
    printf("  LEFT SIDE (3 agents moving right):\n");
    printf("    Agent 9: (1, 11.25) -> (19, 11.25)\n");
    printf("    Agent 10: (1, 7.25) -> (19, 7.25)\n");
    printf("    Agent 11: (1, 3.25) -> (19, 3.25)\n");
    printf("\nParameters:\n");
    printf("  Horizon: N=%d, dt=%.1f s, total time=%.1f s\n", N, dt, N*dt);
    printf("  Velocity bounds: -%.1f m/s <= v <= %.1f m/s\n", v_max, v_max);
    printf("  Safety distance: %.1f m\n", coupling.d_safe);
    printf("  ADMM penalty: ρ=%.1f\n", params.rho);
    printf("  Cost weights: Q_pos=%.1f, Q_vel=%.2f, R_ctrl=%.1f\n\n", Q_pos, Q_vel, R_ctrl);

    // Setup and solve
    TurboADMM admm_solver;
    returnValue ret = admm_solver.setup(agents, NUM_AGENTS, &coupling, &params, neighbors, num_neighbors);

    if (ret != SUCCESSFUL_RETURN) {
        printf("Setup failed with code %d\n", ret);
        delete[] agents;
        return -1;
    }

    printf("Setup complete. Starting ADMM solve...\n\n");

    // Setup initial conditions for all agents (20x20 workspace with 0.5m lane offsets)
    real_t x0_init[4] = {2.75, 19.0, 0.0, 0.0};    // Top side (left lane)
    real_t x1_init[4] = {8.75, 19.0, 0.0, 0.0};
    real_t x2_init[4] = {12.75, 19.0, 0.0, 0.0};
    real_t x3_init[4] = {19.0, 15.25, 0.0, 0.0};   // Right side (upper lane)
    real_t x4_init[4] = {19.0, 11.25, 0.0, 0.0};
    real_t x5_init[4] = {19.0, 7.25, 0.0, 0.0};
    real_t x6_init[4] = {17.25, 1.0, 0.0, 0.0};    // Bottom side (right lane)
    real_t x7_init[4] = {13.25, 1.0, 0.0, 0.0};
    real_t x8_init[4] = {9.25, 1.0, 0.0, 0.0};
    real_t x9_init[4] = {1.0, 11.25, 0.0, 0.0};    // Left side (lower lane)
    real_t x10_init[4] = {1.0, 7.25, 0.0, 0.0};
    real_t x11_init[4] = {1.0, 3.25, 0.0, 0.0};
    real_t x12_init[4] = {16.75, 19.0, 0.0, 0.0};  // Top side (4th agent)
    real_t x13_init[4] = {3.25, 1.0, 0.0, 0.0};    // Bottom side (4th agent)
    real_t* x_init[NUM_AGENTS] = {x0_init, x1_init, x2_init, x3_init, 
                                   x4_init, x5_init, x6_init, x7_init,
                                   x8_init, x9_init, x10_init, x11_init,
                                   x12_init, x13_init};

    BooleanType converged = BT_FALSE;

    // Solve
    printf("Solving 16-agent problem...\n");
    double t_start = getTime();
    int admm_iter = admm_solver.solveColdStart(x_init, &converged);
    double t_end = getTime();

    printf("Done! Solve time: %.2f ms\n\n", t_end - t_start);

    // Get statistics
    ADMMStatistics stats;
    admm_solver.getStatistics(&stats);

    printf("\n================================================================================\n");
    printf("                              RESULTS\n");
    printf("================================================================================\n\n");

    printf("ADMM Statistics:\n");
    printf("  Iterations: %d\n", stats.admm_iterations);
    printf("  Total QP iterations: %d\n", stats.total_qp_iterations);
    printf("  Primal residual: %.6f\n", stats.primal_residual);
    printf("  Dual residual: %.6f\n", stats.dual_residual);
    printf("  Converged: %s\n\n", converged ? "YES" : "NO");

    // Extract solutions
    real_t* z_all[NUM_AGENTS];
    for (int i = 0; i < NUM_AGENTS; ++i) {
        z_all[i] = new real_t[agents[i].nV];
        admm_solver.getSolution(i, z_all[i]);
    }

    // Print target positions, final positions, and tracking errors
    printf("Target Positions vs Final Positions:\n");
    printf("Agent | Target Position  | Final Position   | Error (m)\n");
    printf("------|------------------|------------------|----------\n");
    
    real_t total_tracking_error = 0.0;
    for (int i = 0; i < NUM_AGENTS; ++i) {
        int idx = N * (nx + nu);
        real_t final_x = z_all[i][idx];
        real_t final_y = z_all[i][idx+1];
        real_t target_x = agents[i].x_ref[N*nx + 0];
        real_t target_y = agents[i].x_ref[N*nx + 1];
        real_t error = sqrt(pow(final_x - target_x, 2) + pow(final_y - target_y, 2));
        total_tracking_error += error;
        
        printf("  %2d  | (%5.1f, %5.1f) | (%5.2f, %5.2f) | %.3f\n", 
               i, target_x, target_y, final_x, final_y, error);
    }
    printf("\nAverage tracking error: %.3f m\n", total_tracking_error / NUM_AGENTS);

    // Print complete trajectories for all agents
    printf("\n\nComplete Trajectories (all 16 agents):\n");
    printf("========================================\n");
    for (int i = 0; i < NUM_AGENTS; ++i) {
        printf("\nAgent %d trajectory:\n", i);
        printf("  k |    x    |    y    |   vx   |   vy   |   ax   |   ay\n");
        printf("----|---------|---------|--------|--------|--------|--------\n");
        for (int k = 0; k <= N; ++k) {
            int idx = (k == N) ? (N * (nx + nu)) : (k * (nx + nu));
            real_t x = z_all[i][idx];
            real_t y = z_all[i][idx+1];
            real_t vx = z_all[i][idx+2];
            real_t vy = z_all[i][idx+3];
            
            if (k < N) {
                real_t ax = z_all[i][idx+4];
                real_t ay = z_all[i][idx+5];
                printf(" %2d | %7.2f | %7.2f | %6.2f | %6.2f | %6.2f | %6.2f\n", 
                       k, x, y, vx, vy, ax, ay);
            } else {
                printf(" %2d | %7.2f | %7.2f | %6.2f | %6.2f |   -    |   -\n", 
                       k, x, y, vx, vy);
            }
        }
    }

    // Analyze results - check all pairwise distances (120 pairs)
    printf("\n\nCollision Analysis (all 120 pairs):\n");
    printf("Stage | Min Dist | Violating Pair\n");
    printf("------|----------|----------------\n");

    real_t global_min_dist = 1e10;
    int total_violations = 0;

    for (int k = 0; k <= N; k++) {
        int idx = (k == N) ? (N * (nx + nu)) : (k * (nx + nu));
        real_t min_dist_k = 1e10;
        int viol_i = -1, viol_j = -1;

        for (int i = 0; i < NUM_AGENTS; i++) {
            for (int j = i + 1; j < NUM_AGENTS; j++) {
                real_t x_i = z_all[i][idx];
                real_t y_i = z_all[i][idx+1];
                real_t x_j = z_all[j][idx];
                real_t y_j = z_all[j][idx+1];
                real_t dist = sqrt(pow(x_i - x_j, 2) + pow(y_i - y_j, 2));
                if (dist < min_dist_k) {
                    min_dist_k = dist;
                    if (dist < coupling.d_safe) {
                        viol_i = i;
                        viol_j = j;
                    }
                }
            }
        }

        if (min_dist_k < global_min_dist) global_min_dist = min_dist_k;
        if (min_dist_k < 2.0) total_violations++;

        if (viol_i != -1) {
            printf("  %2d  |   %.3f  |   (%d, %d)\n", k, min_dist_k, viol_i, viol_j);
        } else {
            printf("  %2d  |   %.3f  |   -\n", k, min_dist_k);
        }
    }

    // Velocity Analysis
    printf("\nVelocity Profile:\n");
    printf("Stage | A0 Speed | A1 Speed | A2 Speed | A3 Speed | A4 Speed | A5 Speed | A6 Speed | A7 Speed | A8 Speed | A9 Speed |\n");
    printf("------|----------|----------|----------|----------|----------|----------|----------|----------|----------|----------|\n");
    real_t max_speeds[NUM_AGENTS] = {0.0};
    for (int k = 0; k <= N; k++) {
        int idx = (k == N) ? (N * (nx + nu)) : (k * (nx + nu));
        printf("  %2d  |", k);
        for (int i = 0; i < NUM_AGENTS; i++) {
            real_t vx = z_all[i][idx + 2];
            real_t vy = z_all[i][idx + 3];
            real_t speed = sqrt(vx*vx + vy*vy);
            if (speed > max_speeds[i]) max_speeds[i] = speed;
            printf(" %8.2f |", speed);
        }
        printf("\n");
    }

    // Check tracking (use actual reference trajectories)
    real_t tracking_errors[NUM_AGENTS];

    for (int i = 0; i < NUM_AGENTS; i++) {
        real_t x_final = z_all[i][N * (nx + nu) + 0];
        real_t y_final = z_all[i][N * (nx + nu) + 1];
        real_t target_x = agents[i].x_ref[N*nx + 0];
        real_t target_y = agents[i].x_ref[N*nx + 1];
        real_t error_x = x_final - target_x;
        real_t error_y = y_final - target_y;
        tracking_errors[i] = sqrt(error_x*error_x + error_y*error_y);
    }

    printf("\nSummary:\n");
    printf("  Minimum distance: %.3f m (safety: %.1f m)\n", global_min_dist, coupling.d_safe);
    printf("  Violations: %d/%d stages\n", total_violations, N+1);
    printf("  Agent tracking errors:\n");
    for (int i = 0; i < NUM_AGENTS; i++) {
        printf("    Agent %d: %.2f m\n", i, tracking_errors[i]);
    }
    
    // Calculate mean and standard deviation of tracking errors
    real_t mean_error = 0.0;
    for (int i = 0; i < NUM_AGENTS; i++) {
        mean_error += tracking_errors[i];
    }
    mean_error /= NUM_AGENTS;
    
    real_t variance = 0.0;
    for (int i = 0; i < NUM_AGENTS; i++) {
        real_t diff = tracking_errors[i] - mean_error;
        variance += diff * diff;
    }
    variance /= NUM_AGENTS;
    real_t std_dev = sqrt(variance);
    
    printf("  Mean tracking error: %.3f m\n", mean_error);
    printf("  Std dev tracking error: %.3f m\n", std_dev);
    printf("\n");

    bool collision_ok = (global_min_dist >= 2.0);
    bool tracking_ok = true;
    for (int i = 0; i < NUM_AGENTS; i++) {
        if (tracking_errors[i] >= 3.0) tracking_ok = false;
    }

    if (collision_ok && tracking_ok) {
        printf("  ✓ SUCCESS: Good collision avoidance AND tracking!\n");
    } else if (collision_ok) {
        printf("  ⚠ PARTIAL: Collision avoided but poor tracking\n");
    } else if (tracking_ok) {
        printf("  ⚠ PARTIAL: Good tracking but collision violation\n");
    } else {
        printf("  ✗ FAILED: Both collision and tracking issues\n");
    }

    for(int i=0; i<NUM_AGENTS; ++i) {
        delete[] z_all[i];
    }

    delete[] agents;

    printf("\n");
    printf("================================================================================\n");

    return 0;
}
