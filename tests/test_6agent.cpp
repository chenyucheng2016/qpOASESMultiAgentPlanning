/*
 * Test: 6-Agent Crossing Scenario
 *
 * Based on the 4-agent test, this adds two more agents with crossing trajectories.
 * Agent 0 & 2: move right, Agent 1 & 3: move left
 * Agent 4: (0,0) -> (15,15)
 * Agent 5: (0,15) -> (15,0)
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
    printf("                    6-AGENT TEST WITH CROSSING TRAJECTORIES\n");
    printf("================================================================================\n\n");

    const int NUM_AGENTS = 6;
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

    // Create array of AgentData for TurboADMM::setup()
    AgentData* agents = new AgentData[NUM_AGENTS];
    agents[0] = agent0;
    agents[1] = agent1;
    agents[2] = agent2;
    agents[3] = agent3;
    agents[4] = agent4;
    agents[5] = agent5;

    // Set cost weights for all agents
    real_t Q_pos = 2.0;
    real_t Q_vel = 0.01;
    real_t R_ctrl = 0.1;

    for (int i = 0; i < NUM_AGENTS; ++i) {
        agents[i].Q[0*nx + 0] = Q_pos;
        agents[i].Q[1*nx + 1] = Q_pos;
        agents[i].Q[2*nx + 2] = Q_vel;
        agents[i].Q[3*nx + 3] = Q_vel;
        agents[i].R[0*nu + 0] = R_ctrl;
        agents[i].R[1*nu + 1] = R_ctrl;
        agents[i].extractDiagonals();
    }

    // Set reference trajectories
    // Agent 0: (0, 4.8) -> (15, 4.8)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[0].x_ref[k*nx + 0] = 0.0 + alpha * 15.0;
        agents[0].x_ref[k*nx + 1] = 4.8;
    }

    // Agent 1: (15, 5.3) -> (0, 5.3)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[1].x_ref[k*nx + 0] = 15.0 + alpha * (-15.0);
        agents[1].x_ref[k*nx + 1] = 5.3;
    }

    // Agent 2: (7.25, 0) -> (7.25, 15)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[2].x_ref[k*nx + 0] = 7.25;
        agents[2].x_ref[k*nx + 1] = 0.0 + alpha * 15.0;
    }

    // Agent 3: (7.75, 15) -> (7.75, 0)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[3].x_ref[k*nx + 0] = 7.75;
        agents[3].x_ref[k*nx + 1] = 15.0 + alpha * (-15.0);
    }

    // Agent 4: (0,0) -> (15,15)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[4].x_ref[k*nx + 0] = 0.0 + alpha * 15.0;
        agents[4].x_ref[k*nx + 1] = 0.0 + alpha * 15.0;
    }

    // Agent 5: (0,15) -> (15,0)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[5].x_ref[k*nx + 0] = 0.0 + alpha * 15.0;
        agents[5].x_ref[k*nx + 1] = 15.0 + alpha * (-15.0);
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
    coupling.d_safe = 2.0;

    ADMMParameters params;
    params.max_admm_iter = 100;
    params.max_qp_iter = 200;
    params.rho = 25.0;
    params.eps_primal = 1e-3;
    params.eps_dual = 1e-3;
    params.enable_collision_avoidance = BT_TRUE;

    // Setup neighbors for fully connected 6-agent graph
    int neighbors_0[5] = {1, 2, 3, 4, 5};
    int neighbors_1[5] = {0, 2, 3, 4, 5};
    int neighbors_2[5] = {0, 1, 3, 4, 5};
    int neighbors_3[5] = {0, 1, 2, 4, 5};
    int neighbors_4[5] = {0, 1, 2, 3, 5};
    int neighbors_5[5] = {0, 1, 2, 3, 4};
    int* neighbors[NUM_AGENTS] = {neighbors_0, neighbors_1, neighbors_2, neighbors_3, neighbors_4, neighbors_5};
    int num_neighbors[NUM_AGENTS] = {5, 5, 5, 5, 5, 5};

    // Setup and solve
    TurboADMM admm_solver;
    returnValue ret = admm_solver.setup(agents, NUM_AGENTS, &coupling, &params, neighbors, num_neighbors);

    if (ret != SUCCESSFUL_RETURN) {
        printf("Setup failed with code: %d\n", ret);
        delete[] agents;
        return -1;
    }
    printf("Setup successful!\n\n");

    // Setup initial conditions for all agents
    real_t x0_init[4] = {0.0, 4.8, 0.0, 0.0};
    real_t x1_init[4] = {15.0, 5.3, 0.0, 0.0};
    real_t x2_init[4] = {7.25, 0.0, 0.0, 0.0};
    real_t x3_init[4] = {7.75, 15.0, 0.0, 0.0};
    real_t x4_init[4] = {0.0, 0.0, 0.0, 0.0};
    real_t x5_init[4] = {0.0, 15.0, 0.0, 0.0};
    real_t* x_init[NUM_AGENTS] = {x0_init, x1_init, x2_init, x3_init, x4_init, x5_init};

    BooleanType converged = BT_FALSE;

    // Solve
    printf("Solving 6-agent problem...\n");
    double start_time = getTime();
    int admm_iter = admm_solver.solveColdStart(x_init, &converged);
    double end_time = getTime();
    if (ret != SUCCESSFUL_RETURN) {
        printf("Solve failed with code: %d\n", ret);
    }

    printf("Done! Solve time: %.2f ms\n\n", end_time - start_time);

    // Get statistics
    ADMMStatistics stats;
    admm_solver.getStatistics(&stats);

    printf("ADMM Statistics:\n");
    printf("  Iterations: %d\n", stats.admm_iterations);
    printf("  Total QP iterations: %d\n", stats.total_qp_iterations);
    printf("  Primal residual: %.6f\n", stats.primal_residual);
    printf("  Dual residual: %.6f\n", stats.dual_residual);
    printf("  Converged: %s\n\n", converged ? "YES" : "NO");

    // Extract solutions
    real_t* z_all[NUM_AGENTS];
    for(int i=0; i<NUM_AGENTS; ++i) {
        z_all[i] = new real_t[agents[i].nV];
        admm_solver.getSolution(i, z_all[i]);
    }

    // Analysis
    printf("ADMM iterations: %d\n\n", admm_iter);

    printf("Final Trajectories:\n");
    printf("Stage | Agent 0 (x,y) | Agent 1 (x,y) | Agent 2 (x,y) | Agent 3 (x,y) | Agent 4 (x,y) | Agent 5 (x,y)\n");
    printf("------|---------------|---------------|---------------|---------------|---------------|---------------\n");
    for (int k = 0; k <= N; k++) {
        int idx = (k == N) ? (N * (nx + nu)) : (k * (nx + nu));
        printf("  %2d  | (%6.2f,%6.2f) | (%6.2f,%6.2f) | (%6.2f,%6.2f) | (%6.2f,%6.2f) | (%6.2f,%6.2f) | (%6.2f,%6.2f)\n", k,
               z_all[0][idx], z_all[0][idx+1],
               z_all[1][idx], z_all[1][idx+1],
               z_all[2][idx], z_all[2][idx+1],
               z_all[3][idx], z_all[3][idx+1],
               z_all[4][idx], z_all[4][idx+1],
               z_all[5][idx], z_all[5][idx+1]);
    }

    // Analyze results - check all pairwise distances
    printf("\nCollision Analysis (all 15 pairs):\n");
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
        if (min_dist_k < coupling.d_safe) total_violations++;

        if (viol_i != -1) {
            printf("  %2d  |   %.3f  |   (%d, %d)\n", k, min_dist_k, viol_i, viol_j);
        } else {
            printf("  %2d  |   %.3f  |   -\n", k, min_dist_k);
        }
    }

    // Velocity Analysis
    printf("\nVelocity Profile:\n");
    printf("Stage | A0 Speed | A1 Speed | A2 Speed | A3 Speed | A4 Speed | A5 Speed |\n");
    printf("------|----------|----------|----------|----------|----------|----------|\n");
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

    // Check tracking
    real_t tracking_errors[NUM_AGENTS];
    real_t targets_x[NUM_AGENTS] = {15.0, 0.0, 7.25, 7.75, 15.0, 15.0};
    real_t targets_y[NUM_AGENTS] = {4.8, 5.3, 15.0, 0.0, 15.0, 0.0};

    for (int i = 0; i < NUM_AGENTS; i++) {
        real_t x_final = z_all[i][N * (nx + nu) + 0];
        real_t y_final = z_all[i][N * (nx + nu) + 1];
        real_t error_x = fabs(x_final - targets_x[i]);
        real_t error_y = fabs(y_final - targets_y[i]);
        tracking_errors[i] = sqrt(error_x*error_x + error_y*error_y);
    }

    printf("\nSummary:\n");
    printf("  Minimum distance: %.3f m (safety: %.1f m)\n", global_min_dist, coupling.d_safe);
    printf("  Violations: %d/%d stages\n", total_violations, N+1);
    printf("  Agent tracking errors:\n");
    for (int i = 0; i < NUM_AGENTS; i++) {
        printf("    Agent %d: %.2f m\n", i, tracking_errors[i]);
    }
    printf("\n");

    bool collision_ok = (global_min_dist >= coupling.d_safe);
    bool tracking_ok = true;
    for (int i = 0; i < NUM_AGENTS; i++) {
        if (tracking_errors[i] >= 3.0) tracking_ok = false;
    }

    if (collision_ok && tracking_ok) {
        printf("  \u2713 SUCCESS: Good collision avoidance AND tracking!\n");
    } else if (collision_ok) {
        printf("  \u26a0 PARTIAL: Collision avoided but poor tracking\n");
    } else if (tracking_ok) {
        printf("  \u26a0 PARTIAL: Good tracking but collision violation\n");
    } else {
        printf("  \u2717 FAILED: Both collision and tracking issues\n");
    }

    for(int i=0; i<NUM_AGENTS; ++i) {
        delete[] z_all[i];
    }

    delete[] agents;

    printf("\n");
    printf("================================================================================\n");

    return 0;
}
