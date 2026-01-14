/*
 * Test: 4-Agent with ρ=25 (midpoint between 10 and 50)
 *
 * Testing multi-agent coordination with symmetric trajectories
 * Agent 0 & 2: move right, Agent 1 & 3: move left
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
    printf("                    4-AGENT TEST WITH RHO=25\n");
    printf("================================================================================\n\n");

    int N = 20;  // Increased from 10 to 20 (total time: 4.0 seconds)
    int nx = 4;  // PointMass state dimension
    int nu = 2;  // PointMass control dimension
    real_t dt = 0.2;
    real_t v_max = 10.0;  // m/s

    // Create PointMass agents using polymorphism
    PointMass agent0(N, dt, 10.0, v_max);
    PointMass agent1(N, dt, 10.0, v_max);
    PointMass agent2(N, dt, 10.0, v_max);
    PointMass agent3(N, dt, 10.0, v_max);

    // Create array of AgentData for TurboADMM::setup()
    AgentData* agents = new AgentData[4];
    agents[0] = agent0;
    agents[1] = agent1;
    agents[2] = agent2;
    agents[3] = agent3;

    // Set cost weights for all agents
    real_t Q_pos = 2.0;   // Position tracking (reduced from 25 to enable y-maneuvering)
    real_t Q_vel = 0.01;   // Velocity penalty
    real_t R_ctrl = 0.1;  // Control cost

    // Update Q and R matrices
    for (int i = 0; i < 4; ++i) {
        agents[i].Q[0*nx + 0] = Q_pos;  // x position
        agents[i].Q[1*nx + 1] = Q_pos;  // y position
        agents[i].Q[2*nx + 2] = Q_vel;  // vx velocity
        agents[i].Q[3*nx + 3] = Q_vel;  // vy velocity

        agents[i].R[0*nu + 0] = R_ctrl;  // ax control
        agents[i].R[1*nu + 1] = R_ctrl;  // ay control

        agents[i].extractDiagonals();
    }

    // Set reference trajectories
    // Agent 0: (0, 4.8) -> (15, 4.8) - right
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

    // Agent 1: (15, 5.3) -> (0, 5.3) - left
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

    // Agent 2: (7.25, 0) -> (7.25, 15) - up
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[2].x_ref[k*nx + 0] = 7.25;                  // x: constant at 7.25
        agents[2].x_ref[k*nx + 1] = 0.0 + alpha * 15.0;   // y: 0 -> 15
        agents[2].x_ref[k*nx + 2] = 0.0;                   // vx: 0
        agents[2].x_ref[k*nx + 3] = 0.0;                   // vy: 0
    }
    for (int k = 0; k < N; ++k) {
        agents[2].u_ref[k*nu + 0] = 0.0;  // ax
        agents[2].u_ref[k*nu + 1] = 0.0;  // ay
    }

    // Agent 3: (7.75, 15) -> (7.75, 0) - down
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[3].x_ref[k*nx + 0] = 7.75;                  // x: constant at 7.75
        agents[3].x_ref[k*nx + 1] = 15.0 + alpha * (-15.0); // y: 15 -> 0
        agents[3].x_ref[k*nx + 2] = 0.0;                   // vx: 0
        agents[3].x_ref[k*nx + 3] = 0.0;                   // vy: 0
    }
    for (int k = 0; k < N; ++k) {
        agents[3].u_ref[k*nu + 0] = 0.0;  // ax
        agents[3].u_ref[k*nu + 1] = 0.0;  // ay
    }

    printf("Agent 0: (0, 4.8) -> (15, 4.8) [right]\n");
    printf("Agent 1: (15, 5.3) -> (0, 5.3) [left]\n");
    printf("Agent 2: (7.25, 0) -> (7.25, 15) [up]\n");
    printf("Agent 3: (7.75, 15) -> (7.75, 0) [down]\n");
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

    // Setup neighbors for fully connected 4-agent graph
    // Each agent can collide with all other agents
    int neighbors_0[3] = {1, 2, 3};
    int neighbors_1[3] = {0, 2, 3};
    int neighbors_2[3] = {0, 1, 3};
    int neighbors_3[3] = {0, 1, 2};
    int* neighbors[4] = {neighbors_0, neighbors_1, neighbors_2, neighbors_3};
    int num_neighbors[4] = {3, 3, 3, 3};

    // Setup and solve
    TurboADMM admm_solver;
    returnValue ret = admm_solver.setup(agents, 4, &coupling, &params, neighbors, num_neighbors);

    if (ret != SUCCESSFUL_RETURN) {
        printf("Setup failed with code: %d\n", ret);
        delete[] agents;
        return -1;
    }

    printf("Setup successful!\n\n");

    // Setup initial conditions for all agents
    real_t x0_init[4] = {0.0, 4.8, 0.0, 0.0};    // Agent 0 start
    real_t x1_init[4] = {15.0, 5.3, 0.0, 0.0};   // Agent 1 start
    real_t x2_init[4] = {7.25, 0.0, 0.0, 0.0};   // Agent 2 start
    real_t x3_init[4] = {7.75, 15.0, 0.0, 0.0};  // Agent 3 start
    real_t* x_init[4] = {x0_init, x1_init, x2_init, x3_init};

    BooleanType converged = BT_FALSE;

    // Solve
    double start_time = getTime();
    ret = admm_solver.solveColdStart(x_init, &converged);
    double end_time = getTime();

    if (ret != SUCCESSFUL_RETURN) {
        printf("Solve failed with code: %d\n", ret);
        delete[] agents;
        return -1;
    }

    printf("Solve completed in %.2f ms\n\n", end_time - start_time);

    // Get statistics
    ADMMStatistics stats;
    admm_solver.getStatistics(&stats);

    printf("ADMM Statistics:\n");
    printf("  Iterations: %d\n", stats.admm_iterations);
    printf("  Total QP iterations: %d\n", stats.total_qp_iterations);
    printf("  Solve time: %.4f ms\n", end_time - start_time);
    printf("  Primal residual: %.6f\n", stats.primal_residual);
    printf("  Dual residual: %.6f\n", stats.dual_residual);
    printf("  Converged: %s\n\n", converged ? "YES" : "NO");

    // Get solutions
    real_t* z0 = new real_t[agents[0].nV];
    real_t* z1 = new real_t[agents[1].nV];
    real_t* z2 = new real_t[agents[2].nV];
    real_t* z3 = new real_t[agents[3].nV];

    admm_solver.getSolution(0, z0);
    admm_solver.getSolution(1, z1);
    admm_solver.getSolution(2, z2);
    admm_solver.getSolution(3, z3);

    // Print final trajectories
    printf("Final Trajectories:\n");
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
    printf("Stage | Agent 0 State (x,y,vx,vy)      | Agent 1 State (x,y,vx,vy)      | Agent 2 State (x,y,vx,vy)      | Agent 3 State (x,y,vx,vy)\n");
    printf("──────|────────────────────────────────|────────────────────────────────|────────────────────────────────|────────────────────────────────\n");

    for (int k = 0; k < N; k++) {
        int idx = k * (nx + nu);
        printf("  %2d  | (%6.2f, %6.2f, %6.2f, %6.2f) | (%6.2f, %6.2f, %6.2f, %6.2f) | (%6.2f, %6.2f, %6.2f, %6.2f) | (%6.2f, %6.2f, %6.2f, %6.2f)\n",
               k,
               z0[idx], z0[idx+1], z0[idx+2], z0[idx+3],
               z1[idx], z1[idx+1], z1[idx+2], z1[idx+3],
               z2[idx], z2[idx+1], z2[idx+2], z2[idx+3],
               z3[idx], z3[idx+1], z3[idx+2], z3[idx+3]);
    }
    // Terminal state
    int idx_N = N * (nx + nu);
    printf("  %2d  | (%6.2f, %6.2f, %6.2f, %6.2f) | (%6.2f, %6.2f, %6.2f, %6.2f) | (%6.2f, %6.2f, %6.2f, %6.2f) | (%6.2f, %6.2f, %6.2f, %6.2f)\n",
           N,
           z0[idx_N], z0[idx_N+1], z0[idx_N+2], z0[idx_N+3],
           z1[idx_N], z1[idx_N+1], z1[idx_N+2], z1[idx_N+3],
           z2[idx_N], z2[idx_N+1], z2[idx_N+2], z2[idx_N+3],
           z3[idx_N], z3[idx_N+1], z3[idx_N+2], z3[idx_N+3]);
    printf("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n\n");

    // Analyze results - check all pairwise distances
    printf("Collision Analysis:\n");
    printf("Stage | A0-A1 Dist | A0-A2 Dist | A0-A3 Dist | A1-A2 Dist | A1-A3 Dist | A2-A3 Dist | Min Dist\n");
    printf("──────|────────────|────────────|────────────|────────────|────────────|────────────|─────────\n");

    real_t global_min_dist = 1e10;
    int total_violations = 0;

    for (int k = 0; k <= N; k++) {
        int idx = (k == N) ? (N * (nx + nu)) : (k * (nx + nu));

        real_t x[4], y[4];
        x[0] = z0[idx]; y[0] = z0[idx+1];
        x[1] = z1[idx]; y[1] = z1[idx+1];
        x[2] = z2[idx]; y[2] = z2[idx+1];
        x[3] = z3[idx]; y[3] = z3[idx+1];

        real_t dist_01 = sqrt((x[0]-x[1])*(x[0]-x[1]) + (y[0]-y[1])*(y[0]-y[1]));
        real_t dist_02 = sqrt((x[0]-x[2])*(x[0]-x[2]) + (y[0]-y[2])*(y[0]-y[2]));
        real_t dist_03 = sqrt((x[0]-x[3])*(x[0]-x[3]) + (y[0]-y[3])*(y[0]-y[3]));
        real_t dist_12 = sqrt((x[1]-x[2])*(x[1]-x[2]) + (y[1]-y[2])*(y[1]-y[2]));
        real_t dist_13 = sqrt((x[1]-x[3])*(x[1]-x[3]) + (y[1]-y[3])*(y[1]-y[3]));
        real_t dist_23 = sqrt((x[2]-x[3])*(x[2]-x[3]) + (y[2]-y[3])*(y[2]-y[3]));

        real_t min_dist_k = fmin(fmin(fmin(fmin(fmin(dist_01, dist_02), dist_03), dist_12), dist_13), dist_23);

        if (min_dist_k < global_min_dist) global_min_dist = min_dist_k;
        if (min_dist_k < coupling.d_safe) total_violations++;

        printf("  %2d  | %8.3f   | %8.3f   | %8.3f   | %8.3f   | %8.3f   | %8.3f   | %7.3f\n",
               k, dist_01, dist_02, dist_03, dist_12, dist_13, dist_23, min_dist_k);
    }

    // Velocity Analysis
    printf("\nVelocity Profile:\n");
    printf("Stage | A0 Speed | A1 Speed | A2 Speed | A3 Speed |\n");
    printf("──────|──────────|──────────|──────────|──────────|\n");

    real_t max_speeds[4] = {0.0, 0.0, 0.0, 0.0};

    for (int k = 0; k <= N; k++) {
        int idx = (k == N) ? (N * (nx + nu)) : (k * (nx + nu));

        real_t speeds[4];
        real_t* zs[4] = {z0, z1, z2, z3};

        for (int i = 0; i < 4; i++) {
            real_t vx = zs[i][idx + 2];
            real_t vy = zs[i][idx + 3];
            speeds[i] = sqrt(vx*vx + vy*vy);
            if (speeds[i] > max_speeds[i]) max_speeds[i] = speeds[i];
        }

        printf("  %2d  | %8.2f | %8.2f | %8.2f | %8.2f |\n",
               k, speeds[0], speeds[1], speeds[2], speeds[3]);
    }

    printf("\nMax Speeds:\n");
    for (int i = 0; i < 4; i++) {
        printf("  Agent %d: %.2f m/s\n", i, max_speeds[i]);
    }

    // Check tracking
    real_t tracking_errors[4];
    real_t targets_x[4] = {15.0, 0.0, 7.25, 7.75};  // x-targets for agents 0,1,2,3
    real_t targets_y[4] = {4.8, 5.3, 15.0, 0.0};    // y-targets for agents 0,1,2,3

    for (int i = 0; i < 4; i++) {
        real_t* z = (i == 0) ? z0 : (i == 1) ? z1 : (i == 2) ? z2 : z3;
        real_t x_final = z[N * (nx + nu) + 0];
        real_t y_final = z[N * (nx + nu) + 1];
        real_t error_x = fabs(x_final - targets_x[i]);
        real_t error_y = fabs(y_final - targets_y[i]);
        tracking_errors[i] = sqrt(error_x*error_x + error_y*error_y);  // Total position error
    }

    printf("\n");
    printf("Summary:\n");
    printf("  Minimum distance: %.3f m (safety: 2.0 m)\n", global_min_dist);
    printf("  Violations: %d/%d stages\n", total_violations, N+1);
    printf("  Agent tracking errors:\n");
    for (int i = 0; i < 4; i++) {
        printf("    Agent %d: %.2f m (final: %.2f, target: %.1f)\n",
               i, tracking_errors[i],
               ((i == 0) ? z0 : (i == 1) ? z1 : (i == 2) ? z2 : z3)[N * (nx + nu) + 0],
               targets_x[i]);
    }
    printf("\n");

    bool collision_ok = (global_min_dist >= 2.0);
    bool tracking_ok = true;
    for (int i = 0; i < 4; i++) {
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

    delete[] z0;
    delete[] z1;
    delete[] z2;
    delete[] z3;

    // Cleanup
    delete[] agents;

    printf("\n");
    printf("================================================================================\n");

    return 0;
}
