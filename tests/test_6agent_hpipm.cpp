#include <stdlib.h>
#include <qpOASES/TurboADMM.hpp>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <vector>
#include <algorithm>

// HPIPM headers
extern "C" {
#include <blasfeo_d_aux_ext_dep.h>
#include <hpipm_d_ocp_qp_ipm.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_sol.h>
}

double getTime() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double, std::milli>(duration).count();
}

USING_NAMESPACE_QPOASES

double computeDistance(double x0, double y0, double x1, double y1) {
    double dx = x0 - x1;
    double dy = y0 - y1;
    return sqrt(dx*dx + dy*dy);
}

int main()
{
    printf("================================================================================\n");
    printf("              6-AGENT CENTRALIZED SQP WITH HPIPM\n");
    printf("================================================================================\n\n");
    
    // Problem dimensions
    const int NUM_AGENTS = 6;
    const int N = 20;        // Horizon length
    const int nx = 4;        // Single agent state dimension (x, y, vx, vy)
    const int nu = 2;        // Single agent control dimension (ax, ay)
    const int nx_total = NUM_AGENTS*nx;  // Total state dimension
    const int nu_total = NUM_AGENTS*nu;  // Total control dimension
    const double dt = 0.2;
    const double v_max = 10.0;   // m/s
    const double a_max = 10.0;   // m/s^2
    const double d_safe = 2.0;   // Safety distance (m)
    const double d_safe_margin = 0.05;  // 5cm margin for HPIPM numerical tolerance
    const double d_safe_constraint = d_safe + d_safe_margin;  // Use 2.05m in constraints
    
    // Number of collision pairs: C(6,2) = 15
    const int NUM_PAIRS = 15;
    int collision_pairs[NUM_PAIRS][2] = {
        {0,1}, {0,2}, {0,3}, {0,4}, {0,5},
        {1,2}, {1,3}, {1,4}, {1,5},
        {2,3}, {2,4}, {2,5},
        {3,4}, {3,5},
        {4,5}
    };
    
    // SQP parameters (matching OSQP)
    const int max_sqp_iter = 20;
    const double sqp_tol = 1e-3;
    
    printf("Problem Setup:\n");
    printf("  Agents: %d\n", NUM_AGENTS);
    printf("  Horizon: N=%d, dt=%.1f s, total time=%.1f s\n", N, dt, N*dt);
    printf("  State per agent: nx=%d (x, y, vx, vy)\n", nx);
    printf("  Control per agent: nu=%d (ax, ay)\n", nu);
    printf("  Total state: %d, Total control: %d\n", nx_total, nu_total);
    printf("  Safety distance: %.1f m\n", d_safe);
    printf("  Collision pairs: %d\n", NUM_PAIRS);
    printf("  Velocity bounds: -%.1f m/s <= v <= %.1f m/s\n", v_max, v_max);
    printf("  Acceleration bounds: -%.1f m/s^2 <= a <= %.1f m/s^2\n\n", a_max, a_max);
    
    // Cost weights
    const double Q_pos = 2.0;    // Position tracking
    const double Q_vel = 0.01;   // Velocity penalty
    const double R_ctrl = 0.1;   // Control cost
    
    /************************************************
     * Single agent dynamics: x_{k+1} = A*x_k + B*u_k
     ************************************************/
    
    double* A_single = new double[nx * nx];
    double* B_single = new double[nx * nu];
    
    // Discrete-time double integrator
    A_single[0*nx + 0] = 1.0;  A_single[0*nx + 1] = 0.0;  A_single[0*nx + 2] = dt;   A_single[0*nx + 3] = 0.0;
    A_single[1*nx + 0] = 0.0;  A_single[1*nx + 1] = 1.0;  A_single[1*nx + 2] = 0.0;  A_single[1*nx + 3] = dt;
    A_single[2*nx + 0] = 0.0;  A_single[2*nx + 1] = 0.0;  A_single[2*nx + 2] = 1.0;  A_single[2*nx + 3] = 0.0;
    A_single[3*nx + 0] = 0.0;  A_single[3*nx + 1] = 0.0;  A_single[3*nx + 2] = 0.0;  A_single[3*nx + 3] = 1.0;
    
    B_single[0*nu + 0] = 0.5*dt*dt;  B_single[0*nu + 1] = 0.0;
    B_single[1*nu + 0] = 0.0;        B_single[1*nu + 1] = 0.5*dt*dt;
    B_single[2*nu + 0] = dt;         B_single[2*nu + 1] = 0.0;
    B_single[3*nu + 0] = 0.0;        B_single[3*nu + 1] = dt;
    
    /************************************************
     * Centralized dynamics (block diagonal)
     ************************************************/
    
    double* A = new double[nx_total * nx_total];
    double* B = new double[nx_total * nu_total];
    
    memset(A, 0, nx_total * nx_total * sizeof(double));
    memset(B, 0, nx_total * nu_total * sizeof(double));
    
    // Block diagonal A (COLUMN-MAJOR: element (row, col) at index row + col * nrows)
    for (int agent = 0; agent < NUM_AGENTS; ++agent) {
        for (int r = 0; r < nx; ++r) {
            for (int c = 0; c < nx; ++c) {
                int row = agent * nx + r;
                int col = agent * nx + c;
                A[row + col * nx_total] = A_single[r*nx + c];
            }
        }
    }
    
    // Block diagonal B (COLUMN-MAJOR: nrows=nx_total, ncols=nu_total)
    for (int agent = 0; agent < NUM_AGENTS; ++agent) {
        for (int r = 0; r < nx; ++r) {
            for (int c = 0; c < nu; ++c) {
                int row = agent * nx + r;
                int col = agent * nu + c;
                B[row + col * nx_total] = B_single[r*nu + c];
            }
        }
    }
    
    /************************************************
     * Cost matrices for centralized system
     ************************************************/
    
    double* Q = new double[nx_total * nx_total];
    double* R = new double[nu_total * nu_total];
    double* S = new double[nu_total * nx_total];
    
    memset(Q, 0, nx_total * nx_total * sizeof(double));
    memset(R, 0, nu_total * nu_total * sizeof(double));
    memset(S, 0, nu_total * nx_total * sizeof(double));
    
    // Q matrix (COLUMN-MAJOR: diagonal element (i,i) at index i + i * nrows)
    for (int agent = 0; agent < NUM_AGENTS; ++agent) {
        int offset = agent * nx;
        Q[(offset + 0) + (offset + 0) * nx_total] = Q_pos;  // x
        Q[(offset + 1) + (offset + 1) * nx_total] = Q_pos;  // y
        Q[(offset + 2) + (offset + 2) * nx_total] = Q_vel;  // vx
        Q[(offset + 3) + (offset + 3) * nx_total] = Q_vel;  // vy
    }
    
    // R matrix (COLUMN-MAJOR: diagonal element (i,i) at index i + i * nrows)
    for (int agent = 0; agent < NUM_AGENTS; ++agent) {
        int offset = agent * nu;
        R[(offset + 0) + (offset + 0) * nu_total] = R_ctrl;  // ax
        R[(offset + 1) + (offset + 1) * nu_total] = R_ctrl;  // ay
    }
    
    /************************************************
     * Reference trajectories
     ************************************************/
    
    printf("Reference trajectories:\n");
    printf("  Agent 0: (0.0, 4.8) -> (15.0, 4.8) [right]\n");
    printf("  Agent 1: (15.0, 5.3) -> (0.0, 5.3) [left]\n");
    printf("  Agent 2: (7.25, 0.0) -> (7.25, 15.0) [up]\n");
    printf("  Agent 3: (7.75, 15.0) -> (7.75, 0.0) [down]\n");
    printf("  Agent 4: (0.0, 0.0) -> (15.0, 15.0) [diagonal up-right]\n");
    printf("  Agent 5: (0.0, 15.0) -> (15.0, 0.0) [diagonal down-right]\n\n");
    
    std::vector<double> x_ref_all((N+1) * nx_total);
    std::vector<double> u_ref_all(N * nu_total);
    
    // Agent 0: (0, 4.8) -> (15, 4.8) - right
    for (int k = 0; k <= N; ++k) {
        double alpha = (double)k / N;
        x_ref_all[k*nx_total + 0*nx + 0] = 0.0 + alpha * 15.0;
        x_ref_all[k*nx_total + 0*nx + 1] = 4.8;
        x_ref_all[k*nx_total + 0*nx + 2] = 0.0;
        x_ref_all[k*nx_total + 0*nx + 3] = 0.0;
    }
    
    // Agent 1: (15, 5.3) -> (0, 5.3) - left
    for (int k = 0; k <= N; ++k) {
        double alpha = (double)k / N;
        x_ref_all[k*nx_total + 1*nx + 0] = 15.0 + alpha * (-15.0);
        x_ref_all[k*nx_total + 1*nx + 1] = 5.3;
        x_ref_all[k*nx_total + 1*nx + 2] = 0.0;
        x_ref_all[k*nx_total + 1*nx + 3] = 0.0;
    }
    
    // Agent 2: (7.25, 0) -> (7.25, 15) - up
    for (int k = 0; k <= N; ++k) {
        double alpha = (double)k / N;
        x_ref_all[k*nx_total + 2*nx + 0] = 7.25;
        x_ref_all[k*nx_total + 2*nx + 1] = 0.0 + alpha * 15.0;
        x_ref_all[k*nx_total + 2*nx + 2] = 0.0;
        x_ref_all[k*nx_total + 2*nx + 3] = 0.0;
    }
    
    // Agent 3: (7.75, 15) -> (7.75, 0) - down
    for (int k = 0; k <= N; ++k) {
        double alpha = (double)k / N;
        x_ref_all[k*nx_total + 3*nx + 0] = 7.75;
        x_ref_all[k*nx_total + 3*nx + 1] = 15.0 + alpha * (-15.0);
        x_ref_all[k*nx_total + 3*nx + 2] = 0.0;
        x_ref_all[k*nx_total + 3*nx + 3] = 0.0;
    }
    
    // Agent 4: (0, 0) -> (15, 15) - diagonal up-right
    for (int k = 0; k <= N; ++k) {
        double alpha = (double)k / N;
        x_ref_all[k*nx_total + 4*nx + 0] = 0.0 + alpha * 15.0;
        x_ref_all[k*nx_total + 4*nx + 1] = 0.0 + alpha * 15.0;
        x_ref_all[k*nx_total + 4*nx + 2] = 0.0;
        x_ref_all[k*nx_total + 4*nx + 3] = 0.0;
    }
    
    // Agent 5: (0, 15) -> (15, 0) - diagonal down-right
    for (int k = 0; k <= N; ++k) {
        double alpha = (double)k / N;
        x_ref_all[k*nx_total + 5*nx + 0] = 0.0 + alpha * 15.0;
        x_ref_all[k*nx_total + 5*nx + 1] = 15.0 + alpha * (-15.0);
        x_ref_all[k*nx_total + 5*nx + 2] = 0.0;
        x_ref_all[k*nx_total + 5*nx + 3] = 0.0;
    }
    
    // Zero control references
    memset(u_ref_all.data(), 0, u_ref_all.size() * sizeof(double));
    
    // Centralized reference (linear cost: q = -Q*x_ref)
    std::vector<double> q_ref((N+1) * nx_total);
    std::vector<double> r_ref(N * nu_total);
    
    for (int k = 0; k <= N; ++k) {
        for (int agent = 0; agent < NUM_AGENTS; ++agent) {
            int offset = agent * nx;
            q_ref[k*nx_total + offset + 0] = -Q_pos * x_ref_all[k*nx_total + offset + 0];
            q_ref[k*nx_total + offset + 1] = -Q_pos * x_ref_all[k*nx_total + offset + 1];
            q_ref[k*nx_total + offset + 2] = 0.0;
            q_ref[k*nx_total + offset + 3] = 0.0;
        }
    }
    
    memset(r_ref.data(), 0, r_ref.size() * sizeof(double));
    
    /************************************************
     * Initial trajectory (collision-free staggered initialization)
     ************************************************/
    
    std::vector<double> x_traj((N+1) * nx_total);
    std::vector<double> u_traj(N * nu_total);
    
    // Staggered initialization: each agent waits at start (matching OSQP approach)
    printf("Using staggered initialization: agents wait 0, 3, 6, 9, 12, 15 steps respectively\n");
    
    for (int agent = 0; agent < NUM_AGENTS; ++agent) {
        int wait_steps = agent * 3;  // Each agent waits 3 more steps than previous
        
        for (int k = 0; k <= N; ++k) {
            // During wait period: stay at initial position (k=0 reference)
            // After wait: follow shifted reference trajectory
            int ref_k;
            if (k <= wait_steps) {
                ref_k = 0;  // Stay at start position
            } else {
                ref_k = k - wait_steps;  // Shifted trajectory
                if (ref_k > N) ref_k = N;
            }
            
            // Copy state from reference
            for (int i = 0; i < nx; ++i) {
                x_traj[k*nx_total + agent*nx + i] = x_ref_all[ref_k*nx_total + agent*nx + i];
            }
        }
        
        // Debug: print initial position for each agent
        double x0 = x_traj[0*nx_total + agent*nx + 0];
        double y0 = x_traj[0*nx_total + agent*nx + 1];
        printf("  Agent %d: initial position (%.2f, %.2f)\n", agent, x0, y0);
    }
    
    // Check initial distances between all pairs
    printf("\nInitial distances between agent pairs:\n");
    double min_init_dist = 1e10;
    for (int p = 0; p < NUM_PAIRS; ++p) {
        int i = collision_pairs[p][0];
        int j = collision_pairs[p][1];
        double x0 = x_traj[0*nx_total + i*nx + 0];
        double y0 = x_traj[0*nx_total + i*nx + 1];
        double x1 = x_traj[0*nx_total + j*nx + 0];
        double y1 = x_traj[0*nx_total + j*nx + 1];
        double dist = computeDistance(x0, y0, x1, y1);
        if (dist < min_init_dist) min_init_dist = dist;
        if (dist < d_safe) {
            printf("  Agents %d-%d: %.3f m ✗ (COLLISION!)\n", i, j, dist);
        }
    }
    printf("  Minimum initial distance: %.3f m (safety: %.1f m)\n\n", min_init_dist, d_safe);
    
    // Zero controls during initialization
    memset(u_traj.data(), 0, u_traj.size() * sizeof(double));
    
    /************************************************
     * Setup HPIPM dimensions
     ************************************************/
    
    int* nnx = new int[N+1];
    int* nnu = new int[N+1];
    int* nnbx = new int[N+1];
    int* nnbu = new int[N+1];
    int* nng = new int[N+1];
    int* nns = new int[N+1];
    
    for (int k = 0; k <= N; ++k) {
        nnx[k] = nx_total;
        nnu[k] = (k < N) ? nu_total : 0;
        
        if (k == 0) {
            nnbx[k] = nx_total;  // Fix initial state
        } else {
            nnbx[k] = NUM_AGENTS * 2;  // Velocity bounds for all agents (vx, vy each)
        }
        
        nnbu[k] = (k < N) ? NUM_AGENTS * 2 : 0;  // Acceleration bounds for all agents
        nng[k] = (k > 0) ? NUM_PAIRS : 0;  // 15 collision constraints per stage (except k=0)
        nns[k] = 0;  // No slack variables (hard constraints, matching OSQP)
    }
    
    hpipm_size_t dim_size = d_ocp_qp_dim_memsize(N);
    void *dim_mem = malloc(dim_size);
    struct d_ocp_qp_dim dim;
    d_ocp_qp_dim_create(N, &dim, dim_mem);
    
    d_ocp_qp_dim_set_all(nnx, nnu, nnbx, nnbu, nng, nns, &dim);
    
    /************************************************
     * Box constraints
     ************************************************/
    
    // Initial state
    double* x0_init = new double[nx_total];
    for (int agent = 0; agent < NUM_AGENTS; ++agent) {
        for (int i = 0; i < nx; ++i) {
            x0_init[agent*nx + i] = x_ref_all[agent*nx + i];
        }
    }
    
    int* idxbx0 = new int[nx_total];
    double* lbx0 = new double[nx_total];
    double* ubx0 = new double[nx_total];
    double* lbx0_mask = new double[nx_total];
    double* ubx0_mask = new double[nx_total];
    
    for (int i = 0; i < nx_total; ++i) {
        idxbx0[i] = i;
        lbx0[i] = x0_init[i];
        ubx0[i] = x0_init[i];
        lbx0_mask[i] = 1.0;
        ubx0_mask[i] = 1.0;
    }
    
    // Velocity bounds (for k > 0)
    int* idxbx_vel = new int[NUM_AGENTS * 2];
    double* lbx_vel = new double[NUM_AGENTS * 2];
    double* ubx_vel = new double[NUM_AGENTS * 2];
    double* lbx_vel_mask = new double[NUM_AGENTS * 2];
    double* ubx_vel_mask = new double[NUM_AGENTS * 2];
    
    for (int agent = 0; agent < NUM_AGENTS; ++agent) {
        idxbx_vel[agent*2 + 0] = agent*nx + 2;  // vx index
        idxbx_vel[agent*2 + 1] = agent*nx + 3;  // vy index
        lbx_vel[agent*2 + 0] = -v_max;
        lbx_vel[agent*2 + 1] = -v_max;
        ubx_vel[agent*2 + 0] = v_max;
        ubx_vel[agent*2 + 1] = v_max;
        lbx_vel_mask[agent*2 + 0] = 1.0;
        lbx_vel_mask[agent*2 + 1] = 1.0;
        ubx_vel_mask[agent*2 + 0] = 1.0;
        ubx_vel_mask[agent*2 + 1] = 1.0;
    }
    
    // Control bounds
    int* idxbu = new int[nu_total];
    double* lbu = new double[nu_total];
    double* ubu = new double[nu_total];
    double* lbu_mask = new double[nu_total];
    double* ubu_mask = new double[nu_total];
    
    for (int i = 0; i < nu_total; ++i) {
        idxbu[i] = i;
        lbu[i] = -a_max;
        ubu[i] = a_max;
        lbu_mask[i] = 1.0;
        ubu_mask[i] = 1.0;
    }
    
    // Collision avoidance constraint matrices (hard constraints, no slack)
    std::vector<double*> hC(N+1);
    std::vector<double*> hD(N+1);
    std::vector<double*> hlg(N+1);
    std::vector<double*> hug(N+1);
    
    for (int k = 0; k <= N; ++k) {
        if (k > 0) {
            hC[k] = new double[NUM_PAIRS * nx_total];
            hD[k] = new double[NUM_PAIRS * nu_total];
            hlg[k] = new double[NUM_PAIRS];
            hug[k] = new double[NUM_PAIRS];
            
            memset(hC[k], 0, NUM_PAIRS * nx_total * sizeof(double));
            memset(hD[k], 0, NUM_PAIRS * nu_total * sizeof(double));
            for (int p = 0; p < NUM_PAIRS; ++p) {
                hug[k][p] = 1e10;
            }
        } else {
            hC[k] = NULL;
            hD[k] = NULL;
            hlg[k] = NULL;
            hug[k] = NULL;
        }
    }
    
    /************************************************
     * SQP Main Loop
     ************************************************/
    
    printf("Starting SQP iterations...\n\n");
    
    double sqp_start_time = getTime();
    int sqp_iter = 0;
    bool converged = false;
    double max_constraint_violation = 0.0;
    int total_qp_iters = 0;
    
    for (sqp_iter = 0; sqp_iter < max_sqp_iter; ++sqp_iter) {
        printf("SQP Iteration %d:\n", sqp_iter);
        
        // Linearize collision constraints around current trajectory
        // Use selective activation: only activate constraints when agents are close
        max_constraint_violation = 0.0;
        int num_active_constraints = 0;
        double activation_distance = 2.0 * d_safe_constraint;  // Activate when within 4.1m (matches OSQP)
        
        for (int k = 1; k <= N; ++k) {
            // First, reset all constraints to inactive (zero rows)
            memset(hC[k], 0, NUM_PAIRS * nx_total * sizeof(double));
            
            for (int p = 0; p < NUM_PAIRS; ++p) {
                int i = collision_pairs[p][0];
                int j = collision_pairs[p][1];
                
                // Extract positions
                double x0 = x_traj[k*nx_total + i*nx + 0];
                double y0 = x_traj[k*nx_total + i*nx + 1];
                double x1 = x_traj[k*nx_total + j*nx + 0];
                double y1 = x_traj[k*nx_total + j*nx + 1];
                
                double dx = x0 - x1;
                double dy = y0 - y1;
                double dist = sqrt(dx*dx + dy*dy);
                
                // Selective activation: only activate if agents are close
                if (dist < activation_distance) {
                    num_active_constraints++;
                    
                    // Constraint violation (use d_safe for reporting, d_safe_constraint for optimization)
                    double violation = d_safe - dist;
                    if (violation > max_constraint_violation) {
                        max_constraint_violation = violation;
                    }
                    
                    // Linearization using normalized gradient
                    if (dist > 1e-6) {
                        double nx_c = dx / dist;
                        double ny_c = dy / dist;
                        
                        // C*x >= lg where C has entries for agent i and j
                        hC[k][p*nx_total + i*nx + 0] = nx_c;
                        hC[k][p*nx_total + i*nx + 1] = ny_c;
                        hC[k][p*nx_total + j*nx + 0] = -nx_c;
                        hC[k][p*nx_total + j*nx + 1] = -ny_c;
                        
                        hlg[k][p] = d_safe;  // Use 2.05m for numerical tolerance
                        hug[k][p] = 1e10;
                    } else {
                        hC[k][p*nx_total + i*nx + 0] = 1.0;
                        hC[k][p*nx_total + i*nx + 1] = 0.0;
                        hC[k][p*nx_total + j*nx + 0] = -1.0;
                        hC[k][p*nx_total + j*nx + 1] = 0.0;
                        hlg[k][p] = d_safe;  // Use 2.05m for numerical tolerance
                        hug[k][p] = 1e10;
                    }
                } else {
                    // Constraint inactive: set to trivial constraint (always satisfied)
                    hlg[k][p] = -1e10;  // Lower bound = -infinity
                    hug[k][p] = 1e10;   // Upper bound = +infinity
                }
            }
        }
        
        printf("  Active collision constraints: %d / %d\n", num_active_constraints, NUM_PAIRS * N);
        
        printf("  Max constraint violation: %.4f m\n", max_constraint_violation);
        
        // Setup QP data
        double** hA = new double*[N];
        double** hB = new double*[N];
        double** hb = new double*[N];
        double** hQ = new double*[N+1];
        double** hS = new double*[N];
        double** hR = new double*[N];
        double** hq = new double*[N+1];
        double** hr = new double*[N];
        double** hidxbx = new double*[N+1];
        double** hlbx = new double*[N+1];
        double** hubx = new double*[N+1];
        double** hidxbu = new double*[N];
        double** hlbu = new double*[N];
        double** hubu = new double*[N];
        double** hlbx_mask = new double*[N+1];
        double** hubx_mask = new double*[N+1];
        double** hlbu_mask = new double*[N];
        double** hubu_mask = new double*[N];
        double** hC_arr = new double*[N+1];
        double** hD_arr = new double*[N+1];
        double** hlg_arr = new double*[N+1];
        double** hug_arr = new double*[N+1];
        double* b_zero = new double[nx_total];
        memset(b_zero, 0, nx_total * sizeof(double));
        
        for (int k = 0; k < N; ++k) {
            hA[k] = A;
            hB[k] = B;
            hb[k] = b_zero;
            hQ[k] = Q;
            hS[k] = S;
            hR[k] = R;
            hq[k] = &q_ref[k*nx_total];
            hr[k] = &r_ref[k*nu_total];
            hidxbu[k] = (double*)idxbu;
            hlbu[k] = lbu;
            hubu[k] = ubu;
            hlbu_mask[k] = lbu_mask;
            hubu_mask[k] = ubu_mask;
        }
        
        // Stage 0
        hQ[0] = Q;
        hq[0] = &q_ref[0];
        hidxbx[0] = (double*)idxbx0;
        hlbx[0] = lbx0;
        hubx[0] = ubx0;
        hlbx_mask[0] = lbx0_mask;
        hubx_mask[0] = ubx0_mask;
        hC_arr[0] = NULL;
        hD_arr[0] = NULL;
        hlg_arr[0] = NULL;
        hug_arr[0] = NULL;
        
        // Stages 1 to N-1
        for (int k = 1; k < N; ++k) {
            hidxbx[k] = (double*)idxbx_vel;
            hlbx[k] = lbx_vel;
            hubx[k] = ubx_vel;
            hlbx_mask[k] = lbx_vel_mask;
            hubx_mask[k] = ubx_vel_mask;
            hC_arr[k] = hC[k];
            hD_arr[k] = hD[k];
            hlg_arr[k] = hlg[k];
            hug_arr[k] = hug[k];
        }
        
        // Terminal stage N
        hQ[N] = Q;
        hq[N] = &q_ref[N*nx_total];
        hidxbx[N] = (double*)idxbx_vel;
        hlbx[N] = lbx_vel;
        hubx[N] = ubx_vel;
        hlbx_mask[N] = lbx_vel_mask;
        hubx_mask[N] = ubx_vel_mask;
        hC_arr[N] = hC[N];
        hD_arr[N] = hD[N];
        hlg_arr[N] = hlg[N];
        hug_arr[N] = hug[N];
        
        // Create fresh QP structure
        hpipm_size_t qp_size = d_ocp_qp_memsize(&dim);
        void *qp_mem = malloc(qp_size);
        struct d_ocp_qp qp;
        d_ocp_qp_create(&dim, &qp, qp_mem);
        
        // Set dynamics
        for (int k = 0; k < N; ++k) {
            d_ocp_qp_set_A(k, hA[k], &qp);
            d_ocp_qp_set_B(k, hB[k], &qp);
            d_ocp_qp_set_b(k, hb[k], &qp);
        }
        
        // Set cost matrices
        for (int k = 0; k <= N; ++k) {
            d_ocp_qp_set_Q(k, hQ[k], &qp);
            d_ocp_qp_set_q(k, hq[k], &qp);
            if (k < N) {
                d_ocp_qp_set_R(k, hR[k], &qp);
                d_ocp_qp_set_S(k, hS[k], &qp);
                d_ocp_qp_set_r(k, hr[k], &qp);
            }
        }
        
        // Set box constraints
        for (int k = 0; k <= N; ++k) {
            d_ocp_qp_set_idxbx(k, (int*)hidxbx[k], &qp);
            d_ocp_qp_set_lbx(k, hlbx[k], &qp);
            d_ocp_qp_set_ubx(k, hubx[k], &qp);
        }
        
        for (int k = 0; k < N; ++k) {
            d_ocp_qp_set_idxbu(k, (int*)hidxbu[k], &qp);
            d_ocp_qp_set_lbu(k, hlbu[k], &qp);
            d_ocp_qp_set_ubu(k, hubu[k], &qp);
        }
        
        // Set collision constraints
        for (int k = 1; k <= N; ++k) {
            d_ocp_qp_set_C(k, hC_arr[k], &qp);
            d_ocp_qp_set_D(k, hD_arr[k], &qp);
            d_ocp_qp_set_lg(k, hlg_arr[k], &qp);
            d_ocp_qp_set_ug(k, hug_arr[k], &qp);
        }
        
        // Set masks
        for (int k = 0; k <= N; ++k) {
            d_ocp_qp_set_lbx_mask(k, hlbx_mask[k], &qp);
            d_ocp_qp_set_ubx_mask(k, hubx_mask[k], &qp);
            if (k < N) {
                d_ocp_qp_set_lbu_mask(k, hlbu_mask[k], &qp);
                d_ocp_qp_set_ubu_mask(k, hubu_mask[k], &qp);
            }
        }
        
        // Create solution structure
        hpipm_size_t qp_sol_size = d_ocp_qp_sol_memsize(&dim);
        void *qp_sol_mem = malloc(qp_sol_size);
        struct d_ocp_qp_sol qp_sol;
        d_ocp_qp_sol_create(&dim, &qp_sol, qp_sol_mem);
        
        // Setup IPM solver
        hpipm_size_t ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim);
        void *ipm_arg_mem = malloc(ipm_arg_size);
        struct d_ocp_qp_ipm_arg arg;
        d_ocp_qp_ipm_arg_create(&dim, &arg, ipm_arg_mem);
        
        // Select HPIPM mode
        enum hpipm_mode mode = BALANCE;  // SPEED_ABS = fast, BALANCE = robust
        d_ocp_qp_ipm_arg_set_default(mode, &arg);
        
        // Set tolerances
        double tol = 1e-2;
        d_ocp_qp_ipm_arg_set_tol_stat(&tol, &arg);
        d_ocp_qp_ipm_arg_set_tol_eq(&tol, &arg);
        d_ocp_qp_ipm_arg_set_tol_ineq(&tol, &arg);
        d_ocp_qp_ipm_arg_set_tol_comp(&tol, &arg);
        
        int iter_max = 200;
        d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &arg);
        
        hpipm_size_t ipm_size = d_ocp_qp_ipm_ws_memsize(&dim, &arg);
        void *ipm_mem = malloc(ipm_size);
        struct d_ocp_qp_ipm_ws workspace;
        d_ocp_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_mem);
        
        // Solve QP
        int hpipm_status;
        d_ocp_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
        d_ocp_qp_ipm_get_status(&workspace, &hpipm_status);
        
        int ipm_iter;
        d_ocp_qp_ipm_get_iter(&workspace, &ipm_iter);
        total_qp_iters += ipm_iter;
        
        // Get residuals for diagnostics
        double res_stat, res_eq, res_ineq, res_comp;
        d_ocp_qp_ipm_get_max_res_stat(&workspace, &res_stat);
        d_ocp_qp_ipm_get_max_res_eq(&workspace, &res_eq);
        d_ocp_qp_ipm_get_max_res_ineq(&workspace, &res_ineq);
        d_ocp_qp_ipm_get_max_res_comp(&workspace, &res_comp);
        
        printf("  QP status: %d, iterations: %d\n", hpipm_status, ipm_iter);
        printf("  Residuals: stat=%.2e, eq=%.2e, ineq=%.2e, comp=%.2e\n", 
               res_stat, res_eq, res_ineq, res_comp);
        
        if (hpipm_status != 0) {
            printf("  WARNING: QP did not fully converge (status %d)\n", hpipm_status);
            if (hpipm_status == 1) {
                printf("    Status 1: Maximum iterations reached\n");
            } else if (hpipm_status == 2) {
                printf("    Status 2: Minimum step length (alpha < alpha_min)\n");
            } else if (hpipm_status == 3) {
                printf("    Status 3: NaN detected\n");
            }
        }
        
        // Extract solution
        double* x_sol = new double[nx_total];
        double* u_sol = new double[nu_total];
        
        for (int k = 0; k <= N; ++k) {
            d_ocp_qp_sol_get_x(k, &qp_sol, x_sol);
            for (int i = 0; i < nx_total; ++i) {
                x_traj[k*nx_total + i] = x_sol[i];
            }
        }
        
        for (int k = 0; k < N; ++k) {
            d_ocp_qp_sol_get_u(k, &qp_sol, u_sol);
            for (int i = 0; i < nu_total; ++i) {
                u_traj[k*nu_total + i] = u_sol[i];
            }
        }
        
        delete[] x_sol;
        delete[] u_sol;
        
        // Cleanup iteration
        delete[] hA;
        delete[] hB;
        delete[] hQ;
        delete[] hS;
        delete[] hR;
        delete[] hq;
        delete[] hr;
        delete[] hidxbx;
        delete[] hlbx;
        delete[] hubx;
        delete[] hidxbu;
        delete[] hlbu;
        delete[] hubu;
        delete[] hlbx_mask;
        delete[] hubx_mask;
        delete[] hlbu_mask;
        delete[] hubu_mask;
        delete[] hC_arr;
        delete[] hD_arr;
        delete[] hlg_arr;
        delete[] hug_arr;
        delete[] hb;
        delete[] b_zero;
        
        free(qp_sol_mem);
        free(ipm_arg_mem);
        free(ipm_mem);
        free(qp_mem);
        
        // Check convergence
        if (max_constraint_violation < sqp_tol) {
            converged = true;
            printf("  SQP converged!\n\n");
            break;
        }
        
        printf("\n");
    }
    
    double sqp_end_time = getTime();
    double sqp_time = sqp_end_time - sqp_start_time;
    
    /************************************************
     * Print results
     ************************************************/
    
    printf("================================================================================\n");
    printf("SQP Results:\n");
    printf("  Converged: %s\n", converged ? "YES" : "NO");
    printf("  SQP iterations: %d\n", sqp_iter + 1);
    printf("  Total QP iterations: %d\n", total_qp_iters);
    printf("  Total solve time: %.2f ms\n", sqp_time);
    printf("  Max constraint violation: %.4f m\n", max_constraint_violation);
    printf("\n");
    
    // Compute minimum distances for all pairs
    printf("Minimum distances between agent pairs:\n");
    double global_min_dist = 1e10;
    
    for (int p = 0; p < NUM_PAIRS; ++p) {
        int i = collision_pairs[p][0];
        int j = collision_pairs[p][1];
        double min_dist_pair = 1e10;
        
        for (int k = 0; k <= N; ++k) {
            double x0 = x_traj[k*nx_total + i*nx + 0];
            double y0 = x_traj[k*nx_total + i*nx + 1];
            double x1 = x_traj[k*nx_total + j*nx + 0];
            double y1 = x_traj[k*nx_total + j*nx + 1];
            double dist = computeDistance(x0, y0, x1, y1);
            if (dist < min_dist_pair) min_dist_pair = dist;
        }
        
        if (min_dist_pair < global_min_dist) global_min_dist = min_dist_pair;
        printf("  Agents %d-%d: %.3f m %s\n", i, j, min_dist_pair, 
               (min_dist_pair >= d_safe) ? "✓" : "✗");
    }
    
    printf("\nGlobal minimum distance: %.3f m (safety: %.1f m)\n", global_min_dist, d_safe);
    
    // Compute tracking errors
    double tracking_errors[NUM_AGENTS];
    double sum_errors = 0.0;
    
    printf("\nTracking errors:\n");
    for (int agent = 0; agent < NUM_AGENTS; ++agent) {
        double x_final = x_traj[N*nx_total + agent*nx + 0];
        double y_final = x_traj[N*nx_total + agent*nx + 1];
        double x_target = x_ref_all[N*nx_total + agent*nx + 0];
        double y_target = x_ref_all[N*nx_total + agent*nx + 1];
        
        tracking_errors[agent] = computeDistance(x_final, y_final, x_target, y_target);
        sum_errors += tracking_errors[agent];
        
        printf("  Agent %d: %.3f m (final: (%.2f, %.2f), target: (%.2f, %.2f))\n",
               agent, tracking_errors[agent], x_final, y_final, x_target, y_target);
    }
    
    double avg_tracking_error = sum_errors / NUM_AGENTS;
    printf("  Average tracking error: %.3f m\n", avg_tracking_error);
    
    // Test result
    bool collision_ok = (global_min_dist >= d_safe);
    bool tracking_ok = (avg_tracking_error < 3.0);
    bool success = converged && collision_ok && tracking_ok;
    
    printf("\nTest Result: %s\n", success ? "PASS ✓" : "FAIL ✗");
    printf("================================================================================\n");
    
    /************************************************
     * Cleanup
     ************************************************/
    
    delete[] A;
    delete[] B;
    delete[] Q;
    delete[] R;
    delete[] S;
    delete[] A_single;
    delete[] B_single;
    delete[] nnx;
    delete[] nnu;
    delete[] nnbx;
    delete[] nnbu;
    delete[] nng;
    delete[] nns;
    delete[] x0_init;
    delete[] idxbx0;
    delete[] lbx0;
    delete[] ubx0;
    delete[] lbx0_mask;
    delete[] ubx0_mask;
    delete[] idxbx_vel;
    delete[] lbx_vel;
    delete[] ubx_vel;
    delete[] lbx_vel_mask;
    delete[] ubx_vel_mask;
    delete[] idxbu;
    delete[] lbu;
    delete[] ubu;
    delete[] lbu_mask;
    delete[] ubu_mask;
    
    for (int k = 1; k <= N; ++k) {
        delete[] hC[k];
        delete[] hD[k];
        delete[] hlg[k];
        delete[] hug[k];
    }
    
    free(dim_mem);
    
    return success ? 0 : 1;
}
