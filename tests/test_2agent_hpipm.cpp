/*
 * Test: 2-Agent Centralized SQP with HPIPM
 * 
 * Centralized Sequential Quadratic Programming approach for multi-agent
 * collision avoidance using HPIPM as the QP solver core.
 * 
 * Problem formulation:
 * - 2 agents with collision avoidance constraints
 * - Centralized state: z = [x0, x1] (both agents stacked)
 * - Nonlinear collision constraints: ||p0 - p1||^2 >= d_safe^2
 * - Linearized in SQP iterations as: 2*(p0-p1)'*(dp0-dp1) >= d_safe^2 - ||p0-p1||^2
 * 
 * SQP Algorithm:
 * 1. Initialize with reference trajectory
 * 2. Repeat until convergence:
 *    a. Linearize collision constraints around current trajectory
 *    b. Solve QP with HPIPM
 *    c. Update trajectory
 *    d. Check convergence
 */

#include <cstdio>
#include <cmath>
#include <chrono>
#include <cstring>
#include <vector>
#include <algorithm>

extern "C" {
#include <hpipm_d_ocp_qp_ipm.h>
#include <hpipm_d_ocp_qp_dim.h>
#include <hpipm_d_ocp_qp.h>
#include <hpipm_d_ocp_qp_sol.h>
#include <hpipm_timing.h>
}

double getTime() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double, std::milli>(duration).count();
}

// Helper: Compute distance between two 2D points
double distance2D(double x0, double y0, double x1, double y1) {
    double dx = x0 - x1;
    double dy = y0 - y1;
    return sqrt(dx*dx + dy*dy);
}

int main()
{
    printf("================================================================================\n");
    printf("              2-AGENT CENTRALIZED SQP WITH HPIPM\n");
    printf("================================================================================\n\n");
    
    // Problem dimensions
    const int N = 20;        // Horizon length
    const int nx = 4;        // Single agent state dimension (x, y, vx, vy)
    const int nu = 2;        // Single agent control dimension (ax, ay)
    const int nx_total = 2*nx;  // Total state dimension (both agents)
    const int nu_total = 2*nu;  // Total control dimension (both agents)
    const double dt = 0.2;
    const double v_max = 10.0;   // m/s
    const double a_max = 10.0;   // m/s^2
    const double d_safe = 2.0;   // Safety distance (m)
    
    // SQP parameters
    const int max_sqp_iter = 20;
    const double sqp_tol = 1e-3;
    
    printf("Problem Setup:\n");
    printf("  Agents: 2\n");
    printf("  Horizon: N=%d, dt=%.1f s, total time=%.1f s\n", N, dt, N*dt);
    printf("  State per agent: nx=%d (x, y, vx, vy)\n", nx);
    printf("  Control per agent: nu=%d (ax, ay)\n", nu);
    printf("  Total state: %d, Total control: %d\n", nx_total, nu_total);
    printf("  Safety distance: %.1f m\n", d_safe);
    printf("  Velocity bounds: -%.1f m/s <= v <= %.1f m/s\n", v_max, v_max);
    printf("  Acceleration bounds: -%.1f m/s^2 <= a <= %.1f m/s^2\n\n", a_max, a_max);
    
    // Cost weights
    const double Q_pos = 2.0;    // Position tracking
    const double Q_vel = 0.01;   // Velocity penalty
    const double R_ctrl = 0.1;   // Control cost
    
    /************************************************
     * Dynamics matrices for centralized system
     ************************************************/
    
    // Single agent dynamics: x[k+1] = A_single*x[k] + B_single*u[k]
    double A_single[nx*nx];
    double B_single[nx*nu];
    
    memset(A_single, 0, sizeof(A_single));
    A_single[0*nx + 0] = 1.0;  // x += vx*dt
    A_single[0*nx + 2] = dt;
    A_single[1*nx + 1] = 1.0;  // y += vy*dt
    A_single[1*nx + 3] = dt;
    A_single[2*nx + 2] = 1.0;  // vx (unchanged)
    A_single[3*nx + 3] = 1.0;  // vy (unchanged)
    
    memset(B_single, 0, sizeof(B_single));
    B_single[2*nu + 0] = dt;   // vx += ax*dt
    B_single[3*nu + 1] = dt;   // vy += ay*dt
    
    // Centralized dynamics: [x0; x1][k+1] = A*[x0; x1][k] + B*[u0; u1][k]
    // A = diag(A_single, A_single), B = diag(B_single, B_single)
    double* A = new double[nx_total * nx_total];
    double* B = new double[nx_total * nu_total];
    memset(A, 0, nx_total * nx_total * sizeof(double));
    memset(B, 0, nx_total * nu_total * sizeof(double));
    
    // Block diagonal A (COLUMN-MAJOR: element (row, col) at index row + col * nrows)
    for (int r = 0; r < nx; ++r) {
        for (int c = 0; c < nx; ++c) {
            // Agent 0 block at rows [0..nx-1], cols [0..nx-1]
            A[r + c * nx_total] = A_single[r*nx + c];
            // Agent 1 block at rows [nx..2*nx-1], cols [nx..2*nx-1]
            A[(nx + r) + (nx + c) * nx_total] = A_single[r*nx + c];
        }
    }
    
    // Block diagonal B (COLUMN-MAJOR: nrows=nx_total, ncols=nu_total)
    for (int r = 0; r < nx; ++r) {
        for (int c = 0; c < nu; ++c) {
            // Agent 0 block: rows r, cols c
            B[r + c * nx_total] = B_single[r*nu + c];
            // Agent 1 block: rows nx + r, cols nu + c
            B[(nx + r) + (nu + c) * nx_total] = B_single[r*nu + c];
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
    Q[0 + 0 * nx_total] = Q_pos;  // Agent 0 x
    Q[1 + 1 * nx_total] = Q_pos;  // Agent 0 y
    Q[2 + 2 * nx_total] = Q_vel;  // Agent 0 vx
    Q[3 + 3 * nx_total] = Q_vel;  // Agent 0 vy
    Q[4 + 4 * nx_total] = Q_pos;  // Agent 1 x
    Q[5 + 5 * nx_total] = Q_pos;  // Agent 1 y
    Q[6 + 6 * nx_total] = Q_vel;  // Agent 1 vx
    Q[7 + 7 * nx_total] = Q_vel;  // Agent 1 vy
    
    // R matrix (COLUMN-MAJOR: diagonal element (i,i) at index i + i * nrows)
    R[0 + 0 * nu_total] = R_ctrl;  // Agent 0 ax
    R[1 + 1 * nu_total] = R_ctrl;  // Agent 0 ay
    R[2 + 2 * nu_total] = R_ctrl;  // Agent 1 ax
    R[3 + 3 * nu_total] = R_ctrl;  // Agent 1 ay
    
    /************************************************
     * Reference trajectories
     ************************************************/
    
    printf("Agent 0: (0.0, 4.8) -> (15.0, 4.8)\n");
    printf("Agent 1: (15.0, 5.3) -> (0.0, 5.3)\n\n");
    
    // Reference for Agent 0: (0, 4.8) -> (15, 4.8)
    std::vector<double> x0_ref((N+1) * nx);
    std::vector<double> u0_ref(N * nu);
    
    for (int k = 0; k <= N; ++k) {
        double alpha = (double)k / N;
        x0_ref[k*nx + 0] = 0.0 + alpha * 15.0;   // x: 0 -> 15
        x0_ref[k*nx + 1] = 4.8;                   // y: constant
        x0_ref[k*nx + 2] = 0.0;                   // vx
        x0_ref[k*nx + 3] = 0.0;                   // vy
    }
    for (int k = 0; k < N; ++k) {
        u0_ref[k*nu + 0] = 0.0;  // ax
        u0_ref[k*nu + 1] = 0.0;  // ay
    }
    
    // Reference for Agent 1: (15, 5.3) -> (0, 5.3)
    std::vector<double> x1_ref((N+1) * nx);
    std::vector<double> u1_ref(N * nu);
    
    for (int k = 0; k <= N; ++k) {
        double alpha = (double)k / N;
        x1_ref[k*nx + 0] = 15.0 - alpha * 15.0;  // x: 15 -> 0
        x1_ref[k*nx + 1] = 5.3;                   // y: constant
        x1_ref[k*nx + 2] = 0.0;                   // vx
        x1_ref[k*nx + 3] = 0.0;                   // vy
    }
    for (int k = 0; k < N; ++k) {
        u1_ref[k*nu + 0] = 0.0;  // ax
        u1_ref[k*nu + 1] = 0.0;  // ay
    }
    
    // Centralized reference (stack both agents)
    std::vector<double> q_ref((N+1) * nx_total);  // Linear cost: q = -Q*x_ref
    std::vector<double> r_ref(N * nu_total);      // Linear cost: r = -R*u_ref
    
    for (int k = 0; k <= N; ++k) {
        // Agent 0
        q_ref[k*nx_total + 0] = -Q_pos * x0_ref[k*nx + 0];
        q_ref[k*nx_total + 1] = -Q_pos * x0_ref[k*nx + 1];
        q_ref[k*nx_total + 2] = 0.0;
        q_ref[k*nx_total + 3] = 0.0;
        // Agent 1
        q_ref[k*nx_total + 4] = -Q_pos * x1_ref[k*nx + 0];
        q_ref[k*nx_total + 5] = -Q_pos * x1_ref[k*nx + 1];
        q_ref[k*nx_total + 6] = 0.0;
        q_ref[k*nx_total + 7] = 0.0;
    }
    
    memset(r_ref.data(), 0, r_ref.size() * sizeof(double));
    
    /************************************************
     * Initial trajectory (collision-free staggered initialization)
     ************************************************/
    
    std::vector<double> x_traj((N+1) * nx_total);  // Current trajectory
    std::vector<double> u_traj(N * nu_total);
    
    // Staggered initialization: Agent 1 waits at start while Agent 0 moves first
    // This avoids the infeasible initial condition where agents collide at midpoint
    int wait_steps_agent1 = 10;  // Agent 1 waits 10 time steps (2 seconds)
    
    // Agent 0: follows reference trajectory normally
    for (int k = 0; k <= N; ++k) {
        for (int i = 0; i < nx; ++i) {
            x_traj[k*nx_total + i] = x0_ref[k*nx + i];
        }
    }
    
    // Agent 1: stays at initial position for wait_steps, then follows shifted reference
    for (int k = 0; k <= N; ++k) {
        int effective_k = (k > wait_steps_agent1) ? (k - wait_steps_agent1) : 0;
        if (effective_k > N) effective_k = N;
        
        for (int i = 0; i < nx; ++i) {
            x_traj[k*nx_total + nx + i] = x1_ref[effective_k*nx + i];
        }
    }
    
    // Controls: zero during wait, then follow reference
    for (int k = 0; k < N; ++k) {
        // Agent 0: normal controls
        for (int i = 0; i < nu; ++i) {
            u_traj[k*nu_total + i] = 0.0;
        }
        
        // Agent 1: zero control during wait
        for (int i = 0; i < nu; ++i) {
            u_traj[k*nu_total + nu + i] = 0.0;
        }
    }
    
    printf("Using staggered initialization: Agent 1 waits %d steps (%.1f s) at start\n\n", 
           wait_steps_agent1, wait_steps_agent1 * dt);
    
    /************************************************
     * Setup HPIPM dimensions
     ************************************************/
    
    int* nnx = new int[N+1];
    int* nnu = new int[N+1];
    int* nnbx = new int[N+1];  // Number of box constraints on states per stage
    int* nnbu = new int[N+1];  // Number of box constraints on controls per stage
    int* nng = new int[N+1];   // Number of general constraints per stage
    int* nns = new int[N+1];   // Number of slack variables per stage (for soft constraints)
    
    for (int k = 0; k <= N; ++k) {
        nnx[k] = nx_total;
        nnu[k] = (k < N) ? nu_total : 0;
        
        if (k == 0) {
            nnbx[k] = nx_total;  // Fix initial state
        } else {
            nnbx[k] = 2 * 2;  // Velocity bounds for both agents (vx, vy each)
        }
        
        nnbu[k] = (k < N) ? 2 * 2 : 0;  // Acceleration bounds for both agents
        nng[k] = (k > 0) ? 1 : 0;  // One collision constraint per stage (except k=0)
        nns[k] = 0;  // No slack variables (hard constraints, matching OSQP)
    }
    
    hpipm_size_t dim_size = d_ocp_qp_dim_memsize(N);
    void *dim_mem = malloc(dim_size);
    struct d_ocp_qp_dim dim;
    d_ocp_qp_dim_create(N, &dim, dim_mem);
    d_ocp_qp_dim_set_all(nnx, nnu, nnbx, nnbu, nng, nns, &dim);
    
    /************************************************
     * Setup constraint data structures
     ************************************************/
    
    // Initial state constraint
    double* x0_init = new double[nx_total];
    x0_init[0] = 0.0;   // Agent 0: x
    x0_init[1] = 4.8;   // Agent 0: y
    x0_init[2] = 0.0;   // Agent 0: vx
    x0_init[3] = 0.0;   // Agent 0: vy
    x0_init[4] = 15.0;  // Agent 1: x
    x0_init[5] = 5.3;   // Agent 1: y
    x0_init[6] = 0.0;   // Agent 1: vx
    x0_init[7] = 0.0;   // Agent 1: vy
    
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
    int* idxbx_vel = new int[4];
    double* lbx_vel = new double[4];
    double* ubx_vel = new double[4];
    double* lbx_vel_mask = new double[4];
    double* ubx_vel_mask = new double[4];
    
    idxbx_vel[0] = 2;  // Agent 0 vx
    idxbx_vel[1] = 3;  // Agent 0 vy
    idxbx_vel[2] = 6;  // Agent 1 vx
    idxbx_vel[3] = 7;  // Agent 1 vy
    
    for (int i = 0; i < 4; ++i) {
        lbx_vel[i] = -v_max;
        ubx_vel[i] = v_max;
        lbx_vel_mask[i] = 1.0;
        ubx_vel_mask[i] = 1.0;
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
            hC[k] = new double[1 * nx_total];  // 1 constraint, nx_total states
            hD[k] = new double[1 * nu_total];  // 1 constraint, nu_total controls
            hlg[k] = new double[1];
            hug[k] = new double[1];
            
            memset(hC[k], 0, 1 * nx_total * sizeof(double));
            memset(hD[k], 0, 1 * nu_total * sizeof(double));
            hug[k][0] = 1e10;  // Upper bound (infinity)
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
        max_constraint_violation = 0.0;
        
        for (int k = 1; k <= N; ++k) {
            // Extract positions
            double x0 = x_traj[k*nx_total + 0];
            double y0 = x_traj[k*nx_total + 1];
            double x1 = x_traj[k*nx_total + 4];
            double y1 = x_traj[k*nx_total + 5];
            
            double dx = x0 - x1;
            double dy = y0 - y1;
            double dist = sqrt(dx*dx + dy*dy);
            
            // Constraint violation
            double violation = d_safe - dist;
            if (violation > max_constraint_violation) {
                max_constraint_violation = violation;
            }
            
            // Linearization using normalized gradient (same as OSQP)
            // Constraint: n · (p0 - p1) >= d_safe
            // where n = (p0 - p1) / ||p0 - p1|| is the normalized direction
            if (dist > 1e-6 && dist < 3.0 * d_safe) {
                double nx_c = dx / dist;  // Normalized x-direction
                double ny_c = dy / dist;  // Normalized y-direction
                
                // C*x >= lg where C = [nx_c, ny_c, 0, 0, -nx_c, -ny_c, 0, 0]
                hC[k][0*nx_total + 0] = nx_c;    // d/dx0
                hC[k][0*nx_total + 1] = ny_c;    // d/dy0
                hC[k][0*nx_total + 4] = -nx_c;   // d/dx1
                hC[k][0*nx_total + 5] = -ny_c;   // d/dy1
                
                // RHS: d_safe (linear constraint on distance)
                hlg[k][0] = d_safe;
                hug[k][0] = 1e10;
            } else {
                // Agents at same position - use arbitrary direction
                hC[k][0*nx_total + 0] = 0.0;
                hC[k][0*nx_total + 1] = 0.0;
                hC[k][0*nx_total + 4] = 0.0;
                hC[k][0*nx_total + 5] = 0.0;
                hlg[k][0] = -1e10;
                hug[k][0] = 1e10;
            }
        }
        
        printf("  Max constraint violation: %.4f m\n", max_constraint_violation);
        
        printf("  [DEBUG] Starting QP data setup...\n");
        //fflush(stdout);
        
        // Setup QP data
        double** hA = new double*[N];
        double** hB = new double*[N];
        double** hb = new double*[N];
        double** hQ = new double*[N+1];
        double** hS = new double*[N+1];
        double** hR = new double*[N+1];
        double** hq = new double*[N+1];
        double** hr = new double*[N+1];
        int** hidxbx = new int*[N+1];
        double** hlbx = new double*[N+1];
        double** hubx = new double*[N+1];
        double** hlbx_mask = new double*[N+1];
        double** hubx_mask = new double*[N+1];
        int** hidxbu = new int*[N+1];
        double** hlbu = new double*[N+1];
        double** hubu = new double*[N+1];
        double** hlbu_mask = new double*[N+1];
        double** hubu_mask = new double*[N+1];
        
        double* b_zero = new double[nx_total];
        memset(b_zero, 0, nx_total * sizeof(double));
        
        for (int k = 0; k < N; ++k) {
            hA[k] = A;
            hB[k] = B;
            hb[k] = b_zero;
        }
        
        for (int k = 0; k <= N; ++k) {
            hQ[k] = Q;
            hS[k] = (k < N) ? S : NULL;  // No cross-term at terminal stage
            hR[k] = (k < N) ? R : NULL;  // No control cost at terminal stage
            hq[k] = &q_ref[k*nx_total];
            hr[k] = (k < N) ? &r_ref[k*nu_total] : NULL;
            
            if (k == 0) {
                hidxbx[k] = idxbx0;
                hlbx[k] = lbx0;
                hubx[k] = ubx0;
                hlbx_mask[k] = lbx0_mask;
                hubx_mask[k] = ubx0_mask;
            } else {
                hidxbx[k] = idxbx_vel;
                hlbx[k] = lbx_vel;
                hubx[k] = ubx_vel;
                hlbx_mask[k] = lbx_vel_mask;
                hubx_mask[k] = ubx_vel_mask;
            }
            
            if (k < N) {
                hidxbu[k] = idxbu;
                hlbu[k] = lbu;
                hubu[k] = ubu;
                hlbu_mask[k] = lbu_mask;
                hubu_mask[k] = ubu_mask;
            } else {
                hidxbu[k] = NULL;
                hlbu[k] = NULL;
                hubu[k] = NULL;
                hlbu_mask[k] = NULL;
                hubu_mask[k] = NULL;
            }
        }
        
        double** hlg_arr = new double*[N+1];
        double** hug_arr = new double*[N+1];
        double** hC_arr = new double*[N+1];
        double** hD_arr = new double*[N+1];
        
        for (int k = 0; k <= N; ++k) {
            hC_arr[k] = hC[k];
            hD_arr[k] = hD[k];
            hlg_arr[k] = hlg[k];
            hug_arr[k] = hug[k];
        }
        
        printf("  [DEBUG] Pointer arrays created, calling d_ocp_qp_set_all...\n");
        fflush(stdout);
        
        // Validate constraint arrays
        printf("  [DEBUG] Validating constraint arrays...\n");
        for (int k = 0; k <= N; ++k) {
            if (k > 0) {
                if (hC_arr[k] == NULL || hD_arr[k] == NULL || hlg_arr[k] == NULL || hug_arr[k] == NULL) {
                    printf("  [ERROR] NULL constraint array at k=%d: hC=%p, hD=%p, hlg=%p, hug=%p\n",
                           k, (void*)hC_arr[k], (void*)hD_arr[k], (void*)hlg_arr[k], (void*)hug_arr[k]);
                    return 1;
                }
            }
        }
        printf("  [DEBUG] Constraint arrays validated\n");
        //fflush(stdout);
        
        // Create fresh QP structure for this SQP iteration
        printf("  [DEBUG] Creating QP structure...\n");
        //fflush(stdout);
        
        hpipm_size_t qp_size = d_ocp_qp_memsize(&dim);
        void *qp_mem = malloc(qp_size);
        struct d_ocp_qp qp;
        d_ocp_qp_create(&dim, &qp, qp_mem);
        
        printf("  [DEBUG] QP structure created\n");
        printf("  [DEBUG] Sample constraint at k=1: C=[%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f], lg=%.2f\n",
               hC_arr[1][0], hC_arr[1][1], hC_arr[1][2], hC_arr[1][3],
               hC_arr[1][4], hC_arr[1][5], hC_arr[1][6], hC_arr[1][7],
               hlg_arr[1][0]);
        //fflush(stdout);
        
        // Use individual setters instead of d_ocp_qp_set_all for better error handling
        printf("  [DEBUG] Setting QP data using individual setters...\n");
        //fflush(stdout);
        
        // Set dynamics
        for (int k = 0; k < N; ++k) {
            d_ocp_qp_set_A(k, hA[k], &qp);
            d_ocp_qp_set_B(k, hB[k], &qp);
            d_ocp_qp_set_b(k, hb[k], &qp);
        }
        printf("  [DEBUG] Dynamics set\n");
        //fflush(stdout);
        
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
        printf("  [DEBUG] Cost matrices set\n");
        //fflush(stdout);
        
        // Set box constraints on states
        for (int k = 0; k <= N; ++k) {
            d_ocp_qp_set_idxbx(k, hidxbx[k], &qp);
            d_ocp_qp_set_lbx(k, hlbx[k], &qp);
            d_ocp_qp_set_ubx(k, hubx[k], &qp);
        }
        printf("  [DEBUG] State bounds set\n");
        //fflush(stdout);
        
        // Set box constraints on controls
        for (int k = 0; k < N; ++k) {
            d_ocp_qp_set_idxbu(k, hidxbu[k], &qp);
            d_ocp_qp_set_lbu(k, hlbu[k], &qp);
            d_ocp_qp_set_ubu(k, hubu[k], &qp);
        }
        printf("  [DEBUG] Control bounds set\n");
        //fflush(stdout);
        
        // Set collision avoidance constraints (general constraints)
        for (int k = 1; k <= N; ++k) {
            d_ocp_qp_set_C(k, hC_arr[k], &qp);
            d_ocp_qp_set_D(k, hD_arr[k], &qp);
            d_ocp_qp_set_lg(k, hlg_arr[k], &qp);
            d_ocp_qp_set_ug(k, hug_arr[k], &qp);
        }
        printf("  [DEBUG] Collision constraints set\n");
        //fflush(stdout);
        
        printf("  [DEBUG] All QP data set, setting masks...\n");
        //fflush(stdout);
        
        for (int k = 0; k <= N; ++k) {
            d_ocp_qp_set_lbx_mask(k, hlbx_mask[k], &qp);
            d_ocp_qp_set_ubx_mask(k, hubx_mask[k], &qp);
            if (k < N) {
                d_ocp_qp_set_lbu_mask(k, hlbu_mask[k], &qp);
                d_ocp_qp_set_ubu_mask(k, hubu_mask[k], &qp);
            }
        }
        
        printf("  [DEBUG] Masks set, creating solution structure...\n");
        fflush(stdout);
        
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
        
        // Select HPIPM mode: SPEED_ABS (fast) or BALANCE (robust)
        //enum hpipm_mode mode = SPEED_ABS;  // SPEED_ABS = fast, BALANCE = robust
        enum hpipm_mode mode = BALANCE;  // Uncomment to use BALANCE mode
        d_ocp_qp_ipm_arg_set_default(mode, &arg);
        
        // Enable verbose diagnostics to see why IPM stalls
        // Note: print_level function may not exist in all HPIPM versions
        // If compilation fails, comment out the print_level line
        // int print_level = 2;  // 0=none, 1=basic, 2=detailed, 3=verbose
        // d_ocp_qp_ipm_arg_set_print_level(&print_level, &arg);
        
        // Set reasonable tolerances
        double tol = 1e-6;
        d_ocp_qp_ipm_arg_set_tol_stat(&tol, &arg);
        d_ocp_qp_ipm_arg_set_tol_eq(&tol, &arg);
        d_ocp_qp_ipm_arg_set_tol_ineq(&tol, &arg);
        d_ocp_qp_ipm_arg_set_tol_comp(&tol, &arg);
        
        // Moderate iteration limit for debugging
        int iter_max = 50;
        d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &arg);
        
        hpipm_size_t ipm_size = d_ocp_qp_ipm_ws_memsize(&dim, &arg);
        void *ipm_mem = malloc(ipm_size);
        struct d_ocp_qp_ipm_ws workspace;
        d_ocp_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_mem);
        
        printf("  Calling HPIPM solver...\n");
        fflush(stdout);
        
        // Solve QP
        int hpipm_status;
        d_ocp_qp_ipm_solve(&qp, &qp_sol, &arg, &workspace);
        
        printf("  HPIPM solve completed\n");
        fflush(stdout);
        
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
            // Continue anyway - HPIPM may have a useful partial solution
        }
        
        // Extract solution and update trajectory
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
        delete[] hlbx_mask;
        delete[] hubx_mask;
        delete[] hidxbu;
        delete[] hlbu;
        delete[] hubu;
        delete[] hlbu_mask;
        delete[] hubu_mask;
        delete[] hlg_arr;
        delete[] hug_arr;
        delete[] hC_arr;
        delete[] hD_arr;
        delete[] b_zero;
        
        free(qp_sol_mem);
        free(ipm_arg_mem);
        free(ipm_mem);
        free(qp_mem);  // Free QP structure memory
        
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
    printf("  Solve time: %.2f ms\n", sqp_time);
    printf("  Max constraint violation: %.4f m\n", max_constraint_violation);
    printf("\n");
    
    // Analyze final trajectory
    printf("Trajectory Analysis:\n");
    printf("Stage | Agent 0 Pos    | Agent 1 Pos    | Distance | Status\n");
    printf("------|----------------|----------------|----------|--------\n");
    
    double min_dist = 1e10;
    int violations = 0;
    
    for (int k = 0; k <= N; ++k) {
        double x0 = x_traj[k*nx_total + 0];
        double y0 = x_traj[k*nx_total + 1];
        double x1 = x_traj[k*nx_total + 4];
        double y1 = x_traj[k*nx_total + 5];
        
        double dist = distance2D(x0, y0, x1, y1);
        
        if (dist < min_dist) min_dist = dist;
        if (dist < d_safe) violations++;
        
        const char* status = (dist < d_safe) ? "VIOLATION" : "safe";
        
        printf("  %2d  | (%5.2f, %5.2f) | (%5.2f, %5.2f) | %7.3f  | %s\n",
               k, x0, y0, x1, y1, dist, status);
    }
    
    // Check tracking
    double x0_final = x_traj[N*nx_total + 0];
    double y0_final = x_traj[N*nx_total + 1];
    double x1_final = x_traj[N*nx_total + 4];
    double y1_final = x_traj[N*nx_total + 5];
    
    double tracking_error_0 = distance2D(x0_final, y0_final, 15.0, 4.8);
    double tracking_error_1 = distance2D(x1_final, y1_final, 0.0, 5.3);
    
    printf("\n");
    printf("Summary:\n");
    printf("  Minimum distance: %.3f m (safety: %.1f m)\n", min_dist, d_safe);
    printf("  Violations: %d/%d stages\n", violations, N+1);
    printf("  Agent 0 tracking error: %.2f m (final: (%.2f, %.2f), target: (15.0, 4.8))\n", 
           tracking_error_0, x0_final, y0_final);
    printf("  Agent 1 tracking error: %.2f m (final: (%.2f, %.2f), target: (0.0, 5.3))\n", 
           tracking_error_1, x1_final, y1_final);
    
    // Average tracking error
    double avg_tracking_error = (tracking_error_0 + tracking_error_1) / 2.0;
    printf("  Average tracking error: %.2f m\n", avg_tracking_error);
    printf("\n");
    
    bool collision_ok = (min_dist >= d_safe);
    bool tracking_ok = (tracking_error_0 < 3.0 && tracking_error_1 < 3.0);
    bool success = converged && collision_ok && tracking_ok;
    
    printf("Test Result: %s\n", success ? "PASS ✓" : "FAIL ✗");
    printf("================================================================================\n");
    
    /************************************************
     * Cleanup
     ************************************************/
    
    delete[] A;
    delete[] B;
    delete[] Q;
    delete[] R;
    delete[] S;
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
