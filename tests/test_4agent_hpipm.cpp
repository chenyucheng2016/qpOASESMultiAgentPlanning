#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <math.h>
#include <chrono>
#include <vector>
#include <iostream>
#include "hpipm_d_ocp_qp.h"
#include "hpipm_d_ocp_qp_sol.h"
#include "hpipm_d_ocp_qp_ipm.h"
#include "hpipm_d_ocp_qp_dim.h"
#include "hpipm_d_ocp_qp_utils.h"

double getTime() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double, std::milli>(duration).count();
}

double computeDistance(double x0, double y0, double x1, double y1) {
    double dx = x0 - x1;
    double dy = y0 - y1;
    return sqrt(dx*dx + dy*dy);
}

int main()
{
    printf("================================================================================\n");
    printf("              4-AGENT CENTRALIZED SQP WITH HPIPM\n");
    printf("================================================================================\n\n");
    
    // Problem dimensions
    const int NUM_AGENTS = 4;
    const int N = 20;
    const int nx = 4;
    const int nu = 2;
    const int nx_total = NUM_AGENTS * nx;
    const int nu_total = NUM_AGENTS * nu;
    const double dt = 0.2;
    const double v_max = 10.0;
    const double a_max = 10.0;
    const double d_safe = 2.0;
    int input = -1;
    char data_path[] = "/mnt/c/Users/cheny/Documents/GitHub/qpOASESMultiAgentPlanning/hpipmSolverData/hpipm_4agent.c";
    char data_path_matlab[] = "/mnt/c/Users/cheny/Documents/GitHub/qpOASESMultiAgentPlanning/hpipmSolverData/hpipm_4agent_solve.m";
    // Number of collision pairs: C(4,2) = 6

    std::string prompt = "Enter the collision pair: 0 for (0,1), 1 for (2-3), 2 for both:";
    std::cout << prompt << std::endl;
    std::cin >> input;
    // Allow up to 2 collision pairs; set NUM_PAIRS at runtime based on input
    int NUM_PAIRS = 1;
    int collision_pairs[2][2] = { {0,1}, {0,0} };
    if (input == 0) {
        NUM_PAIRS = 1;
        collision_pairs[0][0] = 0;
        collision_pairs[0][1] = 1;
    } else if  (input == 1) {
        NUM_PAIRS = 1;
        collision_pairs[0][0] = 2;
        collision_pairs[0][1] = 3;
    } else if (input == 2) {
        NUM_PAIRS = 2;
        collision_pairs[0][0] = 0;
        collision_pairs[0][1] = 1;
        collision_pairs[1][0] = 2;
        collision_pairs[1][1] = 3;
    } else {
        std::cout << "Invalid input!" << std::endl;
        return -1;
    }
    
    // SQP parameters (matching 2-agent test)
    const int max_sqp_iter = 50;
    const double sqp_tol = 1e-3;
    
    printf("Problem Setup:\n");
    printf("  Agents: %d\n", NUM_AGENTS);
    printf("  Horizon: N=%d, dt=%.1f s\n", N, dt);
    printf("  State per agent: nx=%d (x, y, vx, vy)\n", nx);
    printf("  Control per agent: nu=%d (ax, ay)\n", nu);
    printf("  Total state: %d, Total control: %d\n", nx_total, nu_total);
    printf("  Safety distance: %.1f m\n", d_safe);
    printf("  Collision pairs: %d\n\n", NUM_PAIRS);
    
    // Cost weights (matching 2-agent test)
    const double Q_pos = 2.0;
    const double Q_vel = 0.01;
    const double R_ctrl = 0.1;
    
    /************************************************
     * Single agent dynamics: x_{k+1} = A*x_k + B*u_k
     ************************************************/
    
    double A_single[nx*nx];
    double B_single[nx*nu];
    
    // HPIPM uses COLUMN-MAJOR format: A[i,j] = A[i + j*nrows]
    // A = I + dt*[0 0 1 0; 0 0 0 1; 0 0 0 0; 0 0 0 0]
    memset(A_single, 0, nx*nx*sizeof(double));
    A_single[0 + 0*nx] = 1.0;
    A_single[1 + 1*nx] = 1.0;
    A_single[2 + 2*nx] = 1.0;
    A_single[3 + 3*nx] = 1.0;
    A_single[0 + 2*nx] = dt;
    A_single[1 + 3*nx] = dt;
    
    // B = dt*[0 0; 0 0; 1 0; 0 1]
    memset(B_single, 0, nx*nu*sizeof(double));
    B_single[2 + 0*nx] = dt;
    B_single[3 + 1*nx] = dt;
    
    // Build centralized dynamics: block diagonal (column-major)
    double* A = new double[nx_total * nx_total];
    double* B = new double[nx_total * nu_total];
    memset(A, 0, nx_total * nx_total * sizeof(double));
    memset(B, 0, nx_total * nu_total * sizeof(double));
    
    for (int agent = 0; agent < NUM_AGENTS; ++agent) {
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nx; ++j) {
                A[(agent*nx + i) + (agent*nx + j)*nx_total] = A_single[i + j*nx];
            }
        }
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nu; ++j) {
                B[(agent*nx + i) + (agent*nu + j)*nx_total] = B_single[i + j*nx];
            }
        }
    }
    
    // Cost matrices (block diagonal)
    double* Q = new double[nx_total * nx_total];
    double* S = new double[nx_total * nu_total];
    double* R = new double[nu_total * nu_total];
    
    memset(Q, 0, nx_total * nx_total * sizeof(double));
    memset(S, 0, nx_total * nu_total * sizeof(double));
    memset(R, 0, nu_total * nu_total * sizeof(double));
    
    // HPIPM uses COLUMN-MAJOR format: Q[i,j] = Q[i + j*nrows]
    for (int agent = 0; agent < NUM_AGENTS; ++agent) {
        Q[(agent*nx + 0) + (agent*nx + 0)*nx_total] = Q_pos;  // x position
        Q[(agent*nx + 1) + (agent*nx + 1)*nx_total] = Q_pos;  // y position
        Q[(agent*nx + 2) + (agent*nx + 2)*nx_total] = Q_vel;  // vx velocity
        Q[(agent*nx + 3) + (agent*nx + 3)*nx_total] = Q_vel;  // vy velocity
        
        R[(agent*nu + 0) + (agent*nu + 0)*nu_total] = R_ctrl;  // ax control
        R[(agent*nu + 1) + (agent*nu + 1)*nu_total] = R_ctrl;  // ay control
    }
    
    /************************************************
     * Reference trajectories
     ************************************************/
    
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
    
    // Zero control references
    memset(u_ref_all.data(), 0, u_ref_all.size() * sizeof(double));
    
    // Compute gradient references
    std::vector<double> q_ref((N+1) * nx_total);
    std::vector<double> r_ref(N * nu_total);


    
    memset(r_ref.data(), 0, r_ref.size() * sizeof(double));
    
    /************************************************
     * Initial trajectory (manually designed feasible paths)
     ************************************************/
    
    std::vector<double> x_traj((N+1) * nx_total);
    std::vector<double> u_traj(N * nu_total);
    
    printf("Using feasible initialization with larger detours to avoid collisions:\n");
    printf("  Agents take larger detours to ensure d >= 2.0m safety distance\n\n");
    
    // Create collision-free initialization with LARGER detours
    // Need to ensure minimum distance >= 2.0m at all times
    // Agent 0: (0, 4.8) -> (15, 4.8), goes down near center
    // Agent 1: (15, 5.3) -> (0, 5.3), goes up near center
    // Agent 2: (7.25, 0) -> (7.25, 15), goes left near center
    // Agent 3: (7.75, 15) -> (7.75, 0), goes right near center
    
    for (int k = 0; k <= N; ++k) {
        double alpha = (double)k / N;
        double detour_scale = 0.0;  // Smooth detour (0 at start/end, 1 at middle)
        
        // Agent 0: dip down by 2.5m at center (increased from 1.5m)
        x_traj[k*nx_total + 0*nx + 0] = 0.0 + alpha * 15.0;
        x_traj[k*nx_total + 0*nx + 1] = 4.8 - 4.0 * detour_scale;
        x_traj[k*nx_total + 0*nx + 2] = 0.0;
        x_traj[k*nx_total + 0*nx + 3] = 0.0;
        
        // Agent 1: rise up by 2.5m at center (increased from 1.5m)
        x_traj[k*nx_total + 1*nx + 0] = 15.0 - alpha * 15.0;
        x_traj[k*nx_total + 1*nx + 1] = 5.3 + 4.0 * detour_scale;
        x_traj[k*nx_total + 1*nx + 2] = 0.0;
        x_traj[k*nx_total + 1*nx + 3] = 0.0;
        
        // Agent 2: curve left by 3.0m at center (increased from 2.0m)
        x_traj[k*nx_total + 2*nx + 0] = 7.25 - 4.0 * detour_scale;
        x_traj[k*nx_total + 2*nx + 1] = 0.0 + alpha * 15.0;
        x_traj[k*nx_total + 2*nx + 2] = 0.0;
        x_traj[k*nx_total + 2*nx + 3] = 0.0;
        
        // Agent 3: curve right by 3.0m at center (increased from 2.0m)
        x_traj[k*nx_total + 3*nx + 0] = 7.75 + 4.0 * detour_scale;
        x_traj[k*nx_total + 3*nx + 1] = 15.0 - alpha * 15.0;
        x_traj[k*nx_total + 3*nx + 2] = 0.0;
        x_traj[k*nx_total + 3*nx + 3] = 0.0;
    }
    
    // Print initial positions
    for (int agent = 0; agent < NUM_AGENTS; ++agent) {
        double x0 = x_traj[0*nx_total + agent*nx + 0];
        double y0 = x_traj[0*nx_total + agent*nx + 1];
        printf("  Agent %d: initial position (%.2f, %.2f)\n", agent, x0, y0);
    }
    
    // Check feasibility of initialized trajectory
    printf("\nChecking feasibility of initialized trajectory:\n");
    int total_violations = 0;
    double min_dist_overall = 1e10;
    
    for (int k = 0; k <= N; ++k) {
        int violations_at_k = 0;
        double min_dist_at_k = 1e10;
        
        for (int p = 0; p < NUM_PAIRS; ++p) {
            int i = collision_pairs[p][0];
            int j = collision_pairs[p][1];
            double xi = x_traj[k*nx_total + i*nx + 0];
            double yi = x_traj[k*nx_total + i*nx + 1];
            double xj = x_traj[k*nx_total + j*nx + 0];
            double yj = x_traj[k*nx_total + j*nx + 1];
            double dist = computeDistance(xi, yi, xj, yj);
            
            if (dist < min_dist_at_k) min_dist_at_k = dist;
            if (dist < min_dist_overall) min_dist_overall = dist;
            
            if (dist < d_safe) {
                violations_at_k++;
                total_violations++;
            }
        }
        
        if (violations_at_k > 0) {
            printf("  k=%2d: %d violations, min_dist=%.3f m ✗\n", k, violations_at_k, min_dist_at_k);
        } else if (k % 5 == 0) {  // Print every 5th step if no violations
            printf("  k=%2d: 0 violations, min_dist=%.3f m ✓\n", k, min_dist_at_k);
        }
    }
    for (int k = 0; k <= N; ++k) {
        for (int i = 0; i < nx_total; ++i) {
            q_ref[k*nx_total + i] = 0.0;
            for (int j = 0; j < nx_total; ++j) {
                q_ref[k*nx_total + i] -= Q[i*nx_total + j] * x_traj[k*nx_total + j];
            }
        }
    }

    printf("\nFeasibility check summary:\n");
    printf("  Total violations: %d / %d (%.1f%%)\n", 
           total_violations, NUM_PAIRS * (N+1), 
           100.0 * total_violations / (NUM_PAIRS * (N+1)));
    printf("  Global minimum distance: %.3f m (safety: %.1f m)\n", min_dist_overall, d_safe);
    printf("  Initialization status: %s\n\n", (total_violations == 0) ? "FEASIBLE ✓" : "INFEASIBLE ✗");
    
    // Print initialized trajectories
    printf("Initialized trajectories (positions only):\n");
    printf("  k  |");
    for (int agent = 0; agent < NUM_AGENTS; ++agent) {
        printf("  Agent %d (x, y)  |", agent);
    }
    printf("\n");
    printf("-----|");
    for (int agent = 0; agent < NUM_AGENTS; ++agent) {
        printf("------------------|");
    }
    printf("\n");
    
    for (int k = 0; k <= N; k += 5) {  // Print every 5th step
        printf(" %2d  |", k);
        for (int agent = 0; agent < NUM_AGENTS; ++agent) {
            double x = x_traj[k*nx_total + agent*nx + 0];
            double y = x_traj[k*nx_total + agent*nx + 1];
            printf(" (%5.2f, %5.2f) |", x, y);
        }
        printf("\n");
    }
    printf("\n");
    
    // Check initial distances (legacy output)
    printf("Initial distances between agent pairs:\n");
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
    
    // Zero controls
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
            nnbx[k] = NUM_AGENTS * 2;  // Velocity bounds
        }
        
        nnbu[k] = (k < N) ? NUM_AGENTS * 2 : 0;  // Acceleration bounds
        nng[k] = (k > 0) ? NUM_PAIRS : 0;  // Collision constraints (enabled)
        nns[k] = 0;  // No slack variables (hard constraints)
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
    
    // Velocity bounds
    int* idxbx_vel = new int[NUM_AGENTS * 2];
    double* lbx_vel = new double[NUM_AGENTS * 2];
    double* ubx_vel = new double[NUM_AGENTS * 2];
    double* lbx_vel_mask = new double[NUM_AGENTS * 2];
    double* ubx_vel_mask = new double[NUM_AGENTS * 2];
    
    for (int agent = 0; agent < NUM_AGENTS; ++agent) {
        idxbx_vel[agent*2 + 0] = agent*nx + 2;
        idxbx_vel[agent*2 + 1] = agent*nx + 3;
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
            
            memset(hC[k], 0.0, NUM_PAIRS * nx_total * sizeof(double));
            memset(hD[k], 0.0, NUM_PAIRS * nu_total * sizeof(double));
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
        
        // Selective constraint activation: only enforce if within activation radius
        // Use larger radius for tighter safety constraints
        max_constraint_violation = 0.0;
        int num_active_constraints = 0;
        const double activation_radius = 1.2 * d_safe;  // Increased from 3.0 for d_safe=0.8
        
        for (int k = 1; k <= N; ++k) {
            for (int p = 0; p < NUM_PAIRS; ++p) {
                int i = collision_pairs[p][0];
                int j = collision_pairs[p][1];
                
                double x0 = x_traj[k*nx_total + i*nx + 0];
                double y0 = x_traj[k*nx_total + i*nx + 1];
                double x1 = x_traj[k*nx_total + j*nx + 0];
                double y1 = x_traj[k*nx_total + j*nx + 1];
                
                double dx = x0 - x1;
                double dy = y0 - y1;
                double dist = sqrt(dx*dx + dy*dy);
                
                // Constraint violation
                double violation = d_safe - dist;
                if (violation > max_constraint_violation) {
                    max_constraint_violation = violation;
                }
                
                // Selective constraint activation: only enforce if within activation radius
                if (dist <= activation_radius) {
                    // Linearization: n' * (p0 - p1) >= d_safe
                    // where n = (p0 - p1) / ||p0 - p1|| is the normal vector
                    num_active_constraints++;
                    double nx_c = dx / dist;
                    double ny_c = dy / dist;
                    
                    // HPIPM constraint format: C[constraint_idx, state_idx] = C[constraint_idx * nx_total + state_idx]
                    // Constraint: nx_c*(x0-x1) + ny_c*(y0-y1) >= d_safe
                    hC[k][p*nx_total + i*nx + 0] = nx_c;   // ∂/∂x0
                    hC[k][p*nx_total + i*nx + 1] = ny_c;   // ∂/∂y0
                    hC[k][p*nx_total + j*nx + 0] = -nx_c;  // ∂/∂x1
                    hC[k][p*nx_total + j*nx + 1] = -ny_c;  // ∂/∂y1
                    
                    hlg[k][p] = d_safe;
                    hug[k][p] = 1e10;
                } else {
                    // Constraint inactive: set to always satisfied (unbounded)
                    memset(&hC[k][p*nx_total], 0, nx_total * sizeof(double));
                    hlg[k][p] = -1e10;  // Always satisfied
                    hug[k][p] = 1e10;
                }
            }
        }
        
        printf("  Active collision constraints: %d / %d\n", num_active_constraints, NUM_PAIRS * N);
        printf("  Max constraint violation: %.4f m\n", max_constraint_violation);
        
        // DEBUG: Log detailed constraint information
        // printf("  [DEBUG] Constraint details:\n");
        // int active_count = 0;
        // for (int p = 0; p < NUM_PAIRS; ++p) {
        //     for (int k = 1; k <= N; ++k) {
        //         int i = collision_pairs[p][0];
        //         int j = collision_pairs[p][1];
        //         double x0 = x_traj[k*nx_total + i*nx + 0];
        //         double y0 = x_traj[k*nx_total + i*nx + 1];
        //         double x1 = x_traj[k*nx_total + j*nx + 0];
        //         double y1 = x_traj[k*nx_total + j*nx + 1];
        //         double dist = sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
                
        //         if (dist <= activation_radius) {
        //             active_count++;
        //             printf("    k=%2d, pair %d-%d: dist=%.3f m, active ✓\n", k, i, j, dist);
        //         } else {
        //             printf("    k=%2d, pair %d-%d: dist=%.3f m, inactive (>%.3f)\n", k, i, j, dist, activation_radius);
        //         }
        //     }
        // }
        // printf("  [DEBUG] Total active constraints: %d\n", active_count);
        
        // Setup QP arrays
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
        double** hC_arr = new double*[N+1];
        double** hD_arr = new double*[N+1];
        double** hlg_arr = new double*[N+1];
        double** hug_arr = new double*[N+1];
        
        double* b_zero = new double[nx_total];
        memset(b_zero, 0, nx_total * sizeof(double));
        
        double* b_initial = new double[nx_total];
        for (int i = 0; i < nx_total; ++i) {
            // This must match the values you put in lbx0/ubx0
            b_initial[i] = x0_init[i]; 
        }
        for (int k = 0; k < N; ++k) {
            hA[k] = A;
            hB[k] = B;
            hb[k] = b_zero;
            hQ[k] = Q;
            hS[k] = S;
            hR[k] = R;
            hq[k] = &q_ref[k*nx_total];
            hr[k] = &r_ref[k*nu_total];
            hidxbu[k] = idxbu;
            hlbu[k] = lbu;
            hubu[k] = ubu;
            hlbu_mask[k] = lbu_mask;
            hubu_mask[k] = ubu_mask;
        }
        
        // Stage 0
        hQ[0] = Q;
        hq[0] = &q_ref[0];
        hidxbx[0] = idxbx0;
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
            hidxbx[k] = idxbx_vel;
            hlbx[k] = lbx_vel;
            hubx[k] = ubx_vel;
            hlbx_mask[k] = lbx_vel_mask;
            hubx_mask[k] = ubx_vel_mask;
            // Collision constraints (enabled)
            hC_arr[k] = hC[k];
            hD_arr[k] = hD[k];
            hlg_arr[k] = hlg[k];
            hug_arr[k] = hug[k];
        }
        
        // Terminal stage N
        hQ[N] = Q;
        hq[N] = &q_ref[N*nx_total];
        hidxbx[N] = idxbx_vel;
        hlbx[N] = lbx_vel;
        hubx[N] = ubx_vel;
        hlbx_mask[N] = lbx_vel_mask;
        hubx_mask[N] = ubx_vel_mask;
        // Collision constraints (enabled)
        hC_arr[N] = hC[N];
        hD_arr[N] = hD[N];
        hlg_arr[N] = hlg[N];
        hug_arr[N] = hug[N];
        
        // Create QP structure
        hpipm_size_t qp_size = d_ocp_qp_memsize(&dim);
        void *qp_mem = malloc(qp_size);
        struct d_ocp_qp qp;
        d_ocp_qp_create(&dim, &qp, qp_mem);
        
        // Set QP data using individual setters
        // Dynamics
        for (int k = 0; k < N; ++k) {
            d_ocp_qp_set_A(k, hA[k], &qp);
            d_ocp_qp_set_B(k, hB[k], &qp);
            d_ocp_qp_set_b(k, hb[k], &qp);
        }
        
        // Cost
        for (int k = 0; k <= N; ++k) {
            d_ocp_qp_set_Q(k, hQ[k], &qp);
            d_ocp_qp_set_q(k, hq[k], &qp);
            if (k < N) {
                d_ocp_qp_set_S(k, hS[k], &qp);
                d_ocp_qp_set_R(k, hR[k], &qp);
                d_ocp_qp_set_r(k, hr[k], &qp);
            }
        }
        
        // State bounds
        for (int k = 0; k <= N; ++k) {
            d_ocp_qp_set_idxbx(k, hidxbx[k], &qp);
            d_ocp_qp_set_lbx(k, hlbx[k], &qp);
            d_ocp_qp_set_ubx(k, hubx[k], &qp);
        }
        
        // Control bounds
        for (int k = 0; k < N; ++k) {
            d_ocp_qp_set_idxbu(k, hidxbu[k], &qp);
            d_ocp_qp_set_lbu(k, hlbu[k], &qp);
            d_ocp_qp_set_ubu(k, hubu[k], &qp);
        }
        
        // Collision constraints (enabled)
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
        
        // Setup IPM solver (matching 2-agent settings)
        hpipm_size_t ipm_arg_size = d_ocp_qp_ipm_arg_memsize(&dim);
        void *ipm_arg_mem = malloc(ipm_arg_size);
        struct d_ocp_qp_ipm_arg arg;
        d_ocp_qp_ipm_arg_create(&dim, &arg, ipm_arg_mem);
        
        enum hpipm_mode mode = ROBUST;
        d_ocp_qp_ipm_arg_set_default(mode, &arg);
        
        // Relax tolerances since residuals are already very small
        double tol = 1e-2;  // Relaxed from 1e-6 to 1e-5
        d_ocp_qp_ipm_arg_set_tol_stat(&tol, &arg);
        d_ocp_qp_ipm_arg_set_tol_eq(&tol, &arg);
        d_ocp_qp_ipm_arg_set_tol_ineq(&tol, &arg);
        d_ocp_qp_ipm_arg_set_tol_comp(&tol, &arg);
        
        int iter_max = 500;
        d_ocp_qp_ipm_arg_set_iter_max(&iter_max, &arg);
        
        hpipm_size_t ipm_size = d_ocp_qp_ipm_ws_memsize(&dim, &arg);
        void *ipm_mem = malloc(ipm_size);
        struct d_ocp_qp_ipm_ws workspace;
        d_ocp_qp_ipm_ws_create(&dim, &arg, &workspace, ipm_mem);
        
        // Solve QP
        int hpipm_status;
        d_ocp_qp_dim_codegen(data_path, (char*)"w", &dim);
        d_ocp_qp_codegen(data_path, (char*)"a", &dim, &qp);
        d_ocp_qp_ipm_arg_codegen(data_path, (char*)"a", &dim, &arg);
        d_ocp_qp_codegen_matlab(data_path_matlab, (char*)"w", &dim, &qp);
        
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
        
        // // DEBUG: Check if QP solution satisfies linearized constraints
        // printf("  [DEBUG] Checking QP constraint satisfaction:\n");
        // int qp_constraint_violations = 0;
        // for (int p = 0; p < NUM_PAIRS; ++p) {
        //     for (int k = 1; k <= N; ++k) {
        //         int i = collision_pairs[p][0];
        //         int j = collision_pairs[p][1];
                
        //         // Get solution state vector for this stage
        //         double* x_sol_stage = new double[nx_total];
        //         d_ocp_qp_sol_get_x(k, &qp_sol, x_sol_stage);
                
        //         // Extract agent positions from state vector
        //         double x0_sol = x_sol_stage[i*nx + 0];  // Agent i x position
        //         double y0_sol = x_sol_stage[i*nx + 1];  // Agent i y position
        //         double x1_sol = x_sol_stage[j*nx + 0];  // Agent j x position
        //         double y1_sol = x_sol_stage[j*nx + 1];  // Agent j y position
                
        //         delete[] x_sol_stage;
                
        //         double dist_sol = sqrt((x0_sol-x1_sol)*(x0_sol-x1_sol) + (y0_sol-y1_sol)*(y0_sol-y1_sol));
                
        //         // Get distance from current trajectory (before QP solve)
        //         double x0_curr = x_traj[k*nx_total + i*nx + 0];
        //         double y0_curr = x_traj[k*nx_total + i*nx + 1];
        //         double x1_curr = x_traj[k*nx_total + j*nx + 0];
        //         double y1_curr = x_traj[k*nx_total + j*nx + 1];
        //         double dist_curr = sqrt((x0_curr-x1_curr)*(x0_curr-x1_curr) + (y0_curr-y1_curr)*(y0_curr-y1_curr));
                
        //         // Check linearized constraint if active
        //         if (dist_curr <= activation_radius) {
        //             double constraint_value = (x0_sol - x1_sol) * hC[k][p*nx_total + i*nx + 0] +
        //                                     (y0_sol - y1_sol) * hC[k][p*nx_total + i*nx + 1];
                    
        //             double lower_bound = hlg[k][p];
        //             double violation = lower_bound - constraint_value;
                    
        //             printf("    k=%2d, pair %d-%d: constraint_value=%.3f, lower_bound=%.3f, violation=%.2e\n",
        //                    k, i, j, constraint_value, lower_bound, violation);
                    
        //             if (violation > 1e-6) {  // Allow small numerical tolerance
        //                 qp_constraint_violations++;
        //                 printf("      → LINEARIZED CONSTRAINT VIOLATED!\n");
        //             } else {
        //                 printf("      → Linearized constraint satisfied ✓\n");
        //             }
        //         }
                
        //         if (dist_sol < d_safe) {
        //             printf("    k=%2d, pair %d-%d: QP solution dist=%.3f m < %.1f m safety!\n", 
        //                    k, i, j, dist_sol, d_safe);
        //         }
        //     }
        // }
        // printf("  [DEBUG] QP constraint violations: %d\n", qp_constraint_violations);
        
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
        break;
        
        
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
        
        // Print trajectory for this SQP iteration
        printf("\n  Trajectory after SQP iteration %d:\n", sqp_iter);
        printf("    k  |");
        for (int agent = 0; agent < NUM_AGENTS; ++agent) {
            printf("  Agent %d (x, y)  |", agent);
        }
        printf("\n");
        printf("  -----|");
        for (int agent = 0; agent < NUM_AGENTS; ++agent) {
            printf("------------------|");
        }
        printf("\n");
        
        for (int k = 0; k <= N; k += 1) {  // Print every 5th step
            printf("   %2d  |", k);
            for (int agent = 0; agent < NUM_AGENTS; ++agent) {
                double x = x_traj[k*nx_total + agent*nx + 0];
                double y = x_traj[k*nx_total + agent*nx + 1];
                printf(" (%5.2f, %5.2f) |", x, y);
            }
            printf("\n");
        }
        printf("\n");
        
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
     * Final full trajectory printing (states and controls)
     ************************************************/
    printf("Final trajectories (states and controls) after SQP:\n\n");

    for (int agent = 0; agent < NUM_AGENTS; ++agent) {
        // States: x, y, vx, vy for k = 0..N
        printf("Agent %d - States [k, x, y, vx, vy]:\n", agent);
        for (int k = 0; k <= N; ++k) {
            double x = x_traj[k*nx_total + agent*nx + 0];
            double y = x_traj[k*nx_total + agent*nx + 1];
            double vx = x_traj[k*nx_total + agent*nx + 2];
            double vy = x_traj[k*nx_total + agent*nx + 3];
            printf("  %2d: %9.4f %9.4f %9.4f %9.4f\n", k, x, y, vx, vy);
        }
        printf("\n");

        // Controls: ax, ay for k = 0..N-1
        printf("Agent %d - Controls [k, ax, ay]:\n", agent);
        for (int k = 0; k < N; ++k) {
            double ax = u_traj[k*nu_total + agent*nu + 0];
            double ay = u_traj[k*nu_total + agent*nu + 1];
            printf("  %2d: %9.4f %9.4f\n", k, ax, ay);
        }
        printf("\n");
    }

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
        double x_ref_final = x_ref_all[N*nx_total + agent*nx + 0];
        double y_ref_final = x_ref_all[N*nx_total + agent*nx + 1];
        
        tracking_errors[agent] = computeDistance(x_final, y_final, x_ref_final, y_ref_final);
        sum_errors += tracking_errors[agent];
        printf("  Agent %d: %.3f m\n", agent, tracking_errors[agent]);
    }
    
    double avg_tracking_error = sum_errors / NUM_AGENTS;
    printf("  Average: %.3f m\n", avg_tracking_error);
    
    bool success = converged && (global_min_dist >= d_safe);
    printf("\n%s\n", success ? "SUCCESS ✓" : "FAILED ✗");
    printf("================================================================================\n");
    
    // Cleanup
    delete[] A;
    delete[] B;
    delete[] Q;
    delete[] S;
    delete[] R;
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
    delete[] nnx;
    delete[] nnu;
    delete[] nnbx;
    delete[] nnbu;
    delete[] nng;
    delete[] nns;
    
    for (int k = 1; k <= N; ++k) {
        delete[] hC[k];
        delete[] hD[k];
        delete[] hlg[k];
        delete[] hug[k];
    }
    
    free(dim_mem);
    
    return success ? 0 : 1;
}
