/*
 * Test: 2-Agent with ρ=25 using OSQP
 *
 * Comparative example using OSQP to solve the decoupled MPC problem
 * (without ADMM coupling, for simplicity)
 */

#include <stdlib.h>
#include <osqp/osqp.h>
#include <qpOASES/TurboADMM.hpp>
#include <cstdio>
#include <cmath>
#include <chrono>
#include <vector>
#include <algorithm>

double getTime() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double, std::milli>(duration).count();
}

USING_NAMESPACE_QPOASES

struct Triplet {
    int row, col;
    double val;
};

int main()
{
    double total_start = getTime();
    
    printf("================================================================================\n");
    printf("                    2-AGENT CENTRALIZED MPC WITH OSQP\n");
    printf("================================================================================\n\n");
    
    int N = 20;
    int nx = 4;
    int nu = 2;
    real_t dt = 0.2;
    real_t v_max = 10.0;
    real_t d_safe = 2.0;
    
    PointMass agent0(N, dt, 10.0, v_max);
    PointMass agent1(N, dt, 10.0, v_max);
    
    AgentData* agents = new AgentData[2];
    agents[0] = agent0;
    agents[1] = agent1;
    
    real_t Q_pos = 2.0;   // Position tracking (matching TurboADMM)
    real_t Q_vel = 0.01;
    real_t R_ctrl = 0.1;
    
    for (int i = 0; i < 2; ++i) {
        agents[i].Q[0*nx + 0] = Q_pos;  // Pure tracking cost
        agents[i].Q[1*nx + 1] = Q_pos;  // Pure tracking cost
        agents[i].Q[2*nx + 2] = Q_vel;
        agents[i].Q[3*nx + 3] = Q_vel;
        
        agents[i].R[0*nu + 0] = R_ctrl;
        agents[i].R[1*nu + 1] = R_ctrl;
        
        agents[i].extractDiagonals();
    }
    
    // Set reference trajectories
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[0].x_ref[k*nx + 0] = 0.0 + alpha * 15.0;
        agents[0].x_ref[k*nx + 1] = 4.8;
        agents[0].x_ref[k*nx + 2] = 0.0;
        agents[0].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[0].u_ref[k*nu + 0] = 0.0;
        agents[0].u_ref[k*nu + 1] = 0.0;
    }
    
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[1].x_ref[k*nx + 0] = 15.0 + alpha * (-15.0);
        agents[1].x_ref[k*nx + 1] = 5.3;
        agents[1].x_ref[k*nx + 2] = 0.0;
        agents[1].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[1].u_ref[k*nu + 0] = 0.0;
        agents[1].u_ref[k*nu + 1] = 0.0;
    }
    
    printf("Agent 0: (0, 4.8) -> (15, 4.8)\n");
    printf("Agent 1: (15, 5.3) -> (0, 5.3)\n");
    printf("Time horizon: N=%d, dt=%.1f s, total time=%.1f s\n", N, dt, N*dt);
    printf("Velocity bounds: -10.0 m/s <= v <= 10.0 m/s\n");
    printf("Safety distance: %.1f m\n\n", d_safe);
    
    // Compute minimum reference distance for debugging
    real_t min_ref_dist = 1e10;
    for (int k = 0; k <= N; ++k) {
        real_t x0 = agents[0].x_ref[k*nx + 0];
        real_t y0 = agents[0].x_ref[k*nx + 1];
        real_t x1 = agents[1].x_ref[k*nx + 0];
        real_t y1 = agents[1].x_ref[k*nx + 1];
        real_t dist = sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
        if (dist < min_ref_dist) min_ref_dist = dist;
    }
    printf("Minimum reference distance: %.3f m\n\n", min_ref_dist);
    
    // Build centralized QP
    int nV_agent = (N+1)*nx + N*nu;  // 124
    int nV = 2 * nV_agent;  // 248
    int n_dyn = 2 * N * nx;  // 160
    int n_bounds = 2 * nV;  // 496
    int n_coupling = N + 1;  // 21
    int m = n_dyn + n_bounds + n_coupling;  // 677
    
    // Build P (diagonal, block for each agent)
    std::vector<OSQPFloat> P_x;
    std::vector<OSQPInt> P_i, P_p;
    std::vector<OSQPFloat> P_diag(nV, 0.0);
    P_p.push_back(0);
    for (int i = 0; i < nV; ++i) {
        OSQPFloat val = 0.0;
        if (i < nV_agent) {
            // Agent 0
            int local_i = i;
            int k_val = local_i / (nx + nu); (void)k_val;  // suppress unused warning
            int rem = local_i % (nx + nu);
            if (rem < nx) {
                val = agents[0].Q[rem*nx + rem];
            } else {
                val = agents[0].R[(rem - nx)*nu + (rem - nx)];
            }
        } else {
            // Agent 1
            int local_i = i - nV_agent;
            int k_val = local_i / (nx + nu); (void)k_val;  // suppress unused warning
            int rem = local_i % (nx + nu);
            if (rem < nx) {
                val = agents[1].Q[rem*nx + rem];
            } else {
                val = agents[1].R[(rem - nx)*nu + (rem - nx)];
            }
        }
        P_diag[i] = val;
        if (fabs(val) > 1e-12) {
            P_x.push_back(val);
            P_i.push_back(i);
        }
        P_p.push_back(P_x.size());
    }
    
    // Build q for reference tracking: min 1/2 x'Px + q'x
    // For tracking cost 1/2(z-z_ref)'P(z-z_ref) = 1/2 z'Pz - z'P*z_ref + const
    // So q = -P*z_ref (using diagonal P stored in P_diag)
    std::vector<OSQPFloat> q(nV, 0.0);
    for (int i = 0; i < nV; ++i) {
        if (i < nV_agent) {
            int k = i / (nx + nu);
            int rem = i % (nx + nu);
            if (rem < nx) {
                q[i] = -P_diag[i] * agents[0].x_ref[k*nx + rem];
            } else {
                if (k < N) q[i] = -P_diag[i] * agents[0].u_ref[k*nu + (rem - nx)];
            }
        } else {
            int local_i = i - nV_agent;
            int k = local_i / (nx + nu);
            int rem = local_i % (nx + nu);
            if (rem < nx) {
                q[i] = -P_diag[i] * agents[1].x_ref[k*nx + rem];
            } else {
                if (k < N) q[i] = -P_diag[i] * agents[1].u_ref[k*nu + (rem - nx)];
            }
        }
    }
    
    // Build A (dynamics, bounds, coupling)
    std::vector<Triplet> A_triplets;
    
    // Dynamics for agent 0
    for (int k = 0; k < N; ++k) {
        int idx_xk = k * (nx + nu);
        int idx_uk = idx_xk + nx;
        int idx_xkp1 = (k+1) * (nx + nu);
        for (int j = 0; j < nx; ++j) {
            int row = k * nx + j;
            // x_{k+1,j}
            A_triplets.push_back({row, idx_xkp1 + j, 1.0});
            // -A[j,:] * x_k
            for (int l = 0; l < nx; ++l) {
                OSQPFloat val = -agents[0].A[j*nx + l];
                if (fabs(val) > 1e-12) A_triplets.push_back({row, idx_xk + l, val});
            }
            // -B[j,:] * u_k
            for (int l = 0; l < nu; ++l) {
                OSQPFloat val = -agents[0].B[j*nu + l];
                if (fabs(val) > 1e-12) A_triplets.push_back({row, idx_uk + l, val});
            }
        }
    }
    
    // Dynamics for agent 1
    for (int k = 0; k < N; ++k) {
        int idx_xk = nV_agent + k * (nx + nu);
        int idx_uk = idx_xk + nx;
        int idx_xkp1 = nV_agent + (k+1) * (nx + nu);
        for (int j = 0; j < nx; ++j) {
            int row = n_dyn / 2 + k * nx + j;
            // x_{k+1,j}
            A_triplets.push_back({row, idx_xkp1 + j, 1.0});
            // -A[j,:] * x_k
            for (int l = 0; l < nx; ++l) {
                OSQPFloat val = -agents[1].A[j*nx + l];
                if (fabs(val) > 1e-12) A_triplets.push_back({row, idx_xk + l, val});
            }
            // -B[j,:] * u_k
            for (int l = 0; l < nu; ++l) {
                OSQPFloat val = -agents[1].B[j*nu + l];
                if (fabs(val) > 1e-12) A_triplets.push_back({row, idx_uk + l, val});
            }
        }
    }
    
    // Bounds
    for (int i = 0; i < nV; ++i) {
        int row_ub = n_dyn + i;
        A_triplets.push_back({row_ub, i, 1.0});  // x_i <= ub
        
        int row_lb = n_dyn + nV + i;
        A_triplets.push_back({row_lb, i, 1.0});  // x_i >= lb (represented as x_i with lower bound)
    }
    
    // Coupling
    for (int k = 0; k <= N; ++k) {
        int row = n_dyn + n_bounds + k;
        OSQPFloat x0_ref = agents[0].x_ref[k*nx + 0];
        OSQPFloat y0_ref = agents[0].x_ref[k*nx + 1];
        OSQPFloat x1_ref = agents[1].x_ref[k*nx + 0];
        OSQPFloat y1_ref = agents[1].x_ref[k*nx + 1];
        OSQPFloat dx = x0_ref - x1_ref;
        OSQPFloat dy = y0_ref - y1_ref;
        OSQPFloat dist = sqrt(dx*dx + dy*dy);
        if (dist > 0 && dist < d_safe) {
            OSQPFloat nx_c = dx / dist;
            OSQPFloat ny_c = dy / dist;
            // n · (p0 - p1) >= d
            // nx*(x0 - x1) + ny*(y0 - y1) >= d
            int idx_x0 = k * (nx + nu);
            int idx_y0 = idx_x0 + 1;
            int idx_x1 = nV_agent + k * (nx + nu);
            int idx_y1 = idx_x1 + 1;
            A_triplets.push_back({row, idx_x0, nx_c});
            A_triplets.push_back({row, idx_y0, ny_c});
            A_triplets.push_back({row, idx_x1, -nx_c});
            A_triplets.push_back({row, idx_y1, -ny_c});
        }
    }
    
    // Sort A triplets
    std::sort(A_triplets.begin(), A_triplets.end(), [](const Triplet& a, const Triplet& b) {
        if (a.col != b.col) return a.col < b.col;
        return a.row < b.row;
    });
    
    // Build A CSC
    std::vector<OSQPFloat> A_x(A_triplets.size());
    std::vector<OSQPInt> A_i(A_triplets.size()), A_p(nV + 1, 0);
    int idx = 0;
    int current_col = 0;
    for (auto& t : A_triplets) {
        while (current_col < t.col) {
            A_p[current_col + 1] = idx;
            current_col++;
        }
        A_i[idx] = t.row;
        A_x[idx] = t.val;
        idx++;
        A_p[current_col + 1] = idx;
    }
    while (current_col < nV) {
        A_p[current_col + 1] = idx;
        current_col++;
    }
    
    // l and u
    std::vector<OSQPFloat> l(m, 0.0);
    std::vector<OSQPFloat> u(m, 0.0);
    // Dynamics: equality
    // Bounds
    for (int i = 0; i < nV; ++i) {
        int row_ub = n_dyn + i;
        l[row_ub] = -OSQP_INFTY;
        u[row_ub] = (i < nV_agent) ? agents[0].ub[i % nV_agent] : agents[1].ub[(i - nV_agent) % nV_agent];
        
        int row_lb = n_dyn + nV + i;
        l[row_lb] = (i < nV_agent) ? agents[0].lb[i % nV_agent] : agents[1].lb[(i - nV_agent) % nV_agent];
        u[row_lb] = OSQP_INFTY;
        
        // Fix initial positions to initial state
        if (i < nx) {
            // Agent 0 initial position
            u[row_ub] = agents[0].x_ref[0*nx + i];
            l[row_lb] = agents[0].x_ref[0*nx + i];
        } else if (i >= nV_agent && i < nV_agent + nx) {
            // Agent 1 initial position
            int local_i = i - nV_agent;
            u[row_ub] = agents[1].x_ref[0*nx + local_i];
            l[row_lb] = agents[1].x_ref[0*nx + local_i];
        }
    }
    // Initialize coupling bounds to inactive (will be set dynamically in SQP loop)
    for (int k = 0; k <= N; ++k) {
        int row = n_dyn + n_bounds + k;
        l[row] = -OSQP_INFTY;
        u[row] = OSQP_INFTY;
    }
    
    // ====== DIAGNOSTIC LOGGING ======
    printf("\n=== QP PROBLEM DIAGNOSTICS ===\n");
    printf("Problem size: nV=%d, m=%d\n", nV, m);
    printf("  Variables: %d states + %d controls per agent, 2 agents\n", (N+1)*nx, N*nu);
    printf("  Constraints: %d dynamics + %d bounds + %d coupling\n", n_dyn, n_bounds, n_coupling);
    
    // Check initial state bounds
    printf("\nInitial state bounds (Agent 0, k=0):\n");
    for (int j = 0; j < nx; ++j) {
        int row_ub = n_dyn + j;
        int row_lb = n_dyn + nV + j;
        printf("  State[%d]: l[%d]=%.4f (should be -%.4f), u[%d]=%.4f (should be %.4f)\n",
               j, row_lb, l[row_lb], agents[0].x_ref[j], row_ub, u[row_ub], agents[0].x_ref[j]);
    }
    printf("\nInitial state bounds (Agent 1, k=0):\n");
    for (int j = 0; j < nx; ++j) {
        int row_ub = n_dyn + nV_agent + j;
        int row_lb = n_dyn + nV + nV_agent + j;
        printf("  State[%d]: l[%d]=%.4f (should be -%.4f), u[%d]=%.4f (should be %.4f)\n",
               j, row_lb, l[row_lb], agents[1].x_ref[j], row_ub, u[row_ub], agents[1].x_ref[j]);
    }
    
    // Check cost gradient
    printf("\nCost gradient q (first 10 elements):\n");
    for (int i = 0; i < 10 && i < nV; ++i) {
        printf("  q[%d] = %.6f\n", i, q[i]);
    }
    
    // Check Hessian diagonal
    printf("\nHessian P diagonal (first 10 elements):\n");
    for (int i = 0; i < 10 && i < (int)P_x.size(); ++i) {
        printf("  P[%d,%d] = %.6f\n", P_i[i], P_i[i], P_x[i]);
    }
    
    // Print ALL constraint bounds
    printf("\n=== CONSTRAINT BOUNDS VERIFICATION ===\n");
    printf("Dynamics constraints (rows 0-%d): should all be l=u=0\n", n_dyn-1);
    for (int i = 0; i < n_dyn && i < 20; ++i) {
        printf("  Row %d: l=%.6f, u=%.6f\n", i, l[i], u[i]);
    }
    if (n_dyn > 20) printf("  ... (%d more dynamics rows)\n", n_dyn - 20);
    
    printf("\nBound constraints (rows %d-%d):\n", n_dyn, n_dyn + n_bounds - 1);
    printf("First 20 bound constraints:\n");
    for (int i = n_dyn; i < n_dyn + n_bounds && i < n_dyn + 20; ++i) {
        int var_idx = i - n_dyn;
        const char* type = (var_idx < nV) ? "upper" : "lower";
        printf("  Row %d (%s bound var %d): l=%.6f, u=%.6f\n", 
               i, type, var_idx % nV, l[i], u[i]);
    }
    
    printf("\nCoupling constraints (rows %d-%d): should be l=2.0, u=INF\n", 
           n_dyn + n_bounds, n_dyn + n_bounds + n_coupling - 1);
    for (int i = n_dyn + n_bounds; i < m; ++i) {
        printf("  Row %d: l=%.6f, u=%.6f\n", i, l[i], u[i]);
    }
    printf("\n");
    
    // Create matrices
    OSQPCscMatrix* P = OSQPCscMatrix_new(nV, nV, P_x.size(), P_x.data(), P_i.data(), P_p.data());
    OSQPCscMatrix* A = OSQPCscMatrix_new(m, nV, A_x.size(), A_x.data(), A_i.data(), A_p.data());
    
    // Settings
    OSQPSettings* settings = OSQPSettings_new();
    settings->verbose = 1;
    settings->max_iter = 20000;
    // settings->eps_abs = 1e-3;
    // settings->eps_rel = 1e-3;
    // settings->eps_prim_inf = 1e-3;
    // settings->eps_dual_inf = 1e-3;
    
    // Solver
    OSQPSolver* solver;
    
    // SQP loop
    const int max_sqp_iter = 300;
    const real_t constraint_tol = 1e-2;  // 1mm tolerance for collision constraints
    const real_t objective_tol = 2e-3;  // Relative objective change tolerance for convergence
    
    std::vector<real_t> current_trajectory(nV, 0.0);
    std::vector<real_t> previous_trajectory(nV, 0.0);
    real_t previous_objective = 1e30;
    
    // Store dynamics and bounds triplets (constant across SQP iterations)
    std::vector<Triplet> A_triplets_fixed;
    for (const auto& t : A_triplets) {
        if (t.row < n_dyn + n_bounds) {  // Dynamics and bounds only
            A_triplets_fixed.push_back(t);
        }
    }
    
    // Initialize with reference trajectory
    for (int i = 0; i < nV; ++i) {
        if (i < nV_agent) {
            int k = i / (nx + nu);
            int rem = i % (nx + nu);
            if (rem < nx) {
                current_trajectory[i] = agents[0].x_ref[k*nx + rem];
            } else {
                if (k < N) current_trajectory[i] = agents[0].u_ref[k*nu + (rem - nx)];
            }
        } else {
            int local_i = i - nV_agent;
            int k = local_i / (nx + nu);
            int rem = local_i % (nx + nu);
            if (rem < nx) {
                current_trajectory[i] = agents[1].x_ref[k*nx + rem];
            } else {
                if (k < N) current_trajectory[i] = agents[1].u_ref[k*nu + (rem - nx)];
            }
        }
    }
    
    bool converged = false;
    int final_iter = 0;
    
    for (int sqp_iter = 0; sqp_iter < max_sqp_iter; ++sqp_iter) {
        printf("\n=== SQP Iteration %d ===\n", sqp_iter);
        
        // Save previous trajectory for convergence check
        previous_trajectory = current_trajectory;
        
        // Rebuild A_triplets: start with fixed part (dynamics + bounds)
        A_triplets = A_triplets_fixed;
        
        // Add coupling constraints based on current_trajectory
        int num_active_collisions = 0;
        printf("\nCoupling constraint activation (SQP iter %d):\n", sqp_iter);
        for (int k = 0; k <= N; ++k) {
            int row = n_dyn + n_bounds + k;
            int idx = k * (nx + nu);
            real_t p0x = current_trajectory[idx];
            real_t p0y = current_trajectory[idx + 1];
            real_t p1x = current_trajectory[nV_agent + idx];
            real_t p1y = current_trajectory[nV_agent + idx + 1];
            real_t dx = p0x - p1x;
            real_t dy = p0y - p1y;
            real_t dist = sqrt(dx*dx + dy*dy);
            if (dist > 0 && dist < d_safe) {
                num_active_collisions++;
                // Agents are close - enforce collision avoidance
                real_t nx_c = dx / dist;
                real_t ny_c = dy / dist;
                // n · (p0 - p1) >= d_safe
                int idx_x0 = idx;
                int idx_y0 = idx + 1;
                int idx_x1 = nV_agent + idx;
                int idx_y1 = nV_agent + idx + 1;
                A_triplets.push_back({row, idx_x0, nx_c});
                A_triplets.push_back({row, idx_y0, ny_c});
                A_triplets.push_back({row, idx_x1, -nx_c});
                A_triplets.push_back({row, idx_y1, -ny_c});
                // Set bounds for active constraint
                l[row] = d_safe;
                u[row] = OSQP_INFTY;
                if (num_active_collisions <= 5) {
                    printf("  k=%d: p0=(%.2f,%.2f) p1=(%.2f,%.2f) dist=%.3f < %.1f ACTIVE\n",
                           k, p0x, p0y, p1x, p1y, dist, d_safe);
                }
            } else {
                // Agents are far - constraint inactive
                l[row] = -OSQP_INFTY;
                u[row] = OSQP_INFTY;
            }
        }
        printf("  Total active collision constraints: %d/%d\n", num_active_collisions, N+1);
        
        // Sort A triplets
        std::sort(A_triplets.begin(), A_triplets.end(), [](const Triplet& a, const Triplet& b) {
            if (a.col != b.col) return a.col < b.col;
            return a.row < b.row;
        });
        
        // Build A CSC
        std::vector<OSQPFloat> A_x(A_triplets.size());
        std::vector<OSQPInt> A_i(A_triplets.size()), A_p(nV + 1, 0);
        int idx_sort = 0;
        int current_col_sort = 0;
        for (auto& t : A_triplets) {
            while (current_col_sort < t.col) {
                A_p[current_col_sort + 1] = idx_sort;
                current_col_sort++;
            }
            A_i[idx_sort] = t.row;
            A_x[idx_sort] = t.val;
            idx_sort++;
            A_p[current_col_sort + 1] = idx_sort;
        }
        while (current_col_sort < nV) {
            A_p[current_col_sort + 1] = idx_sort;
            current_col_sort++;
        }
        
        // Create matrices
        OSQPCscMatrix* A_new = OSQPCscMatrix_new(m, nV, A_x.size(), A_x.data(), A_i.data(), A_p.data());
        
        // Solver
        OSQPSolver* solver;
        OSQPInt exitflag = osqp_setup(&solver, P, q.data(), A_new, l.data(), u.data(), m, nV, settings);
        
        if (exitflag != 0) {
            printf("OSQP setup failed\n");
            return 1;
        }
        
        // Set warm start if not first iteration
        if (sqp_iter > 0) {
            osqp_warm_start(solver, current_trajectory.data(), NULL);
        }
        
        exitflag = osqp_solve(solver);
        
        if (exitflag != 0) {
            printf("OSQP solve failed\n");
            return 1;
        }
        
        // Update current_trajectory with solution
        OSQPFloat* solution = solver->solution->x;
        for (int j = 0; j < nV; ++j) {
            current_trajectory[j] = solution[j];
        }
        
        printf("OSQP status: %s\n", solver->info->status);
        printf("OSQP iterations: %d\n", (int)solver->info->iter);
        printf("OSQP objective: %.6e\n", solver->info->obj_val);
        
        // Check initial state violations
        printf("Initial state check:\n");
        printf("  Agent 0 k=0: (%.4f, %.4f, %.4f, %.4f) vs bounds u=(%.4f, %.4f, %.4f, %.4f) l=(%.4f, %.4f, %.4f, %.4f)\n",
               solver->solution->x[0], solver->solution->x[1], solver->solution->x[2], solver->solution->x[3],
               u[n_dyn+0], u[n_dyn+1], u[n_dyn+2], u[n_dyn+3],
               l[n_dyn+nV+0], l[n_dyn+nV+1], l[n_dyn+nV+2], l[n_dyn+nV+3]);
        printf("  Agent 1 k=0: (%.4f, %.4f, %.4f, %.4f) vs bounds u=(%.4f, %.4f, %.4f, %.4f) l=(%.4f, %.4f, %.4f, %.4f)\n",
               solver->solution->x[nV_agent], solver->solution->x[nV_agent+1], solver->solution->x[nV_agent+2], solver->solution->x[nV_agent+3],
               u[n_dyn+nV_agent], u[n_dyn+nV_agent+1], u[n_dyn+nV_agent+2], u[n_dyn+nV_agent+3],
               l[n_dyn+nV+nV_agent], l[n_dyn+nV+nV_agent+1], l[n_dyn+nV+nV_agent+2], l[n_dyn+nV+nV_agent+3]);
        
        osqp_cleanup(solver);
        OSQPCscMatrix_free(A_new);
        
        // Clear coupling triplets for next iteration
        A_triplets.resize(n_dyn + 2 * nV);  // Keep dynamics and bounds
        
        // Compute constraint violations
        real_t max_constraint_viol = 0.0;
        real_t min_dist_iter = 1e10;
        for (int k = 0; k <= N; ++k) {
            int idx = k * (nx + nu);
            real_t x0 = current_trajectory[idx];
            real_t y0 = current_trajectory[idx + 1];
            real_t x1 = current_trajectory[nV_agent + idx];
            real_t y1 = current_trajectory[nV_agent + idx + 1];
            real_t dist = sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
            if (dist < min_dist_iter) min_dist_iter = dist;
            real_t viol = d_safe - dist;
            if (viol > max_constraint_viol) max_constraint_viol = viol;
        }
        printf("Min distance: %.4f m, Max constraint violation: %.4e m\n", min_dist_iter, max_constraint_viol);
        
        // Compute objective change
        real_t current_objective = solver->info->obj_val;
        real_t obj_change = fabs(current_objective - previous_objective);
        real_t rel_obj_change = (fabs(previous_objective) > 1e-10) ? 
                                 obj_change / fabs(previous_objective) : obj_change;
        printf("Objective: %.6e, Change: %.4e (rel: %.4e)\n", 
               current_objective, obj_change, rel_obj_change);
        
        // Check convergence
        final_iter = sqp_iter + 1;
        if (max_constraint_viol <= 0.0 && rel_obj_change < objective_tol) {
            printf("\n*** SQP CONVERGED: Constraints satisfied and objective converged ***\n");
            converged = true;
            previous_objective = current_objective;
            break;
        } else if (sqp_iter == max_sqp_iter - 1) {
            printf("\n*** SQP TERMINATED: Max iterations reached ***\n");
        }
        
        // Update previous objective for next iteration
        previous_objective = current_objective;
    }
    
    // Use final trajectory
    real_t* z0 = new real_t[nV_agent];
    real_t* z1 = new real_t[nV_agent];
    for (int j = 0; j < nV_agent; ++j) {
        z0[j] = current_trajectory[j];
        z1[j] = current_trajectory[nV_agent + j];
    }
    
    printf("\n================================================================================\n");
    printf("                         SQP SUMMARY\n");
    printf("================================================================================\n");
    printf("Total SQP iterations: %d\n", final_iter);
    printf("Converged: %s\n", converged ? "YES" : "NO (max iterations)");
    printf("\n");
    
    // Print terminal states
    int idx_N = N * (nx + nu);
    printf("Terminal States:\n");
    printf("Agent 0: (%.2f, %.2f, %.2f, %.2f)\n",
           z0[idx_N], z0[idx_N+1], z0[idx_N+2], z0[idx_N+3]);
    printf("Agent 1: (%.2f, %.2f, %.2f, %.2f)\n",
           z1[idx_N], z1[idx_N+1], z1[idx_N+2], z1[idx_N+3]);
    
    // Verify collision constraints
    real_t min_achieved_dist = 1e10;
    for (int k = 0; k <= N; ++k) {
        real_t x0 = z0[k*(nx+nu)];
        real_t y0 = z0[k*(nx+nu)+1];
        real_t x1 = z1[k*(nx+nu)];
        real_t y1 = z1[k*(nx+nu)+1];
        real_t dist = sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
        if (dist < min_achieved_dist) min_achieved_dist = dist;
    }
    printf("Minimum achieved distance: %.3f m (required: %.1f m)\n", min_achieved_dist, d_safe);
    if (min_achieved_dist >= d_safe) {
        printf("Collision constraints satisfied.\n");
    } else {
        printf("Collision constraints violated!\n");
    }
    
    // Print full trajectories
    printf("\nAgent 0 full trajectory:\n");
    for (int k = 0; k <= N; ++k) {
        int traj_idx = k * (nx + nu);
        printf("k=%d: x=(%.3f, %.3f, %.3f, %.3f)", k, z0[traj_idx], z0[traj_idx+1], z0[traj_idx+2], z0[traj_idx+3]);
        if (k < N) {
            printf(" u=(%.3f, %.3f)", z0[traj_idx+nx], z0[traj_idx+nx+1]);
        }
        printf("\n");
    }
    
    printf("\nAgent 1 full trajectory:\n");
    for (int k = 0; k <= N; ++k) {
        int traj_idx = k * (nx + nu);
        printf("k=%d: x=(%.3f, %.3f, %.3f, %.3f)", k, z1[traj_idx], z1[traj_idx+1], z1[traj_idx+2], z1[traj_idx+3]);
        if (k < N) {
            printf(" u=(%.3f, %.3f)", z1[traj_idx+nx], z1[traj_idx+nx+1]);
        }
        printf("\n");
    }
    
    // Cleanup
    OSQPCscMatrix_free(A);
    OSQPCscMatrix_free(P);
    OSQPSettings_free(settings);
    
    delete[] z0;
    delete[] z1;
    delete[] agents;
    
    double total_end = getTime();
    printf("Total execution time: %.3f ms\n", total_end - total_start);
    
    return 0;
}
