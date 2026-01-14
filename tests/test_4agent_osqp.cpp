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
    
    printf("================================================================================\n");
    printf("                    4-AGENT CENTRALIZED MPC WITH OSQP\n");
    printf("================================================================================\n\n");
    
    int N = 20;
    int nx = 4;
    int nu = 2;
    real_t dt = 0.2;
    real_t v_max = 10.0;
    real_t d_safe = 2.0;
    real_t d_safe_margin = 0.05;  // 5cm margin for OSQP numerical tolerance
    real_t d_safe_constraint = d_safe + d_safe_margin;  // Use 2.05m in constraints
    
    // Create 4 PointMass agents
    PointMass agent0(N, dt, 10.0, v_max);
    PointMass agent1(N, dt, 10.0, v_max);
    PointMass agent2(N, dt, 10.0, v_max);
    PointMass agent3(N, dt, 10.0, v_max);
    
    AgentData* agents = new AgentData[4];
    agents[0] = agent0;
    agents[1] = agent1;
    agents[2] = agent2;
    agents[3] = agent3;
    
    real_t Q_pos = 2.0;   // Position tracking (matching TurboADMM)
    real_t Q_vel = 0.01;
    real_t R_ctrl = 0.1;
    
    for (int i = 0; i < 4; ++i) {
        agents[i].Q[0*nx + 0] = Q_pos;  // Pure tracking cost
        agents[i].Q[1*nx + 1] = Q_pos;  // Pure tracking cost
        agents[i].Q[2*nx + 2] = Q_vel;
        agents[i].Q[3*nx + 3] = Q_vel;
        
        agents[i].R[0*nu + 0] = R_ctrl;
        agents[i].R[1*nu + 1] = R_ctrl;
        
        agents[i].extractDiagonals();
    }
    
    // Set reference trajectories
    // Agent 0: (0, 4.8) -> (15, 4.8) - right
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
    
    // Agent 1: (15, 5.3) -> (0, 5.3) - left
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
    
    // Agent 2: (7.25, 0) -> (7.25, 15) - up
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[2].x_ref[k*nx + 0] = 7.25;
        agents[2].x_ref[k*nx + 1] = 0.0 + alpha * 15.0;
        agents[2].x_ref[k*nx + 2] = 0.0;
        agents[2].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[2].u_ref[k*nu + 0] = 0.0;
        agents[2].u_ref[k*nu + 1] = 0.0;
    }
    
    // Agent 3: (7.75, 15) -> (7.75, 0) - down
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[3].x_ref[k*nx + 0] = 7.75;
        agents[3].x_ref[k*nx + 1] = 15.0 + alpha * (-15.0);
        agents[3].x_ref[k*nx + 2] = 0.0;
        agents[3].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[3].u_ref[k*nu + 0] = 0.0;
        agents[3].u_ref[k*nu + 1] = 0.0;
    }
    
    printf("Agent 0: (0, 4.8) -> (15, 4.8) [right]\n");
    printf("Agent 1: (15, 5.3) -> (0, 5.3) [left]\n");
    printf("Agent 2: (7.25, 0) -> (7.25, 15) [up]\n");
    printf("Agent 3: (7.75, 15) -> (7.75, 0) [down]\n");
    printf("Time horizon: N=%d, dt=%.1f s, total time=%.1f s\n", N, dt, N*dt);
    printf("Velocity bounds: -10.0 m/s <= v <= 10.0 m/s\n");
    printf("Safety distance: %.1f m\n\n", d_safe);
    
    // Build centralized QP for 4 agents
    int nV_agent = (N+1)*nx + N*nu;  // 124 per agent
    int nV = 4 * nV_agent;  // 496 total variables
    int n_dyn = 4 * N * nx;  // 320 dynamics constraints
    int n_bounds = 2 * nV;  // 992 bound constraints
    int n_coupling = 6 * (N + 1);  // 126 coupling constraints (6 pairs × 21 time steps)
    int m = n_dyn + n_bounds + n_coupling;  // 1438 total constraints
    
    // Build P (diagonal Hessian)
    std::vector<OSQPFloat> P_x;
    std::vector<OSQPInt> P_i, P_p;
    std::vector<OSQPFloat> P_diag(nV, 0.0);
    P_p.push_back(0);
    for (int i = 0; i < nV; ++i) {
        OSQPFloat val = 0.0;
        int agent_idx = i / nV_agent;
        int local_i = i % nV_agent;
        int k_val = local_i / (nx + nu);
        int rem = local_i % (nx + nu);
        if (rem < nx) {
            val = agents[agent_idx].Q_diag[rem];
        } else {
            if (k_val < N) val = agents[agent_idx].R_diag[rem - nx];
        }
        P_diag[i] = val;
        if (val > 0) {
            P_x.push_back(val);
            P_i.push_back(i);
        }
        P_p.push_back(P_x.size());
    }
    
    // Build q (cost gradient)
    std::vector<OSQPFloat> q(nV, 0.0);
    for (int i = 0; i < nV; ++i) {
        int agent_idx = i / nV_agent;
        int local_i = i % nV_agent;
        int k = local_i / (nx + nu);
        int rem = local_i % (nx + nu);
        if (rem < nx) {
            q[i] = -P_diag[i] * agents[agent_idx].x_ref[k*nx + rem];
        } else {
            if (k < N) q[i] = -P_diag[i] * agents[agent_idx].u_ref[k*nu + (rem - nx)];
        }
    }
    
    // Build A (dynamics, bounds, coupling)
    std::vector<Triplet> A_triplets;
    
    // Dynamics constraints for all 4 agents
    for (int agent_idx = 0; agent_idx < 4; ++agent_idx) {
        int offset = agent_idx * nV_agent;
        int row_offset = agent_idx * N * nx;
        for (int k = 0; k < N; ++k) {
            int idx_xk = offset + k * (nx + nu);
            int idx_uk = idx_xk + nx;
            int idx_xkp1 = offset + (k+1) * (nx + nu);
            for (int j = 0; j < nx; ++j) {
                int row = row_offset + k * nx + j;
                // x_{k+1,j}
                A_triplets.push_back({row, idx_xkp1 + j, 1.0});
                // -A[j,:] * x_k
                for (int l = 0; l < nx; ++l) {
                    OSQPFloat val = -agents[agent_idx].A[j*nx + l];
                    if (fabs(val) > 1e-12) A_triplets.push_back({row, idx_xk + l, val});
                }
                // -B[j,:] * u_k
                for (int l = 0; l < nu; ++l) {
                    OSQPFloat val = -agents[agent_idx].B[j*nu + l];
                    if (fabs(val) > 1e-12) A_triplets.push_back({row, idx_uk + l, val});
                }
            }
        }
    }
    
    // Bounds (upper and lower for all variables)
    for (int i = 0; i < nV; ++i) {
        int row_ub = n_dyn + i;
        A_triplets.push_back({row_ub, i, 1.0});  // x_i <= ub
        
        int row_lb = n_dyn + nV + i;
        A_triplets.push_back({row_lb, i, 1.0});  // x_i >= lb
    }
    
    // Coupling constraints (initialized based on reference, will be updated in SQP)
    // 6 pairs: (0,1), (0,2), (0,3), (1,2), (1,3), (2,3)
    int pair_indices[6][2] = {{0,1}, {0,2}, {0,3}, {1,2}, {1,3}, {2,3}};
    for (int pair = 0; pair < 6; ++pair) {
        int i0 = pair_indices[pair][0];
        int i1 = pair_indices[pair][1];
        for (int k = 0; k <= N; ++k) {
            int row = n_dyn + n_bounds + pair * (N+1) + k;
            OSQPFloat x0_ref = agents[i0].x_ref[k*nx + 0];
            OSQPFloat y0_ref = agents[i0].x_ref[k*nx + 1];
            OSQPFloat x1_ref = agents[i1].x_ref[k*nx + 0];
            OSQPFloat y1_ref = agents[i1].x_ref[k*nx + 1];
            OSQPFloat dx = x0_ref - x1_ref;
            OSQPFloat dy = y0_ref - y1_ref;
            OSQPFloat dist = sqrt(dx*dx + dy*dy);
            
            // Always activate constraint, use safe default normal if agents are too close in reference
            OSQPFloat nx_c, ny_c;
            if (dist > 1e-3) {
                nx_c = dx / dist;
                ny_c = dy / dist;
            } else {
                // Use horizontal separation as default when reference trajectories overlap
                nx_c = 1.0;
                ny_c = 0.0;
            }
            
            int idx_x0 = i0 * nV_agent + k * (nx + nu);
            int idx_y0 = idx_x0 + 1;
            int idx_x1 = i1 * nV_agent + k * (nx + nu);
            int idx_y1 = idx_x1 + 1;
            A_triplets.push_back({row, idx_x0, nx_c});
            A_triplets.push_back({row, idx_y0, ny_c});
            A_triplets.push_back({row, idx_x1, -nx_c});
            A_triplets.push_back({row, idx_y1, -ny_c});
        }
    }
    
    // Sort and build CSC format for A
    std::sort(A_triplets.begin(), A_triplets.end(), [](const Triplet& a, const Triplet& b) {
        if (a.col != b.col) return a.col < b.col;
        return a.row < b.row;
    });
    
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
    
    // Constraint bounds l and u
    std::vector<OSQPFloat> l(m, 0.0);
    std::vector<OSQPFloat> u(m, 0.0);
    
    // Dynamics: equality (l=u=0)
    for (int i = 0; i < n_dyn; ++i) {
        l[i] = 0.0;
        u[i] = 0.0;
    }
    
    // Variable bounds
    for (int i = 0; i < nV; ++i) {
        int agent_idx = i / nV_agent;
        int local_i = i % nV_agent;
        
        int row_ub = n_dyn + i;
        l[row_ub] = -OSQP_INFTY;
        u[row_ub] = agents[agent_idx].ub[local_i % nV_agent];
        
        int row_lb = n_dyn + nV + i;
        l[row_lb] = agents[agent_idx].lb[local_i % nV_agent];
        u[row_lb] = OSQP_INFTY;
        
        // Fix initial positions to initial state (for all 4 agents)
        if (local_i < nx) {
            u[row_ub] = agents[agent_idx].x_ref[0*nx + local_i];
            l[row_lb] = agents[agent_idx].x_ref[0*nx + local_i];
        }
    }
    
    // Initialize coupling bounds to inactive (will be set dynamically in SQP loop)
    for (int i = n_dyn + n_bounds; i < m; ++i) {
        l[i] = -OSQP_INFTY;
        u[i] = OSQP_INFTY;
    }
    
    // Create matrices
    OSQPCscMatrix* P = OSQPCscMatrix_new(nV, nV, P_x.size(), P_x.data(), P_i.data(), P_p.data());
    OSQPCscMatrix* A = OSQPCscMatrix_new(m, nV, A_x.size(), A_x.data(), A_i.data(), A_p.data());
    
    // Settings
    OSQPSettings* settings = OSQPSettings_new();
    settings->verbose = 0;
    settings->max_iter = 20000;
    
    // Solver
    OSQPSolver* solver;
    
    // SQP loop
    const int max_sqp_iter = 300;
    const real_t constraint_tol = 1e-4;
    const real_t objective_tol = 1e-4;
    
    std::vector<real_t> current_trajectory(nV, 0.0);
    std::vector<real_t> previous_trajectory(nV, 0.0);
    real_t previous_objective = 1e30;
    
    // Store dynamics and bounds triplets (constant across SQP iterations)
    std::vector<Triplet> A_triplets_fixed;
    for (const auto& t : A_triplets) {
        if (t.row < n_dyn + n_bounds) {
            A_triplets_fixed.push_back(t);
        }
    }
    
    // Initialize with reference trajectory
    for (int i = 0; i < nV; ++i) {
        int agent_idx = i / nV_agent;
        int local_i = i % nV_agent;
        int k = local_i / (nx + nu);
        int rem = local_i % (nx + nu);
        if (rem < nx) {
            current_trajectory[i] = agents[agent_idx].x_ref[k*nx + rem];
        } else {
            if (k < N) current_trajectory[i] = agents[agent_idx].u_ref[k*nu + (rem - nx)];
        }
    }
    
    bool converged = false;
    int final_iter = 0;
    double total_start = getTime();

    for (int sqp_iter = 0; sqp_iter < max_sqp_iter; ++sqp_iter) {
        // Save previous trajectory for convergence check
        previous_trajectory = current_trajectory;
        
        // Rebuild A_triplets: start with fixed part (dynamics + bounds)
        A_triplets = A_triplets_fixed;
        
        // Add coupling constraints based on current_trajectory
        int num_active_collisions = 0;
        for (int pair = 0; pair < 6; ++pair) {
            int i0 = pair_indices[pair][0];
            int i1 = pair_indices[pair][1];
            for (int k = 0; k <= N; ++k) {
                int row = n_dyn + n_bounds + pair * (N+1) + k;
                int idx0 = i0 * nV_agent + k * (nx + nu);
                int idx1 = i1 * nV_agent + k * (nx + nu);
                real_t p0x = current_trajectory[idx0];
                real_t p0y = current_trajectory[idx0 + 1];
                real_t p1x = current_trajectory[idx1];
                real_t p1y = current_trajectory[idx1 + 1];
                real_t dx = p0x - p1x;
                real_t dy = p0y - p1y;
                real_t dist = sqrt(dx*dx + dy*dy);
                if (dist > 0 && dist < d_safe_constraint * 3.0) {
                    num_active_collisions++;
                    real_t nx_c = dx / dist;
                    real_t ny_c = dy / dist;
                    int idx_x0 = idx0;
                    int idx_y0 = idx0 + 1;
                    int idx_x1 = idx1;
                    int idx_y1 = idx1 + 1;
                    A_triplets.push_back({row, idx_x0, nx_c});
                    A_triplets.push_back({row, idx_y0, ny_c});
                    A_triplets.push_back({row, idx_x1, -nx_c});
                    A_triplets.push_back({row, idx_y1, -ny_c});
                    // Set bounds for active constraint (use margin for numerical tolerance)
                    l[row] = d_safe_constraint;
                    u[row] = OSQP_INFTY;
                } else {
                    // Agents are far - constraint inactive
                    l[row] = -OSQP_INFTY;
                    u[row] = OSQP_INFTY;
                }
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
        
        // Store objective value before cleanup
        real_t current_objective = solver->info->obj_val;
        
        osqp_cleanup(solver);
        OSQPCscMatrix_free(A_new);
        
        // Compute constraint violations
        real_t max_constraint_viol = 0.0;
        real_t min_dist_iter = 1e10;
        for (int pair = 0; pair < 6; ++pair) {
            int i0 = pair_indices[pair][0];
            int i1 = pair_indices[pair][1];
            for (int k = 0; k <= N; ++k) {
                int idx0 = i0 * nV_agent + k * (nx + nu);
                int idx1 = i1 * nV_agent + k * (nx + nu);
                real_t x0 = current_trajectory[idx0];
                real_t y0 = current_trajectory[idx0 + 1];
                real_t x1 = current_trajectory[idx1];
                real_t y1 = current_trajectory[idx1 + 1];
                real_t dist = sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
                if (dist < min_dist_iter) min_dist_iter = dist;
                real_t viol = d_safe - dist;
                if (viol > max_constraint_viol) max_constraint_viol = viol;
            }
        }
        
        // Compute objective change
        real_t obj_change = fabs(current_objective - previous_objective);
        real_t rel_obj_change = (fabs(previous_objective) > 1e-10) ? 
                                 obj_change / fabs(previous_objective) : obj_change;
        
        // Check convergence
        final_iter = sqp_iter + 1;
        if (max_constraint_viol <= 0.0 && rel_obj_change < objective_tol) {
            converged = true;
            previous_objective = current_objective;
            break;
        }
        
        // Update previous objective for next iteration
        previous_objective = current_objective;
    }
    double total_end = getTime();

    // Extract final trajectories
    real_t** z = new real_t*[4];
    for (int i = 0; i < 4; ++i) {
        z[i] = new real_t[nV_agent];
        for (int j = 0; j < nV_agent; ++j) {
            z[i][j] = current_trajectory[i * nV_agent + j];
        }
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
    for (int i = 0; i < 4; ++i) {
        printf("Agent %d: (%.2f, %.2f, %.2f, %.2f)\n", i,
               z[i][idx_N], z[i][idx_N+1], z[i][idx_N+2], z[i][idx_N+3]);
    }
    
    // Verify collision constraints for all pairs
    printf("\nMinimum distances between agent pairs:\n");
    real_t global_min_dist = 1e10;
    for (int pair = 0; pair < 6; ++pair) {
        int i0 = pair_indices[pair][0];
        int i1 = pair_indices[pair][1];
        real_t min_dist_pair = 1e10;
        for (int k = 0; k <= N; ++k) {
            real_t x0 = z[i0][k*(nx+nu)];
            real_t y0 = z[i0][k*(nx+nu)+1];
            real_t x1 = z[i1][k*(nx+nu)];
            real_t y1 = z[i1][k*(nx+nu)+1];
            real_t dist = sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
            if (dist < min_dist_pair) min_dist_pair = dist;
        }
        if (min_dist_pair < global_min_dist) global_min_dist = min_dist_pair;
        printf("  Agents %d-%d: %.3f m\n", i0, i1, min_dist_pair);
    }
    
    printf("\nGlobal minimum distance: %.3f m (required: %.1f m)\n", global_min_dist, d_safe);
    if (global_min_dist >= d_safe) {
        printf("All collision constraints satisfied.\n");
    } else {
        printf("Collision constraints violated!\n");
    }
    
    // Calculate tracking errors
    real_t targets_x[4] = {15.0, 0.0, 7.25, 7.75};
    real_t targets_y[4] = {4.8, 5.3, 15.0, 0.0};
    real_t tracking_errors[4];
    
    printf("\nTracking errors:\n");
    for (int i = 0; i < 4; ++i) {
        real_t x_final = z[i][idx_N];
        real_t y_final = z[i][idx_N + 1];
        real_t error_x = fabs(x_final - targets_x[i]);
        real_t error_y = fabs(y_final - targets_y[i]);
        tracking_errors[i] = sqrt(error_x*error_x + error_y*error_y);
        printf("  Agent %d: %.3f m\n", i, tracking_errors[i]);
    }
    
    // Calculate mean and standard deviation
    real_t mean_error = 0.0;
    for (int i = 0; i < 4; ++i) {
        mean_error += tracking_errors[i];
    }
    mean_error /= 4;
    
    real_t variance = 0.0;
    for (int i = 0; i < 4; ++i) {
        real_t diff = tracking_errors[i] - mean_error;
        variance += diff * diff;
    }
    variance /= 4;
    real_t std_dev = sqrt(variance);
    
    printf("  Mean tracking error: %.3f m\n", mean_error);
    printf("  Std dev tracking error: %.3f m\n", std_dev);
    
    // Print full trajectories
    for (int i = 0; i < 4; ++i) {
        printf("\nAgent %d full trajectory:\n", i);
        for (int k = 0; k <= N; ++k) {
            int traj_idx = k * (nx + nu);
            printf("k=%d: x=(%.3f, %.3f, %.3f, %.3f)", k, 
                   z[i][traj_idx], z[i][traj_idx+1], z[i][traj_idx+2], z[i][traj_idx+3]);
            if (k < N) {
                printf(" u=(%.3f, %.3f)", z[i][traj_idx+nx], z[i][traj_idx+nx+1]);
            }
            printf("\n");
        }
    }
    
    // Cleanup
    OSQPCscMatrix_free(A);
    OSQPCscMatrix_free(P);
    OSQPSettings_free(settings);
    
    for (int i = 0; i < 4; ++i) {
        delete[] z[i];
    }
    delete[] z;
    delete[] agents;
    
    printf("Solve time: %.3f ms\n", total_end - total_start);
    
    return 0;
}
