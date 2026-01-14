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
    printf("                    10-AGENT CENTRALIZED MPC WITH OSQP\n");
    printf("================================================================================\n\n");
    
    const int NUM_AGENTS = 10;
    int N = 20;
    int nx = 4;
    int nu = 2;
    real_t dt = 0.2;
    real_t v_max = 10.0;
    real_t d_safe = 2.0;
    real_t d_safe_margin = 0.05;  // 5cm margin for OSQP numerical tolerance
    real_t d_safe_constraint = d_safe + d_safe_margin;  // Use 2.05m in constraints
    
    // Create 10 PointMass agents
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
    
    real_t Q_pos = 2.0;   // Position tracking (matching TurboADMM)
    real_t Q_vel = 0.1;
    real_t R_ctrl = 0.1;
    
    for (int i = 0; i < NUM_AGENTS; ++i) {
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
    
    // Agent 4: (0, 0) -> (15, 15) - diagonal up-right
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[4].x_ref[k*nx + 0] = 0.0 + alpha * 15.0;
        agents[4].x_ref[k*nx + 1] = 0.0 + alpha * 15.0;
        agents[4].x_ref[k*nx + 2] = 0.0;
        agents[4].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[4].u_ref[k*nu + 0] = 0.0;
        agents[4].u_ref[k*nu + 1] = 0.0;
    }
    
    // Agent 5: (0, 15) -> (15, 0) - diagonal down-right
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[5].x_ref[k*nx + 0] = 0.0 + alpha * 15.0;
        agents[5].x_ref[k*nx + 1] = 15.0 + alpha * (-15.0);
        agents[5].x_ref[k*nx + 2] = 0.0;
        agents[5].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[5].u_ref[k*nu + 0] = 0.0;
        agents[5].u_ref[k*nu + 1] = 0.0;
    }
    
    // Agent 6: (15, 15) -> (0, 0) - diagonal down-left
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[6].x_ref[k*nx + 0] = 15.0 + alpha * (-15.0);
        agents[6].x_ref[k*nx + 1] = 15.0 + alpha * (-15.0);
        agents[6].x_ref[k*nx + 2] = 0.0;
        agents[6].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[6].u_ref[k*nu + 0] = 0.0;
        agents[6].u_ref[k*nu + 1] = 0.0;
    }
    
    // Agent 7: (15, 0) -> (0, 15) - diagonal up-left
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[7].x_ref[k*nx + 0] = 15.0 + alpha * (-15.0);
        agents[7].x_ref[k*nx + 1] = 0.0 + alpha * 15.0;
        agents[7].x_ref[k*nx + 2] = 0.0;
        agents[7].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[7].u_ref[k*nu + 0] = 0.0;
        agents[7].u_ref[k*nu + 1] = 0.0;
    }
    
    // Agent 8: (10.0, 5.0) -> (0.0, 10.0) - diagonal down-left
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[8].x_ref[k*nx + 0] = 10.0 + alpha * (-10.0);
        agents[8].x_ref[k*nx + 1] = 5.0 + alpha * 5.0;
        agents[8].x_ref[k*nx + 2] = 0.0;
        agents[8].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[8].u_ref[k*nu + 0] = 0.0;
        agents[8].u_ref[k*nu + 1] = 0.0;
    }
    
    // Agent 9: (5.0, 5.0) -> (15.0, 10.0) - diagonal up-right
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[9].x_ref[k*nx + 0] = 5.0 + alpha * 10.0;
        agents[9].x_ref[k*nx + 1] = 5.0 + alpha * 5.0;
        agents[9].x_ref[k*nx + 2] = 0.0;
        agents[9].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[9].u_ref[k*nu + 0] = 0.0;
        agents[9].u_ref[k*nu + 1] = 0.0;
    }
    
    printf("Agent Configuration:\n");
    printf("  Agent 0: (0, 4.8) -> (15, 4.8) [horizontal right]\n");
    printf("  Agent 1: (15, 5.3) -> (0, 5.3) [horizontal left]\n");
    printf("  Agent 2: (7.25, 0) -> (7.25, 15) [vertical up]\n");
    printf("  Agent 3: (7.75, 15) -> (7.75, 0) [vertical down]\n");
    printf("  Agent 4: (0, 0) -> (15, 15) [diagonal ↗]\n");
    printf("  Agent 5: (0, 15) -> (15, 0) [diagonal ↘]\n");
    printf("  Agent 6: (15, 15) -> (0, 0) [diagonal ↙]\n");
    printf("  Agent 7: (15, 0) -> (0, 15) [diagonal ↖]\n");
    printf("  Agent 8: (10, 5) -> (0, 10) [diagonal ↙]\n");
    printf("  Agent 9: (5, 5) -> (15, 10) [diagonal ↗]\n");
    printf("Time horizon: N=%d, dt=%.1f s, total time=%.1f s\n", N, dt, N*dt);
    printf("Velocity bounds: -10.0 m/s <= v <= 10.0 m/s\n");
    printf("Safety distance: %.1f m\n\n", d_safe);
    
    // Build centralized QP for 10 agents
    int nV_agent = (N+1)*nx + N*nu;  // 124 per agent
    int nV = NUM_AGENTS * nV_agent;  // 1240 total variables
    int n_dyn = NUM_AGENTS * N * nx;  // 800 dynamics constraints
    int n_bounds = 2 * nV;  // 2480 bound constraints
    int n_coupling = 45 * (N + 1);  // 945 coupling constraints (45 pairs × 21 time steps)
    int m = n_dyn + n_bounds + n_coupling;  // 4225 total constraints
    
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
    for (int i = 0; i < NUM_AGENTS; ++i) {
        for (int k = 0; k <= N; ++k) {
            for (int j = 0; j < nx; ++j) {
                int idx = i * nV_agent + k * (nx + nu) + j;
                q[idx] = -agents[i].Q_diag[j] * agents[i].x_ref[k*nx + j];
            }
        }
        for (int k = 0; k < N; ++k) {
            for (int j = 0; j < nu; ++j) {
                int idx = i * nV_agent + k * (nx + nu) + nx + j;
                q[idx] = -agents[i].R_diag[j] * agents[i].u_ref[k*nu + j];
            }
        }
    }
    
    // Build constraint matrix A
    std::vector<Triplet> A_triplets;
    std::vector<OSQPFloat> l(m), u(m);
    int row = 0;
    
    // Dynamics constraints: x[k+1] = A*x[k] + B*u[k]
    for (int i = 0; i < NUM_AGENTS; ++i) {
        for (int k = 0; k < N; ++k) {
            for (int j = 0; j < nx; ++j) {
                int col_xk = i * nV_agent + k * (nx + nu) + j;
                int col_uk = i * nV_agent + k * (nx + nu) + nx;
                int col_xkp1 = i * nV_agent + (k+1) * (nx + nu) + j;
                
                // A*x[k]
                for (int jj = 0; jj < nx; ++jj) {
                    A_triplets.push_back({row, col_xk - j + jj, agents[i].A[j*nx + jj]});
                }
                // B*u[k]
                for (int jj = 0; jj < nu; ++jj) {
                    A_triplets.push_back({row, col_uk + jj, agents[i].B[j*nu + jj]});
                }
                // -x[k+1]
                A_triplets.push_back({row, col_xkp1, -1.0});
                
                l[row] = 0.0;
                u[row] = 0.0;
                row++;
            }
        }
    }
    
    // Variable bounds (converted to constraints)
    for (int i = 0; i < nV; ++i) {
        // Lower bound: x_i >= lb_i  =>  x_i >= lb_i
        A_triplets.push_back({row, i, 1.0});
        int agent_idx = i / nV_agent;
        int local_i = i % nV_agent;
        l[row] = agents[agent_idx].lb[local_i];
        u[row] = OSQPFloat(1e20);
        row++;
        
        // Upper bound: x_i <= ub_i  =>  x_i <= ub_i
        A_triplets.push_back({row, i, 1.0});
        l[row] = OSQPFloat(-1e20);
        u[row] = agents[agent_idx].ub[local_i];
        row++;
    }
    
    // Collision avoidance constraints (45 pairs)
    int pair_indices[45][2] = {
        {0,1}, {0,2}, {0,3}, {0,4}, {0,5}, {0,6}, {0,7}, {0,8}, {0,9},
        {1,2}, {1,3}, {1,4}, {1,5}, {1,6}, {1,7}, {1,8}, {1,9},
        {2,3}, {2,4}, {2,5}, {2,6}, {2,7}, {2,8}, {2,9},
        {3,4}, {3,5}, {3,6}, {3,7}, {3,8}, {3,9},
        {4,5}, {4,6}, {4,7}, {4,8}, {4,9},
        {5,6}, {5,7}, {5,8}, {5,9},
        {6,7}, {6,8}, {6,9},
        {7,8}, {7,9},
        {8,9}
    };
    
    // Initialize with collision-free trajectory by making agents wait at start
    // This avoids the infeasible initial condition where all agents collide at center
    std::vector<real_t> current_trajectory(nV);
    for (int agent_idx = 0; agent_idx < NUM_AGENTS; ++agent_idx) {
        int wait_steps = agent_idx * 3;  // Each agent waits 3 time steps longer than previous
        for (int k = 0; k <= N; ++k) {
            int effective_k = (k > wait_steps) ? (k - wait_steps) : 0;
            if (effective_k > N) effective_k = N;
            
            // States
            for (int j = 0; j < nx; ++j) {
                int idx = agent_idx * nV_agent + k * (nx + nu) + j;
                current_trajectory[idx] = agents[agent_idx].x_ref[effective_k * nx + j];
            }
            
            // Controls
            if (k < N) {
                for (int j = 0; j < nu; ++j) {
                    int idx = agent_idx * nV_agent + k * (nx + nu) + nx + j;
                    if (k < wait_steps) {
                        current_trajectory[idx] = 0.0;  // Zero control during wait
                    } else {
                        int control_k = effective_k < N ? effective_k : N-1;
                        current_trajectory[idx] = agents[agent_idx].u_ref[control_k * nu + j];
                    }
                }
            }
        }
    }
    
    // Save fixed part of constraint matrix (dynamics + bounds)
    std::vector<Triplet> A_triplets_fixed = A_triplets;
    
    // Create OSQP settings (once, outside loop) - optimized for 8-agent
    OSQPSettings* settings = OSQPSettings_new();
    settings->verbose = 0;
    settings->max_iter = 300;  // Reduced from 300 to fail fast on difficult QPs
    
    // Create P matrix (once, outside loop)
    OSQPCscMatrix* P = OSQPCscMatrix_new(nV, nV, P_x.size(), P_x.data(), P_i.data(), P_p.data());
    
    // SQP loop
    int max_sqp_iter = 300;  // Increased for 8-agent complexity
    real_t objective_tol = 1e-4;  // Relaxed tolerance
    bool converged = false;
    int final_iter = 0;
    std::vector<real_t> previous_trajectory;
    real_t previous_objective = 0.0;
    double total_start = getTime();
    printf("Starting SQP iterations...\n\n");
    
    for (int sqp_iter = 0; sqp_iter < max_sqp_iter; ++sqp_iter) {
        double iter_start = getTime();
        
        // Save previous trajectory for convergence check
        previous_trajectory = current_trajectory;
        
        // Rebuild A_triplets: start with fixed part (dynamics + bounds)
        double rebuild_start = getTime();
        A_triplets = A_triplets_fixed;
        
        // Add coupling constraints based on current_trajectory
        int num_active_collisions = 0;
        for (int pair = 0; pair < 45; ++pair) {
            int i0 = pair_indices[pair][0];
            int i1 = pair_indices[pair][1];
            for (int k = 0; k <= N; ++k) {
                int constraint_row = n_dyn + n_bounds + pair * (N+1) + k;
                int idx0 = i0 * nV_agent + k * (nx + nu);
                int idx1 = i1 * nV_agent + k * (nx + nu);
                real_t p0x = current_trajectory[idx0];
                real_t p0y = current_trajectory[idx0 + 1];
                real_t p1x = current_trajectory[idx1];
                real_t p1y = current_trajectory[idx1 + 1];
                real_t dx = p0x - p1x;
                real_t dy = p0y - p1y;
                real_t dist = sqrt(dx*dx + dy*dy);
                
                // Selective activation: only add constraint when agents are close
                if (dist > 0 && dist < d_safe_constraint * 2.0) {
                    num_active_collisions++;
                    real_t nx_c = dx / dist;
                    real_t ny_c = dy / dist;
                    int idx_x0 = idx0;
                    int idx_y0 = idx0 + 1;
                    int idx_x1 = idx1;
                    int idx_y1 = idx1 + 1;
                    A_triplets.push_back({constraint_row, idx_x0, nx_c});
                    A_triplets.push_back({constraint_row, idx_y0, ny_c});
                    A_triplets.push_back({constraint_row, idx_x1, -nx_c});
                    A_triplets.push_back({constraint_row, idx_y1, -ny_c});
                    // Set bounds for active constraint
                    l[constraint_row] = d_safe_constraint;
                    u[constraint_row] = OSQP_INFTY;
                } else {
                    // Agents are far - constraint inactive
                    l[constraint_row] = -OSQP_INFTY;
                    u[constraint_row] = OSQP_INFTY;
                }
            }
        }
        double rebuild_end = getTime();
        
        // Sort A triplets
        std::sort(A_triplets.begin(), A_triplets.end(), [](const Triplet& a, const Triplet& b) {
            if (a.col != b.col) return a.col < b.col;
            return a.row < b.row;
        });
        
        // Build A CSC
        std::vector<OSQPFloat> A_x_iter(A_triplets.size());
        std::vector<OSQPInt> A_i_iter(A_triplets.size()), A_p_iter(nV + 1, 0);
        int idx_sort = 0;
        int current_col_sort = 0;
        for (auto& t : A_triplets) {
            while (current_col_sort < t.col) {
                A_p_iter[current_col_sort + 1] = idx_sort;
                current_col_sort++;
            }
            A_i_iter[idx_sort] = t.row;
            A_x_iter[idx_sort] = t.val;
            idx_sort++;
            A_p_iter[current_col_sort + 1] = idx_sort;
        }
        while (current_col_sort < nV) {
            A_p_iter[current_col_sort + 1] = idx_sort;
            current_col_sort++;
        }
        
        // Create matrices
        OSQPCscMatrix* A_new = OSQPCscMatrix_new(m, nV, A_x_iter.size(), A_x_iter.data(), A_i_iter.data(), A_p_iter.data());
        
        // Solver
        OSQPSolver* solver_iter;
        OSQPInt exitflag = osqp_setup(&solver_iter, P, q.data(), A_new, l.data(), u.data(), m, nV, settings);
        
        if (exitflag != 0) {
            printf("OSQP setup failed\n");
            return 1;
        }
        
        // Warm-start with previous solution (skip first iteration)
        if (sqp_iter > 0) {
            osqp_warm_start(solver_iter, current_trajectory.data(), NULL);
        }
        
        double solve_start = getTime();
        exitflag = osqp_solve(solver_iter);
        double solve_end = getTime();
        double solve_time = solve_end - solve_start;
        
        if (exitflag != 0) {
            printf("OSQP solve failed with exit code %lld\n", (long long)exitflag);
            osqp_cleanup(solver_iter);
            OSQPCscMatrix_free(A_new);
            return 1;
        }
        
        // Check OSQP status (print only first and every 50th iteration)
        if (sqp_iter == 0 || (sqp_iter + 1) % 50 == 0) {
            printf("  SQP iter %d: OSQP status=%s, iters=%lld, obj=%.2f, active_coll=%d, solve_time=%.2f ms\n",
                   sqp_iter + 1, solver_iter->info->status, (long long)solver_iter->info->iter, 
                   solver_iter->info->obj_val, num_active_collisions, solve_time);
        }
        
        // Check if solution is valid
        if (!solver_iter->solution || !solver_iter->solution->x) {
            printf("OSQP returned NULL solution\n");
            osqp_cleanup(solver_iter);
            OSQPCscMatrix_free(A_new);
            return 1;
        }
        
        // Extract solution and check if it changed
        real_t max_change = 0.0;
        OSQPFloat* solution = solver_iter->solution->x;
        for (int i = 0; i < nV; ++i) {
            real_t change = fabs(solution[i] - current_trajectory[i]);
            if (change > max_change) max_change = change;
            current_trajectory[i] = solution[i];
        }
        
        // Store objective value before cleanup
        real_t current_objective = solver_iter->info->obj_val;
        
        if (sqp_iter == 0 || (sqp_iter + 1) % 50 == 0) {
            printf("  Max trajectory change: %.6f\n", max_change);
        }
        
        osqp_cleanup(solver_iter);
        OSQPCscMatrix_free(A_new);
        
        double iter_end = getTime();
        
        // Compute constraint violations
        real_t max_constraint_viol = 0.0;
        real_t min_dist_iter = 1e10;
        
        for (int pair = 0; pair < 45; ++pair) {
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
        
        // Print progress every 10 iterations
        if ((sqp_iter + 1) % 10 == 0 || sqp_iter == 0) {
            printf("SQP iter %3d: obj=%.2f, max_viol=%.4f m, min_dist=%.3f m, obj_change=%.2e\n",
                   sqp_iter + 1, current_objective, max_constraint_viol, min_dist_iter, rel_obj_change);
        }
        
        // Check convergence
        final_iter = sqp_iter + 1;
        if (max_constraint_viol <= 0.0 && rel_obj_change < objective_tol) {
            converged = true;
            previous_objective = current_objective;
            printf("Converged at iteration %d!\n", final_iter);
            break;
        }
        
        // Update previous objective for next iteration
        previous_objective = current_objective;
    }
    double total_end = getTime();
    printf("\nSolve time: %.3f ms\n", total_end - total_start);
    // Cleanup OSQP resources
    OSQPCscMatrix_free(P);
    OSQPSettings_free(settings);
    
    // Extract final trajectories
    real_t** z = new real_t*[NUM_AGENTS];
    for (int i = 0; i < NUM_AGENTS; ++i) {
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
    for (int i = 0; i < NUM_AGENTS; ++i) {
        printf("Agent %d: (%.2f, %.2f, %.2f, %.2f)\n", i,
               z[i][idx_N], z[i][idx_N+1], z[i][idx_N+2], z[i][idx_N+3]);
    }
    
    // Verify collision constraints for all pairs
    printf("\nMinimum distances between agent pairs:\n");
    real_t global_min_dist = 1e10;
    for (int pair = 0; pair < 45; ++pair) {
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
    real_t targets_x[NUM_AGENTS] = {15.0, 0.0, 7.25, 7.75, 15.0, 15.0, 0.0, 0.0, 0.0, 15.0};
    real_t targets_y[NUM_AGENTS] = {4.8, 5.3, 15.0, 0.0, 15.0, 0.0, 0.0, 15.0, 10.0, 10.0};
    real_t tracking_errors[NUM_AGENTS];
    
    printf("\nTracking errors:\n");
    for (int i = 0; i < NUM_AGENTS; ++i) {
        real_t x_final = z[i][idx_N];
        real_t y_final = z[i][idx_N + 1];
        real_t error_x = fabs(x_final - targets_x[i]);
        real_t error_y = fabs(y_final - targets_y[i]);
        tracking_errors[i] = sqrt(error_x*error_x + error_y*error_y);
        printf("  Agent %d: %.3f m\n", i, tracking_errors[i]);
    }
    
    // Calculate mean and standard deviation
    real_t mean_error = 0.0;
    for (int i = 0; i < NUM_AGENTS; ++i) {
        mean_error += tracking_errors[i];
    }
    mean_error /= NUM_AGENTS;
    
    real_t variance = 0.0;
    for (int i = 0; i < NUM_AGENTS; ++i) {
        real_t diff = tracking_errors[i] - mean_error;
        variance += diff * diff;
    }
    variance /= NUM_AGENTS;
    real_t std_dev = sqrt(variance);
    
    printf("  Mean tracking error: %.3f m\n", mean_error);
    printf("  Std dev tracking error: %.3f m\n", std_dev);
    
    // Cleanup (matrices and settings already freed in loop)
    
    for (int i = 0; i < NUM_AGENTS; ++i) {
        delete[] z[i];
    }
    delete[] z;
    delete[] agents;
    
   
    
    return 0;
}
