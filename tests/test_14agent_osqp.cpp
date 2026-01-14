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
    printf("                    14-AGENT CENTRALIZED MPC WITH OSQP\n");
    printf("                         20m × 20m Workspace\n");
    printf("================================================================================\n\n");
    
    const int NUM_AGENTS = 14;
    int N = 20;
    int nx = 4;
    int nu = 2;
    real_t dt = 0.2;
    real_t v_max = 10.0;
    real_t d_safe = 2.0;
    real_t d_safe_margin = 0.05;
    real_t d_safe_constraint = d_safe + d_safe_margin;
    
    // Create 14 PointMass agents
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
    
    // Set cost weights
    real_t Q_pos = 15.0;
    real_t Q_vel = 0.1;
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
    // TOP SIDE (4 agents moving DOWN)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[0].x_ref[k*nx + 0] = 2.75;
        agents[0].x_ref[k*nx + 1] = 19.0 + alpha * (-18.0);
        agents[0].x_ref[k*nx + 2] = 0.0;
        agents[0].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[0].u_ref[k*nu + 0] = 0.0;
        agents[0].u_ref[k*nu + 1] = 0.0;
    }
    
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[1].x_ref[k*nx + 0] = 8.75;
        agents[1].x_ref[k*nx + 1] = 19.0 + alpha * (-18.0);
        agents[1].x_ref[k*nx + 2] = 0.0;
        agents[1].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[1].u_ref[k*nu + 0] = 0.0;
        agents[1].u_ref[k*nu + 1] = 0.0;
    }
    
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[2].x_ref[k*nx + 0] = 12.75;
        agents[2].x_ref[k*nx + 1] = 19.0 + alpha * (-18.0);
        agents[2].x_ref[k*nx + 2] = 0.0;
        agents[2].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[2].u_ref[k*nu + 0] = 0.0;
        agents[2].u_ref[k*nu + 1] = 0.0;
    }
    
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[12].x_ref[k*nx + 0] = 16.75;
        agents[12].x_ref[k*nx + 1] = 19.0 + alpha * (-18.0);
        agents[12].x_ref[k*nx + 2] = 0.0;
        agents[12].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[12].u_ref[k*nu + 0] = 0.0;
        agents[12].u_ref[k*nu + 1] = 0.0;
    }
    
    // RIGHT SIDE (3 agents moving LEFT)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[3].x_ref[k*nx + 0] = 19.0 + alpha * (-18.0);
        agents[3].x_ref[k*nx + 1] = 15.25;
        agents[3].x_ref[k*nx + 2] = 0.0;
        agents[3].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[3].u_ref[k*nu + 0] = 0.0;
        agents[3].u_ref[k*nu + 1] = 0.0;
    }
    
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[4].x_ref[k*nx + 0] = 19.0 + alpha * (-18.0);
        agents[4].x_ref[k*nx + 1] = 11.25;
        agents[4].x_ref[k*nx + 2] = 0.0;
        agents[4].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[4].u_ref[k*nu + 0] = 0.0;
        agents[4].u_ref[k*nu + 1] = 0.0;
    }
    
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[5].x_ref[k*nx + 0] = 19.0 + alpha * (-18.0);
        agents[5].x_ref[k*nx + 1] = 7.25;
        agents[5].x_ref[k*nx + 2] = 0.0;
        agents[5].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[5].u_ref[k*nu + 0] = 0.0;
        agents[5].u_ref[k*nu + 1] = 0.0;
    }
    
    // BOTTOM SIDE (4 agents moving UP)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[6].x_ref[k*nx + 0] = 17.25;
        agents[6].x_ref[k*nx + 1] = 1.0 + alpha * 18.0;
        agents[6].x_ref[k*nx + 2] = 0.0;
        agents[6].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[6].u_ref[k*nu + 0] = 0.0;
        agents[6].u_ref[k*nu + 1] = 0.0;
    }
    
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[7].x_ref[k*nx + 0] = 13.25;
        agents[7].x_ref[k*nx + 1] = 1.0 + alpha * 18.0;
        agents[7].x_ref[k*nx + 2] = 0.0;
        agents[7].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[7].u_ref[k*nu + 0] = 0.0;
        agents[7].u_ref[k*nu + 1] = 0.0;
    }
    
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[8].x_ref[k*nx + 0] = 9.25;
        agents[8].x_ref[k*nx + 1] = 1.0 + alpha * 18.0;
        agents[8].x_ref[k*nx + 2] = 0.0;
        agents[8].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[8].u_ref[k*nu + 0] = 0.0;
        agents[8].u_ref[k*nu + 1] = 0.0;
    }
    
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[13].x_ref[k*nx + 0] = 3.25;
        agents[13].x_ref[k*nx + 1] = 1.0 + alpha * 18.0;
        agents[13].x_ref[k*nx + 2] = 0.0;
        agents[13].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[13].u_ref[k*nu + 0] = 0.0;
        agents[13].u_ref[k*nu + 1] = 0.0;
    }
    
    // LEFT SIDE (3 agents moving RIGHT)
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[9].x_ref[k*nx + 0] = 1.0 + alpha * 18.0;
        agents[9].x_ref[k*nx + 1] = 11.25;
        agents[9].x_ref[k*nx + 2] = 0.0;
        agents[9].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[9].u_ref[k*nu + 0] = 0.0;
        agents[9].u_ref[k*nu + 1] = 0.0;
    }
    
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[10].x_ref[k*nx + 0] = 1.0 + alpha * 18.0;
        agents[10].x_ref[k*nx + 1] = 7.25;
        agents[10].x_ref[k*nx + 2] = 0.0;
        agents[10].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[10].u_ref[k*nu + 0] = 0.0;
        agents[10].u_ref[k*nu + 1] = 0.0;
    }
    
    for (int k = 0; k <= N; ++k) {
        real_t alpha = (real_t)k / N;
        agents[11].x_ref[k*nx + 0] = 1.0 + alpha * 18.0;
        agents[11].x_ref[k*nx + 1] = 3.25;
        agents[11].x_ref[k*nx + 2] = 0.0;
        agents[11].x_ref[k*nx + 3] = 0.0;
    }
    for (int k = 0; k < N; ++k) {
        agents[11].u_ref[k*nu + 0] = 0.0;
        agents[11].u_ref[k*nu + 1] = 0.0;
    }
    
    printf("Agent Configuration (20m × 20m workspace, 14 agents):\n");
    printf("  TOP SIDE (4 agents moving down)\n");
    printf("  RIGHT SIDE (3 agents moving left)\n");
    printf("  BOTTOM SIDE (4 agents moving up)\n");
    printf("  LEFT SIDE (3 agents moving right)\n");
    printf("Time horizon: N=%d, dt=%.1f s, total time=%.1f s\n", N, dt, N*dt);
    printf("Velocity bounds: -%.1f m/s <= v <= %.1f m/s\n", v_max, v_max);
    printf("Safety distance: %.1f m\n\n", d_safe);
    
    // Build centralized QP for 14 agents
    int nV_agent = (N+1)*nx + N*nu;  // 124 per agent
    int nV = NUM_AGENTS * nV_agent;  // 1736 total variables
    int n_dyn = NUM_AGENTS * N * nx;  // 1120 dynamics constraints
    int n_bounds = 2 * nV;  // 3472 bound constraints
    int n_coupling = 91 * (N + 1);  // 1911 coupling constraints (91 pairs × 21 time steps)
    int m = n_dyn + n_bounds + n_coupling;  // 6503 total constraints
    
    printf("Problem dimensions:\n");
    printf("  Variables: %d\n", nV);
    printf("  Dynamics constraints: %d\n", n_dyn);
    printf("  Bound constraints: %d\n", n_bounds);
    printf("  Collision constraints: %d (91 pairs)\n", n_coupling);
    printf("  Total constraints: %d\n\n", m);
    
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
        // Lower bound
        A_triplets.push_back({row, i, 1.0});
        int agent_idx = i / nV_agent;
        int local_i = i % nV_agent;
        l[row] = agents[agent_idx].lb[local_i];
        u[row] = OSQP_INFTY;
        row++;
        
        // Upper bound
        A_triplets.push_back({row, i, 1.0});
        l[row] = -OSQP_INFTY;
        u[row] = agents[agent_idx].ub[local_i];
        row++;
    }
    
    // Collision avoidance constraints (91 pairs)
    int pair_indices[91][2] = {
        {0,1}, {0,2}, {0,3}, {0,4}, {0,5}, {0,6}, {0,7}, {0,8}, {0,9}, {0,10}, {0,11}, {0,12}, {0,13},
        {1,2}, {1,3}, {1,4}, {1,5}, {1,6}, {1,7}, {1,8}, {1,9}, {1,10}, {1,11}, {1,12}, {1,13},
        {2,3}, {2,4}, {2,5}, {2,6}, {2,7}, {2,8}, {2,9}, {2,10}, {2,11}, {2,12}, {2,13},
        {3,4}, {3,5}, {3,6}, {3,7}, {3,8}, {3,9}, {3,10}, {3,11}, {3,12}, {3,13},
        {4,5}, {4,6}, {4,7}, {4,8}, {4,9}, {4,10}, {4,11}, {4,12}, {4,13},
        {5,6}, {5,7}, {5,8}, {5,9}, {5,10}, {5,11}, {5,12}, {5,13},
        {6,7}, {6,8}, {6,9}, {6,10}, {6,11}, {6,12}, {6,13},
        {7,8}, {7,9}, {7,10}, {7,11}, {7,12}, {7,13},
        {8,9}, {8,10}, {8,11}, {8,12}, {8,13},
        {9,10}, {9,11}, {9,12}, {9,13},
        {10,11}, {10,12}, {10,13},
        {11,12}, {11,13},
        {12,13}
    };
    
    // Initialize with staggered start
    std::vector<real_t> current_trajectory(nV);
    for (int agent_idx = 0; agent_idx < NUM_AGENTS; ++agent_idx) {
        int wait_steps = agent_idx * 2;
        for (int k = 0; k <= N; ++k) {
            int effective_k = (k > wait_steps) ? (k - wait_steps) : 0;
            if (effective_k > N) effective_k = N;
            
            for (int j = 0; j < nx; ++j) {
                int idx = agent_idx * nV_agent + k * (nx + nu) + j;
                current_trajectory[idx] = agents[agent_idx].x_ref[effective_k * nx + j];
            }
            
            if (k < N) {
                for (int j = 0; j < nu; ++j) {
                    int idx = agent_idx * nV_agent + k * (nx + nu) + nx + j;
                    if (k < wait_steps) {
                        current_trajectory[idx] = 0.0;
                    } else {
                        int control_k = effective_k < N ? effective_k : N-1;
                        current_trajectory[idx] = agents[agent_idx].u_ref[control_k * nu + j];
                    }
                }
            }
        }
    }
    
    // Create P matrix
    OSQPCscMatrix* P = OSQPCscMatrix_new(nV, nV, P_x.size(), P_x.data(), P_i.data(), P_p.data());
    
    // OSQP settings
    OSQPSettings* settings = (OSQPSettings*)malloc(sizeof(OSQPSettings));
    osqp_set_default_settings(settings);
    settings->verbose = 0;
    settings->eps_abs = 1e-3;
    settings->eps_rel = 1e-3;
    settings->max_iter = 300;

    
    // SQP loop
    int max_sqp_iter = 200;
    real_t objective_tol = 1e-2;
    bool converged = false;
    int final_iter = 0;
    real_t previous_objective = 1e10;
    
    double total_time = 0.0;
    double total_start = getTime();
    for (int sqp_iter = 0; sqp_iter < max_sqp_iter; ++sqp_iter) {
        double rebuild_start = getTime();
        
        // Rebuild collision constraints
        int constraint_row = n_dyn + n_bounds;
        A_triplets.resize(n_dyn * (2*nx + nu) + n_bounds);
        
        for (int pair = 0; pair < 91; ++pair) {
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
                
                if (dist < 1e-6) dist = 1e-6;
                
                if (dist < 3.0 * d_safe_constraint) {
                    real_t grad_x0 = (x0 - x1) / dist;
                    real_t grad_y0 = (y0 - y1) / dist;
                    real_t grad_x1 = (x1 - x0) / dist;
                    real_t grad_y1 = (y1 - y0) / dist;
                    
                    A_triplets.push_back({constraint_row, idx0, grad_x0});
                    A_triplets.push_back({constraint_row, idx0 + 1, grad_y0});
                    A_triplets.push_back({constraint_row, idx1, grad_x1});
                    A_triplets.push_back({constraint_row, idx1 + 1, grad_y1});
                    
                    real_t rhs = d_safe_constraint - dist + grad_x0 * x0 + grad_y0 * y0 + grad_x1 * x1 + grad_y1 * y1;
                    l[constraint_row] = rhs;
                    u[constraint_row] = OSQP_INFTY;
                } else {
                    l[constraint_row] = -OSQP_INFTY;
                    u[constraint_row] = OSQP_INFTY;
                }
                constraint_row++;
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
        double t_start = getTime();
        OSQPInt exitflag = osqp_setup(&solver_iter, P, q.data(), A_new, l.data(), u.data(), m, nV, settings);
        
        if (exitflag != 0) {
            printf("OSQP setup failed\n");
            return 1;
        }
        
        // Warm-start
        if (sqp_iter > 0) {
            osqp_warm_start(solver_iter, current_trajectory.data(), NULL);
        }
        
        osqp_solve(solver_iter);
        double t_end = getTime();
        total_time += (t_end - t_start);
        
        // Update trajectory
        real_t current_objective = 0.0;
        for (int i = 0; i < nV; ++i) {
            current_trajectory[i] = solver_iter->solution->x[i];
            current_objective += 0.5 * P_diag[i] * current_trajectory[i] * current_trajectory[i] + q[i] * current_trajectory[i];
        }
        
        // Check convergence
        real_t max_constraint_viol = 0.0;
        real_t min_dist_iter = 1e10;
        
        for (int pair = 0; pair < 91; ++pair) {
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
        
        real_t obj_change = fabs(current_objective - previous_objective);
        real_t rel_obj_change = (fabs(previous_objective) > 1e-10) ? 
                                 obj_change / fabs(previous_objective) : obj_change;
        
        if ((sqp_iter + 1) % 10 == 0 || sqp_iter == 0) {
            printf("SQP iter %3d: obj=%.2f, max_viol=%.4f m, min_dist=%.3f m, obj_change=%.2e\n",
                   sqp_iter + 1, current_objective, max_constraint_viol, min_dist_iter, rel_obj_change);
        }
        
        final_iter = sqp_iter + 1;
        if (max_constraint_viol <= 0.0 && rel_obj_change < objective_tol) {
            converged = true;
            previous_objective = current_objective;
            printf("Converged at iteration %d!\n", final_iter);
            osqp_cleanup(solver_iter);
            OSQPCscMatrix_free(A_new);
            break;
        }
        
        previous_objective = current_objective;
        osqp_cleanup(solver_iter);
        OSQPCscMatrix_free(A_new);
    }
    double total_end = getTime();

    printf("\n================================================================================\n");
    printf("                              RESULTS\n");
    printf("================================================================================\n\n");
    
    printf("SQP Statistics:\n");
    printf("  Iterations: %d\n", final_iter);
    printf("  Converged: %s\n", converged ? "YES" : "NO");
    printf("  Solve time: %.2f ms\n\n", total_end - total_start);
    
    // Compute tracking errors
    real_t tracking_errors[NUM_AGENTS];
    for (int agent_idx = 0; agent_idx < NUM_AGENTS; ++agent_idx) {
        int offset = agent_idx * nV_agent;
        real_t x_final = current_trajectory[offset + N * (nx + nu) + 0];
        real_t y_final = current_trajectory[offset + N * (nx + nu) + 1];
        real_t target_x = agents[agent_idx].x_ref[N*nx + 0];
        real_t target_y = agents[agent_idx].x_ref[N*nx + 1];
        real_t error_x = x_final - target_x;
        real_t error_y = y_final - target_y;
        tracking_errors[agent_idx] = sqrt(error_x*error_x + error_y*error_y);
    }
    
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
    
    printf("Tracking Performance:\n");
    printf("  Mean tracking error: %.3f m\n", mean_error);
    printf("  Std dev tracking error: %.3f m\n\n", std_dev);
    
    // Check collisions
    real_t global_min_dist = 1e9;
    int total_violations = 0;
    
    for (int pair = 0; pair < 91; ++pair) {
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
            
            if (dist < global_min_dist) global_min_dist = dist;
            if (dist < d_safe) total_violations++;
        }
    }
    
    printf("Collision Avoidance:\n");
    printf("  Minimum distance: %.3f m (safety: %.1f m)\n", global_min_dist, d_safe);
    printf("  Violations: %d/%d stages\n\n", total_violations, (N+1)*91);
    
    bool collision_ok = (global_min_dist >= d_safe);
    bool tracking_ok = (mean_error < 3.0);
    
    if (converged && collision_ok && tracking_ok) {
        printf("✓ TEST PASSED\n");
    } else {
        printf("✗ TEST FAILED\n");
        if (!converged) printf("  - SQP did not converge\n");
        if (!collision_ok) printf("  - Collision violations detected\n");
        if (!tracking_ok) printf("  - Poor tracking performance\n");
    }
    
    printf("\n================================================================================\n\n");
    
    // Cleanup
    OSQPCscMatrix_free(P);
    free(settings);
    delete[] agents;
    
    return 0;
}
