#include <stdlib.h>
#include <mosek.h>
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

static void MSKAPI printstr(void *handle, const char str[]) {
    printf("%s", str);
}

int main()
{
    double total_start = getTime();
    
    printf("================================================================================\n");
    printf("                    8-AGENT CENTRALIZED MPC WITH MOSEK\n");
    printf("================================================================================\n\n");
    
    const int NUM_AGENTS = 8;
    int N = 20;
    int nx = 4;
    int nu = 2;
    real_t dt = 0.2;
    real_t v_max = 10.0;
    real_t d_safe = 2.0;
    real_t d_safe_margin = 0.05;
    real_t d_safe_constraint = d_safe + d_safe_margin;
    
    // Create 8 PointMass agents
    PointMass agent0(N, dt, 10.0, v_max);
    PointMass agent1(N, dt, 10.0, v_max);
    PointMass agent2(N, dt, 10.0, v_max);
    PointMass agent3(N, dt, 10.0, v_max);
    PointMass agent4(N, dt, 10.0, v_max);
    PointMass agent5(N, dt, 10.0, v_max);
    PointMass agent6(N, dt, 10.0, v_max);
    PointMass agent7(N, dt, 10.0, v_max);
    
    AgentData* agents = new AgentData[NUM_AGENTS];
    agents[0] = agent0;
    agents[1] = agent1;
    agents[2] = agent2;
    agents[3] = agent3;
    agents[4] = agent4;
    agents[5] = agent5;
    agents[6] = agent6;
    agents[7] = agent7;
    
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
    
    // Agent 0: (0, 4.8) -> (15, 4.8) - horizontal right
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
    
    // Agent 1: (15, 5.3) -> (0, 5.3) - horizontal left
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
    
    // Agent 2: (7.25, 0) -> (7.25, 15) - vertical up
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
    
    // Agent 3: (7.75, 15) -> (7.75, 0) - vertical down
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
    
    // Agent 4: (0, 0) -> (15, 15) - diagonal ↗
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
    
    // Agent 5: (0, 15) -> (15, 0) - diagonal ↘
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
    
    // Agent 6: (15, 15) -> (0, 0) - diagonal ↙
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
    
    // Agent 7: (15, 0) -> (0, 15) - diagonal ↖
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
    
    printf("Agent Configuration:\n");
    printf("  Agent 0: (0, 4.8) -> (15, 4.8) [horizontal right]\n");
    printf("  Agent 1: (15, 5.3) -> (0, 5.3) [horizontal left]\n");
    printf("  Agent 2: (7.25, 0) -> (7.25, 15) [vertical up]\n");
    printf("  Agent 3: (7.75, 15) -> (7.75, 0) [vertical down]\n");
    printf("  Agent 4: (0, 0) -> (15, 15) [diagonal ↗]\n");
    printf("  Agent 5: (0, 15) -> (15, 0) [diagonal ↘]\n");
    printf("  Agent 6: (15, 15) -> (0, 0) [diagonal ↙]\n");
    printf("  Agent 7: (15, 0) -> (0, 15) [diagonal ↖]\n");
    printf("Time horizon: N=%d, dt=%.1f s, total time=%.1f s\n", N, dt, N*dt);
    printf("Velocity bounds: -%.1f m/s <= v <= %.1f m/s\n", v_max, v_max);
    printf("Safety distance: %.1f m\n\n", d_safe);
    
    int nV_agent = (N+1)*nx + N*nu;
    int nV = NUM_AGENTS * nV_agent;
    int n_dyn = NUM_AGENTS * N * nx;
    int n_collision = 28 * (N+1);  // 28 pairs for 8 agents, N+1 timesteps each
    int numcon = n_dyn + n_collision;
    
    printf("Problem dimensions:\n");
    printf("  Variables: %d\n", nV);
    printf("  Dynamics constraints: %d\n", n_dyn);
    printf("  Collision constraints: %d\n\n", n_collision);
    
    // Agent pairs for collision avoidance (28 pairs for 8 agents)
    int pair_indices[28][2] = {
        {0,1}, {0,2}, {0,3}, {0,4}, {0,5}, {0,6}, {0,7},
        {1,2}, {1,3}, {1,4}, {1,5}, {1,6}, {1,7},
        {2,3}, {2,4}, {2,5}, {2,6}, {2,7},
        {3,4}, {3,5}, {3,6}, {3,7},
        {4,5}, {4,6}, {4,7},
        {5,6}, {5,7},
        {6,7}
    };
    
    // SQP parameters
    const int max_sqp_iter = 300;
    const real_t constraint_tol = 2e-3;
    const real_t objective_tol = 2e-3;
    
    std::vector<real_t> current_trajectory(nV, 0.0);
    real_t previous_objective = 1e30;
    
    // Initialize with collision-free trajectory by making agents wait at start
    // This avoids the infeasible initial condition where all agents collide at center
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
    
    bool converged = false;
    int final_iter = 0;
    
    printf("Starting SQP iterations...\n\n");
    
    for (int sqp_iter = 0; sqp_iter < max_sqp_iter; ++sqp_iter) {
        // Initialize MOSEK
        MSKenv_t env = NULL;
        MSKtask_t task = NULL;
        MSKrescodee r = MSK_RES_OK;
        
        r = MSK_makeenv(&env, NULL);
        r = MSK_maketask(env, numcon, nV, &task);
        r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);
        
        r = MSK_appendvars(task, nV);
        r = MSK_appendcons(task, numcon);
        
        // Set up quadratic objective
        std::vector<MSKint32t> qsubi, qsubj;
        std::vector<MSKrealt> qval;
        std::vector<MSKrealt> c(nV, 0.0);
        
        for (int agent_idx = 0; agent_idx < NUM_AGENTS; ++agent_idx) {
            int offset = agent_idx * nV_agent;
            
            for (int k = 0; k <= N; ++k) {
                for (int j = 0; j < nx; ++j) {
                    int var_idx = offset + k * (nx + nu) + j;
                    MSKrealt q_val = agents[agent_idx].Q[j*nx + j];
                    qsubi.push_back(var_idx);
                    qsubj.push_back(var_idx);
                    qval.push_back(q_val);
                    c[var_idx] = -q_val * agents[agent_idx].x_ref[k*nx + j];
                }
                
                if (k < N) {
                    for (int j = 0; j < nu; ++j) {
                        int var_idx = offset + k * (nx + nu) + nx + j;
                        MSKrealt r_val = agents[agent_idx].R[j*nu + j];
                        qsubi.push_back(var_idx);
                        qsubj.push_back(var_idx);
                        qval.push_back(r_val);
                        c[var_idx] = -r_val * agents[agent_idx].u_ref[k*nu + j];
                    }
                }
            }
        }
        
        r = MSK_putqobj(task, qsubi.size(), qsubi.data(), qsubj.data(), qval.data());
        for (int j = 0; j < nV; ++j) {
            r = MSK_putcj(task, j, c[j]);
        }
        
        // Set variable bounds
        for (int agent_idx = 0; agent_idx < NUM_AGENTS; ++agent_idx) {
            int offset = agent_idx * nV_agent;
            
            for (int k = 0; k <= N; ++k) {
                for (int j = 0; j < nx; ++j) {
                    int var_idx = offset + k*(nx+nu) + j;
                    real_t lb_val = -MSK_INFINITY;
                    real_t ub_val = MSK_INFINITY;
                    
                    if (j == 2 || j == 3) {  // vx or vy
                        lb_val = -v_max;
                        ub_val = v_max;
                    }
                    
                    // Fix initial state
                    if (k == 0) {
                        lb_val = agents[agent_idx].x_ref[0*nx + j];
                        ub_val = agents[agent_idx].x_ref[0*nx + j];
                    }
                    
                    r = MSK_putvarbound(task, var_idx, MSK_BK_RA, lb_val, ub_val);
                }
                
                // Control variables (unbounded)
                if (k < N) {
                    for (int j = 0; j < nu; ++j) {
                        int var_idx = offset + k*(nx+nu) + nx + j;
                        r = MSK_putvarbound(task, var_idx, MSK_BK_FR, -MSK_INFINITY, MSK_INFINITY);
                    }
                }
            }
        }
        
        // Add dynamics constraints (equality)
        int con_idx = 0;
        for (int agent_idx = 0; agent_idx < NUM_AGENTS; ++agent_idx) {
            int offset = agent_idx * nV_agent;
            
            for (int k = 0; k < N; ++k) {
                for (int i = 0; i < nx; ++i) {
                    std::vector<MSKint32t> asub_row;
                    std::vector<MSKrealt> aval_row;
                    
                    asub_row.push_back(offset + (k+1)*(nx+nu) + i);
                    aval_row.push_back(1.0);
                    
                    for (int j = 0; j < nx; ++j) {
                        asub_row.push_back(offset + k*(nx+nu) + j);
                        aval_row.push_back(-agents[agent_idx].A[i*nx + j]);
                    }
                    
                    for (int j = 0; j < nu; ++j) {
                        asub_row.push_back(offset + k*(nx+nu) + nx + j);
                        aval_row.push_back(-agents[agent_idx].B[i*nu + j]);
                    }
                    
                    r = MSK_putarow(task, con_idx, asub_row.size(), asub_row.data(), aval_row.data());
                    r = MSK_putconbound(task, con_idx, MSK_BK_FX, 0.0, 0.0);
                    con_idx++;
                }
            }
        }
        
        // Add collision constraints (linearized)
        int num_active_collisions = 0;
        for (int pair = 0; pair < 28; ++pair) {
            int i0 = pair_indices[pair][0];
            int i1 = pair_indices[pair][1];
            
            for (int k = 0; k <= N; ++k) {
                int idx0 = i0 * nV_agent + k * (nx + nu);
                int idx1 = i1 * nV_agent + k * (nx + nu);
                real_t p0x = current_trajectory[idx0];
                real_t p0y = current_trajectory[idx0 + 1];
                real_t p1x = current_trajectory[idx1];
                real_t p1y = current_trajectory[idx1 + 1];
                real_t dx = p0x - p1x;
                real_t dy = p0y - p1y;
                real_t dist = sqrt(dx*dx + dy*dy);
                
                // Use safe default normal if agents are too close
                real_t nx_c, ny_c;
                if (dist > 1e-3) {
                    nx_c = dx / dist;
                    ny_c = dy / dist;
                } else {
                    nx_c = 1.0;
                    ny_c = 0.0;
                }
                
                // Always add constraint row with safe normal
                std::vector<MSKint32t> asub_row = {idx0, idx0+1, idx1, idx1+1};
                std::vector<MSKrealt> aval_row = {nx_c, ny_c, -nx_c, -ny_c};
                r = MSK_putarow(task, con_idx, asub_row.size(), asub_row.data(), aval_row.data());
                
                // Activate/deactivate based on distance
                if (dist > 0 && dist < d_safe_constraint * 3.0) {
                    num_active_collisions++;
                    r = MSK_putconbound(task, con_idx, MSK_BK_LO, d_safe_constraint, MSK_INFINITY);
                } else {
                    r = MSK_putconbound(task, con_idx, MSK_BK_FR, -MSK_INFINITY, MSK_INFINITY);
                }
                con_idx++;
            }
        }
        
        // Optimize
        MSK_putintparam(task, MSK_IPAR_LOG, 0);
        r = MSK_optimizetrm(task, NULL);
        
        if (r != MSK_RES_OK) {
            printf("MOSEK optimization failed with code %d\n", r);
            MSK_deletetask(&task);
            MSK_deleteenv(&env);
            return 1;
        }
        
        MSKsolstae solsta;
        MSK_getsolsta(task, MSK_SOL_ITR, &solsta);
        
        if (solsta != MSK_SOL_STA_OPTIMAL) {
            printf("MOSEK solution not optimal (status: %d)\n", solsta);
        }
        
        // Get solution
        std::vector<MSKrealt> xx(nV);
        MSK_getxx(task, MSK_SOL_ITR, xx.data());
        
        // Update trajectory
        for (int i = 0; i < nV; ++i) {
            current_trajectory[i] = xx[i];
        }
        
        // Compute objective
        real_t current_objective = 0.0;
        for (int i = 0; i < nV; ++i) {
            current_objective += c[i] * xx[i];
        }
        for (size_t i = 0; i < qsubi.size(); ++i) {
            current_objective += 0.5 * qval[i] * xx[qsubi[i]] * xx[qsubj[i]];
        }
        
        MSK_deletetask(&task);
        MSK_deleteenv(&env);
        
        // Compute constraint violations
        real_t max_constraint_viol = 0.0;
        real_t min_dist_iter = 1e10;
        
        for (int pair = 0; pair < 28; ++pair) {
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
        
        if (sqp_iter % 10 == 0 || sqp_iter < 5) {
            printf("SQP iter %3d: obj=%10.2f, min_dist=%.3f m, max_viol=%.4f m, active=%d\n",
                   sqp_iter + 1, current_objective, min_dist_iter, max_constraint_viol, num_active_collisions);
        }
        
        // Check convergence
        final_iter = sqp_iter + 1;
        if (max_constraint_viol <= constraint_tol && rel_obj_change < objective_tol) {
            converged = true;
            previous_objective = current_objective;
            break;
        }
        
        previous_objective = current_objective;
    }
    
    printf("\n================================================================================\n");
    printf("                         SQP SUMMARY\n");
    printf("================================================================================\n");
    printf("Total SQP iterations: %d\n", final_iter);
    printf("Converged: %s\n\n", converged ? "YES" : "NO (max iterations)");
    
    // Print terminal states
    int idx_N = N * (nx + nu);
    printf("Terminal States:\n");
    for (int i = 0; i < NUM_AGENTS; ++i) {
        int offset = i * nV_agent;
        printf("Agent %d: (%.2f, %.2f, %.2f, %.2f)\n", i,
               current_trajectory[offset + idx_N],
               current_trajectory[offset + idx_N + 1],
               current_trajectory[offset + idx_N + 2],
               current_trajectory[offset + idx_N + 3]);
    }
    
    // Verify collision constraints
    printf("\nMinimum distances between agent pairs:\n");
    real_t global_min_dist = 1e10;
    for (int pair = 0; pair < 28; ++pair) {
        int i0 = pair_indices[pair][0];
        int i1 = pair_indices[pair][1];
        
        real_t min_dist_pair = 1e10;
        for (int k = 0; k <= N; ++k) {
            int idx0 = i0 * nV_agent + k * (nx + nu);
            int idx1 = i1 * nV_agent + k * (nx + nu);
            real_t x0 = current_trajectory[idx0];
            real_t y0 = current_trajectory[idx0 + 1];
            real_t x1 = current_trajectory[idx1];
            real_t y1 = current_trajectory[idx1 + 1];
            real_t dist = sqrt((x0-x1)*(x0-x1) + (y0-y1)*(y0-y1));
            if (dist < min_dist_pair) min_dist_pair = dist;
        }
        printf("  Agents %d-%d: %.3f m\n", i0, i1, min_dist_pair);
        if (min_dist_pair < global_min_dist) global_min_dist = min_dist_pair;
    }
    
    printf("\nGlobal minimum distance: %.3f m (required: %.1f m)\n", global_min_dist, d_safe);
    if (global_min_dist >= d_safe) {
        printf("All collision constraints satisfied.\n");
    } else {
        printf("Collision constraints violated!\n");
    }
    
    // Cleanup
    delete[] agents;
    
    double total_end = getTime();
    printf("\nTotal execution time: %.3f ms\n", total_end - total_start);
    
    return 0;
}
