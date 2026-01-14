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
    
    printf("================================================================================\n");
    printf("                    4-AGENT CENTRALIZED MPC WITH MOSEK\n");
    printf("================================================================================\n\n");
    
    int N = 20;
    int nx = 4;
    int nu = 2;
    real_t dt = 0.2;
    real_t v_max = 10.0;
    real_t d_safe = 2.0;
    real_t d_safe_margin = 0.05;
    real_t d_safe_constraint = d_safe + d_safe_margin;
    
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
    
    real_t Q_pos = 2.0;
    real_t Q_vel = 0.01;
    real_t R_ctrl = 0.1;
    
    for (int i = 0; i < 4; ++i) {
        agents[i].Q[0*nx + 0] = Q_pos;
        agents[i].Q[1*nx + 1] = Q_pos;
        agents[i].Q[2*nx + 2] = Q_vel;
        agents[i].Q[3*nx + 3] = Q_vel;
        
        agents[i].R[0*nu + 0] = R_ctrl;
        agents[i].R[1*nu + 1] = R_ctrl;
        
        agents[i].extractDiagonals();
    }
    
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
    
    int nV_agent = (N+1)*nx + N*nu;
    int nV = 4 * nV_agent;
    int n_dyn = 4 * N * nx;
    int n_collision = 6 * (N + 1);  // 6 pairs, N+1 timesteps each
    int numcon = n_dyn + n_collision;  // Fixed problem size
    
    printf("Problem dimensions:\n");
    printf("  Variables: %d\n", nV);
    printf("  Dynamics constraints: %d\n", n_dyn);
    printf("  Collision constraints: %d\n\n", n_collision);
    
    // Agent pairs for collision avoidance (6 pairs)
    std::vector<std::pair<int,int>> agent_pairs = {{0,1}, {0,2}, {0,3}, {1,2}, {1,3}, {2,3}};
    
    // SQP parameters
    const int max_sqp_iter = 300;
    const real_t constraint_tol = 2e-3;
    const real_t objective_tol = 2e-3;
    
    std::vector<real_t> current_trajectory(nV, 0.0);
    real_t previous_objective = 1e30;
    
    // Initialize with reference trajectory
    for (int agent_idx = 0; agent_idx < 4; ++agent_idx) {
        int offset = agent_idx * nV_agent;
        for (int k = 0; k <= N; ++k) {
            for (int j = 0; j < nx; ++j) {
                current_trajectory[offset + k*(nx+nu) + j] = agents[agent_idx].x_ref[k*nx + j];
            }
        }
    }
    
    // Pre-compute constant quadratic objective (Hessian and linear cost)
    // This is constant across all SQP iterations
    std::vector<MSKint32t> qsubi, qsubj;
    std::vector<MSKrealt> qval;
    std::vector<MSKrealt> c(nV, 0.0);
    
    for (int agent_idx = 0; agent_idx < 4; ++agent_idx) {
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
    
    bool converged = false;
    int final_iter = 0;
    
    printf("Starting SQP iterations...\n\n");
    double total_start = getTime();

    for (int sqp_iter = 0; sqp_iter < max_sqp_iter; ++sqp_iter) {
        // Initialize MOSEK with fixed problem size
        MSKenv_t env = NULL;
        MSKtask_t task = NULL;
        MSKrescodee r = MSK_RES_OK;
        
        r = MSK_makeenv(&env, NULL);
        r = MSK_maketask(env, numcon, nV, &task);
        // r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);  // Disabled for performance
        
        r = MSK_appendvars(task, nV);
        r = MSK_appendcons(task, numcon);
        
        // Set up quadratic objective (constant, pre-computed)
        r = MSK_putqobj(task, qsubi.size(), qsubi.data(), qsubj.data(), qval.data());
        for (int j = 0; j < nV; ++j) {
            r = MSK_putcj(task, j, c[j]);
        }
        
        // Set variable bounds
        for (int agent_idx = 0; agent_idx < 4; ++agent_idx) {
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
        
        // Add dynamics constraints
        int con_idx = 0;
        for (int agent_idx = 0; agent_idx < 4; ++agent_idx) {
            int offset = agent_idx * nV_agent;
            
            for (int k = 0; k < N; ++k) {
                for (int i = 0; i < nx; ++i) {
                    std::vector<MSKint32t> asub;
                    std::vector<MSKrealt> aval;
                    
                    asub.push_back(offset + (k+1)*(nx+nu) + i);
                    aval.push_back(1.0);
                    
                    for (int j = 0; j < nx; ++j) {
                        asub.push_back(offset + k*(nx+nu) + j);
                        aval.push_back(-agents[agent_idx].A[i*nx + j]);
                    }
                    
                    for (int j = 0; j < nu; ++j) {
                        asub.push_back(offset + k*(nx+nu) + nx + j);
                        aval.push_back(-agents[agent_idx].B[i*nu + j]);
                    }
                    
                    r = MSK_putarow(task, con_idx, asub.size(), asub.data(), aval.data());
                    r = MSK_putconbound(task, con_idx, MSK_BK_FX, 0.0, 0.0);
                    con_idx++;
                }
            }
        }
        
        // Add collision constraints with safe default normals (match OSQP/6-agent approach)
        int num_active_collisions = 0;
        
        for (const auto& pair : agent_pairs) {
            int i1 = pair.first;
            int i2 = pair.second;
            int offset1 = i1 * nV_agent;
            int offset2 = i2 * nV_agent;
            
            for (int k = 0; k <= N; ++k) {
                int idx = k * (nx + nu);
                real_t x1 = current_trajectory[offset1 + idx];
                real_t y1 = current_trajectory[offset1 + idx + 1];
                real_t x2 = current_trajectory[offset2 + idx];
                real_t y2 = current_trajectory[offset2 + idx + 1];
                real_t dx = x1 - x2;
                real_t dy = y1 - y2;
                real_t dist = sqrt(dx*dx + dy*dy);
                
                // Use safe default normal if agents are too close (match OSQP)
                real_t nx_c, ny_c;
                if (dist > 1e-3) {
                    nx_c = dx / dist;
                    ny_c = dy / dist;
                } else {
                    // Use horizontal separation as default when trajectories overlap
                    nx_c = 1.0;
                    ny_c = 0.0;
                }
                
                // Always add constraint row with safe normal
                std::vector<MSKint32t> asub = {offset1+idx, offset1+idx+1, offset2+idx, offset2+idx+1};
                std::vector<MSKrealt> aval = {nx_c, ny_c, -nx_c, -ny_c};
                r = MSK_putarow(task, con_idx, asub.size(), asub.data(), aval.data());
                
                // Activate/deactivate based on distance (match OSQP threshold)
                if (dist > 0 && dist < d_safe_constraint * 3.0) {
                    num_active_collisions++;
                    r = MSK_putconbound(task, con_idx, MSK_BK_LO, d_safe_constraint, MSK_INFINITY);
                } else {
                    // Constraint inactive - set to free
                    r = MSK_putconbound(task, con_idx, MSK_BK_FR, -MSK_INFINITY, MSK_INFINITY);
                }
                con_idx++;
            }
        }
        
        // Solve QP
        r = MSK_optimizetrm(task, NULL);
        
        MSKsolstae solsta;
        r = MSK_getsolsta(task, MSK_SOL_ITR, &solsta);
        
        if (solsta != MSK_SOL_STA_OPTIMAL) {
            printf("SQP iteration %d: Solution not optimal (status=%d)\n", sqp_iter, solsta);
            MSK_deletetask(&task);
            MSK_deleteenv(&env);
            break;
        }
        
        // Get solution
        std::vector<MSKrealt> xx(nV);
        r = MSK_getxx(task, MSK_SOL_ITR, xx.data());
        
        for (int j = 0; j < nV; ++j) {
            current_trajectory[j] = xx[j];
        }
        
        // Compute objective
        real_t current_objective = 0.0;
        for (int j = 0; j < nV; ++j) {
            current_objective += c[j] * xx[j];
        }
        for (size_t i = 0; i < qsubi.size(); ++i) {
            current_objective += 0.5 * qval[i] * xx[qsubi[i]] * xx[qsubj[i]];
        }
        
        MSK_deletetask(&task);
        MSK_deleteenv(&env);
        
        // Compute constraint violations
        real_t max_constraint_viol = 0.0;
        real_t min_dist_iter = 1e10;
        
        for (const auto& pair : agent_pairs) {
            int i1 = pair.first;
            int i2 = pair.second;
            int offset1 = i1 * nV_agent;
            int offset2 = i2 * nV_agent;
            
            for (int k = 0; k <= N; ++k) {
                int idx = k * (nx + nu);
                real_t x1 = current_trajectory[offset1 + idx];
                real_t y1 = current_trajectory[offset1 + idx + 1];
                real_t x2 = current_trajectory[offset2 + idx];
                real_t y2 = current_trajectory[offset2 + idx + 1];
                real_t dist = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
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
            printf("SQP iter %d: obj=%.6f, min_dist=%.3f m, max_viol=%.3e, active=%d\n",
                   sqp_iter, current_objective, min_dist_iter, max_constraint_viol, num_active_collisions);
        }
        
        // Check convergence
        final_iter = sqp_iter + 1;
        if (max_constraint_viol <= 0.0 && rel_obj_change < objective_tol) {
            converged = true;
            previous_objective = current_objective;
            break;
        }
        
        previous_objective = current_objective;
    }
    double total_end = getTime();

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
        int offset = i * nV_agent;
        printf("Agent %d: (%.2f, %.2f, %.2f, %.2f)\n", i,
               current_trajectory[offset + idx_N],
               current_trajectory[offset + idx_N + 1],
               current_trajectory[offset + idx_N + 2],
               current_trajectory[offset + idx_N + 3]);
    }
    
    // Verify collision constraints
    real_t min_achieved_dist = 1e10;
    for (const auto& pair : agent_pairs) {
        int i1 = pair.first;
        int i2 = pair.second;
        int offset1 = i1 * nV_agent;
        int offset2 = i2 * nV_agent;
        
        for (int k = 0; k <= N; ++k) {
            real_t x1 = current_trajectory[offset1 + k*(nx+nu)];
            real_t y1 = current_trajectory[offset1 + k*(nx+nu) + 1];
            real_t x2 = current_trajectory[offset2 + k*(nx+nu)];
            real_t y2 = current_trajectory[offset2 + k*(nx+nu) + 1];
            real_t dist = sqrt((x1-x2)*(x1-x2) + (y1-y2)*(y1-y2));
            if (dist < min_achieved_dist) min_achieved_dist = dist;
        }
    }
    
    printf("\nMinimum achieved distance: %.3f m (required: %.1f m)\n", min_achieved_dist, d_safe);
    if (min_achieved_dist >= d_safe) {
        printf("Collision constraints satisfied.\n");
    } else {
        printf("Collision constraints violated!\n");
    }
    
    // Calculate tracking errors
    real_t targets_x[4] = {15.0, 0.0, 7.25, 7.75};
    real_t targets_y[4] = {4.8, 5.3, 15.0, 0.0};
    real_t tracking_errors[4];
    
    printf("\nTracking errors:\n");
    for (int i = 0; i < 4; ++i) {
        int offset = i * nV_agent;
        real_t x_final = current_trajectory[offset + idx_N];
        real_t y_final = current_trajectory[offset + idx_N + 1];
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
    
    // Cleanup
    delete[] agents;
    
    printf("\nSolve time: %.3f ms\n", total_end - total_start);
    
    return 0;
}
