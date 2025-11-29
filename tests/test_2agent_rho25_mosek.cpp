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
    printf("                    2-AGENT CENTRALIZED MPC WITH MOSEK\n");
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
    
    real_t Q_pos = 2.0;
    real_t Q_vel = 0.01;
    real_t R_ctrl = 0.1;
    
    for (int i = 0; i < 2; ++i) {
        agents[i].Q[0*nx + 0] = Q_pos;
        agents[i].Q[1*nx + 1] = Q_pos;
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
    
    // Compute minimum reference distance
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
    
    int nV_agent = (N+1)*nx + N*nu;  // 124
    int nV = 2 * nV_agent;  // 248
    int n_dyn = 2 * N * nx;  // 160
    int n_collision = N + 1;  // 21 collision constraints (1 pair, N+1 timesteps)
    int numcon = n_dyn + n_collision;  // Fixed problem size
    
    printf("Problem dimensions:\n");
    printf("  Variables: %d\n", nV);
    printf("  Dynamics constraints: %d\n", n_dyn);
    printf("  Collision constraints: %d\n\n", n_collision);
    
    // SQP parameters
    const int max_sqp_iter = 300;
    const real_t constraint_tol = 2e-3;
    const real_t objective_tol = 2e-3;
    
    std::vector<real_t> current_trajectory(nV, 0.0);
    real_t previous_objective = 1e30;
    
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
    
    printf("Starting SQP iterations...\n\n");
    
    for (int sqp_iter = 0; sqp_iter < max_sqp_iter; ++sqp_iter) {
        // Initialize MOSEK with fixed problem size
        MSKenv_t env = NULL;
        MSKtask_t task = NULL;
        MSKrescodee r = MSK_RES_OK;
        
        r = MSK_makeenv(&env, NULL);
        r = MSK_maketask(env, numcon, nV, &task);
        r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);
        
        // Append variables and constraints
        r = MSK_appendvars(task, nV);
        r = MSK_appendcons(task, numcon);
        
        // Set up quadratic objective
        std::vector<MSKint32t> qsubi, qsubj;
        std::vector<MSKrealt> qval;
        std::vector<MSKrealt> c(nV, 0.0);
        
        for (int agent_idx = 0; agent_idx < 2; ++agent_idx) {
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
        
        // Set variable bounds (including velocity limits and initial state fixing)
        for (int j = 0; j < nV; ++j) {
            real_t lb_val = -MSK_INFINITY;
            real_t ub_val = MSK_INFINITY;
            
            if (j < nV_agent) {
                int local_j = j;
                int k = local_j / (nx + nu);
                int rem = local_j % (nx + nu);
                
                if (rem == 2 || rem == 3) {  // vx or vy
                    lb_val = -v_max;
                    ub_val = v_max;
                }
                
                // Fix initial state
                if (k == 0 && rem < nx) {
                    lb_val = agents[0].x_ref[0*nx + rem];
                    ub_val = agents[0].x_ref[0*nx + rem];
                }
            } else {
                int local_j = j - nV_agent;
                int k = local_j / (nx + nu);
                int rem = local_j % (nx + nu);
                
                if (rem == 2 || rem == 3) {  // vx or vy
                    lb_val = -v_max;
                    ub_val = v_max;
                }
                
                // Fix initial state
                if (k == 0 && rem < nx) {
                    lb_val = agents[1].x_ref[0*nx + rem];
                    ub_val = agents[1].x_ref[0*nx + rem];
                }
            }
            
            r = MSK_putvarbound(task, j, MSK_BK_RA, lb_val, ub_val);
        }
        
        // Add dynamics constraints (equality)
        int con_idx = 0;
        for (int agent_idx = 0; agent_idx < 2; ++agent_idx) {
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
        real_t d_safe_margin = 0.05;
        real_t d_safe_constraint = d_safe + d_safe_margin;
        int num_active_collisions = 0;
        
        for (int k = 0; k <= N; ++k) {
            int idx = k * (nx + nu);
            real_t p0x = current_trajectory[idx];
            real_t p0y = current_trajectory[idx + 1];
            real_t p1x = current_trajectory[nV_agent + idx];
            real_t p1y = current_trajectory[nV_agent + idx + 1];
            real_t dx = p0x - p1x;
            real_t dy = p0y - p1y;
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
            std::vector<MSKint32t> asub = {idx, idx+1, nV_agent+idx, nV_agent+idx+1};
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
    delete[] z0;
    delete[] z1;
    delete[] agents;
    
    double total_end = getTime();
    printf("\nTotal execution time: %.3f ms\n", total_end - total_start);
    
    return 0;
}
