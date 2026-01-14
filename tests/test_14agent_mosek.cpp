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
    printf("                    14-AGENT CENTRALIZED MPC WITH MOSEK\n");
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
    
    // Set cost weights (matching test_14agent.cpp)
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
    
    // Set reference trajectories (matching test_14agent.cpp)
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
    
    // Problem dimensions
    int nV_agent = (N+1)*nx + N*nu;  // 124 per agent
    int nV = NUM_AGENTS * nV_agent;  // 1736 total variables
    int nC_dynamics = NUM_AGENTS * N * nx;  // 1120 dynamics constraints
    int num_collision_pairs = 91;  // 14 choose 2
    int nC_collision = num_collision_pairs * (N + 1);  // 1911 collision constraints
    int nC = nC_dynamics + nC_collision;  // 3031 total constraints
    
    printf("Problem dimensions:\n");
    printf("  Variables: %d\n", nV);
    printf("  Dynamics constraints: %d\n", nC_dynamics);
    printf("  Collision constraints: %d (91 pairs)\n", nC_collision);
    printf("  Total constraints: %d\n\n", nC);
    
    // Collision pair indices
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
    
    // Initialize trajectory with staggered start
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
    
    // SQP loop
    int max_sqp_iter = 50;
    real_t sqp_tol = 5e-3;
    bool converged = false;
    int final_iter = 0;
    real_t previous_objective = 1e10;
    
    double total_time = 0.0;
    double total_start = getTime();

    for (int sqp_iter = 0; sqp_iter < max_sqp_iter; ++sqp_iter) {
        MSKrescodee r;
        MSKenv_t env = NULL;
        MSKtask_t task = NULL;
        
        r = MSK_makeenv(&env, NULL);
        if (r != MSK_RES_OK) {
            printf("Failed to create MOSEK environment\n");
            return 1;
        }
        
        r = MSK_maketask(env, nC, nV, &task);
        if (r != MSK_RES_OK) {
            printf("Failed to create MOSEK task\n");
            MSK_deleteenv(&env);
            return 1;
        }
        
        // Disable verbose output
        // r = MSK_linkfunctotaskstream(task, MSK_STREAM_LOG, NULL, printstr);
        
        // Append variables and constraints
        r = MSK_appendvars(task, nV);
        r = MSK_appendcons(task, nC);
        
        // Set objective (quadratic)
        for (int agent_idx = 0; agent_idx < NUM_AGENTS; ++agent_idx) {
            int offset = agent_idx * nV_agent;
            
            // State costs
            for (int k = 0; k <= N; ++k) {
                for (int j = 0; j < nx; ++j) {
                    int var_idx = offset + k*(nx+nu) + j;
                    real_t q_val = agents[agent_idx].Q_diag[j];
                    r = MSK_putqobjij(task, var_idx, var_idx, q_val);
                    
                    real_t c_val = -q_val * agents[agent_idx].x_ref[k*nx + j];
                    r = MSK_putcj(task, var_idx, c_val);
                }
            }
            
            // Control costs
            for (int k = 0; k < N; ++k) {
                for (int j = 0; j < nu; ++j) {
                    int var_idx = offset + k*(nx+nu) + nx + j;
                    real_t r_val = agents[agent_idx].R_diag[j];
                    r = MSK_putqobjij(task, var_idx, var_idx, r_val);
                    
                    real_t c_val = -r_val * agents[agent_idx].u_ref[k*nu + j];
                    r = MSK_putcj(task, var_idx, c_val);
                }
            }
        }
        
        r = MSK_putobjsense(task, MSK_OBJECTIVE_SENSE_MINIMIZE);
        
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
        
        // Add collision avoidance constraints (linearized)
        for (int pair = 0; pair < num_collision_pairs; ++pair) {
            int i0 = pair_indices[pair][0];
            int i1 = pair_indices[pair][1];
            
            for (int k = 0; k <= N; ++k) {
                int idx0 = i0 * nV_agent + k * (nx + nu);
                int idx1 = i1 * nV_agent + k * (nx + nu);
                
                real_t x0 = current_trajectory[idx0];
                real_t y0 = current_trajectory[idx0 + 1];
                real_t x1 = current_trajectory[idx1];
                real_t y1 = current_trajectory[idx1 + 1];
                
                real_t dx = x0 - x1;
                real_t dy = y0 - y1;
                real_t dist = sqrt(dx*dx + dy*dy);
                
                if (dist < 1e-6) dist = 1e-6;
                
                std::vector<MSKint32t> asub_row;
                std::vector<MSKrealt> aval_row;
                
                real_t grad_x0 = dx / dist;
                real_t grad_y0 = dy / dist;
                real_t grad_x1 = -dx / dist;
                real_t grad_y1 = -dy / dist;
                
                asub_row.push_back(idx0);
                aval_row.push_back(grad_x0);
                asub_row.push_back(idx0 + 1);
                aval_row.push_back(grad_y0);
                asub_row.push_back(idx1);
                aval_row.push_back(grad_x1);
                asub_row.push_back(idx1 + 1);
                aval_row.push_back(grad_y1);
                
                r = MSK_putarow(task, con_idx, asub_row.size(), asub_row.data(), aval_row.data());
                
                real_t rhs = d_safe_constraint - dist + grad_x0 * x0 + grad_y0 * y0 + grad_x1 * x1 + grad_y1 * y1;
                r = MSK_putconbound(task, con_idx, MSK_BK_LO, rhs, MSK_INFINITY);
                con_idx++;
            }
        }
        
        // Optimize
        double t_start = getTime();
        r = MSK_optimizetrm(task, NULL);
        double t_end = getTime();
        total_time += (t_end - t_start);
        
        // Get solution
        std::vector<real_t> xx(nV);
        r = MSK_getxx(task, MSK_SOL_ITR, xx.data());
        
        // Compute objective
        real_t current_objective = 0.0;
        for (int agent_idx = 0; agent_idx < NUM_AGENTS; ++agent_idx) {
            int offset = agent_idx * nV_agent;
            for (int k = 0; k <= N; ++k) {
                for (int j = 0; j < nx; ++j) {
                    int idx = offset + k*(nx+nu) + j;
                    real_t diff = xx[idx] - agents[agent_idx].x_ref[k*nx + j];
                    current_objective += agents[agent_idx].Q_diag[j] * diff * diff;
                }
            }
            for (int k = 0; k < N; ++k) {
                for (int j = 0; j < nu; ++j) {
                    int idx = offset + k*(nx+nu) + nx + j;
                    real_t diff = xx[idx] - agents[agent_idx].u_ref[k*nu + j];
                    current_objective += agents[agent_idx].R_diag[j] * diff * diff;
                }
            }
        }
        current_objective *= 0.5;
        
        // Check convergence
        real_t max_delta = 0.0;
        for (int i = 0; i < nV; ++i) {
            real_t delta = fabs(xx[i] - current_trajectory[i]);
            if (delta > max_delta) max_delta = delta;
            current_trajectory[i] = xx[i];
        }
        
        real_t obj_change = fabs(current_objective - previous_objective);
        real_t rel_obj_change = (fabs(previous_objective) > 1e-10) ? 
                                 obj_change / fabs(previous_objective) : obj_change;
        
        printf("SQP iter %3d: obj=%.2f, max_delta=%.6f, obj_change=%.2e\n",
               sqp_iter + 1, current_objective, max_delta, rel_obj_change);
        
        final_iter = sqp_iter + 1;
        if (rel_obj_change < sqp_tol) {
            converged = true;
            previous_objective = current_objective;
            printf("Converged at iteration %d!\n", final_iter);
            MSK_deletetask(&task);
            MSK_deleteenv(&env);
            break;
        }
        
        previous_objective = current_objective;
        MSK_deletetask(&task);
        MSK_deleteenv(&env);
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
    
    for (int pair = 0; pair < num_collision_pairs; ++pair) {
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
    printf("  Violations: %d/%d stages\n\n", total_violations, (N+1)*num_collision_pairs);
    
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
    
    delete[] agents;
    
    return 0;
}
