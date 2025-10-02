/*
 *  Horizon Sweep Benchmark: Performance vs Planning Horizon
 *  
 *  Compares vanilla vs MPC-aware qpOASES performance across
 *  different planning horizons: N = 5, 10, 30, 50, 70, 100
 *  
 *  Outputs CSV data for plotting and verifies solution accuracy.
 */

#include "utils/test_problems.hpp"
#include <iostream>
#include <chrono>
#include <cmath>
#include <fstream>

USING_NAMESPACE_QPOASES

// ============================================================================
// TIMING UTILITIES
// ============================================================================

class Timer {
private:
    std::chrono::high_resolution_clock::time_point start_time;
    
public:
    void start() {
        start_time = std::chrono::high_resolution_clock::now();
    }
    
    double elapsed_ms() {
        auto end_time = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end_time - start_time);
        return duration.count() / 1000.0;
    }
};

// ============================================================================
// SOLUTION COMPARISON
// ============================================================================

double computeSolutionError(const real_t* x1, const real_t* x2, int n) {
    double max_error = 0.0;
    for (int i = 0; i < n; ++i) {
        double error = fabs(x1[i] - x2[i]);
        if (error > max_error) max_error = error;
    }
    return max_error;
}

// ============================================================================
// BENCHMARK FUNCTION
// ============================================================================

struct HorizonBenchmarkResult {
    int N;
    double vanilla_time_ms;
    double mpc_time_ms;
    int vanilla_iterations;
    int mpc_iterations;
    double solution_error;
    bool success;
};

HorizonBenchmarkResult benchmarkHorizon(int N, int num_runs = 10) {
    using namespace RealWorldProblems;
    
    const int nx = QuadrotorHover::nx;
    const int nu = QuadrotorHover::nu;
    
    HorizonBenchmarkResult result;
    result.N = N;
    result.success = false;
    
    // Setup system matrices
    real_t* A = new real_t[nx*nx];
    real_t* B = new real_t[nx*nu];
    real_t* Q = new real_t[nx*nx];
    real_t* R = new real_t[nu*nu];
    
    QuadrotorHover::getSystemMatrices(A, B, 0.1);
    QuadrotorHover::getCostMatrices(Q, R);
    
    // Setup QP problem dimensions
    int nV = nx*N + nu*(N-1);
    int nC = (N-1)*nx;
    
    // Build QP matrices
    real_t* H = new real_t[nV*nV];
    real_t* g = new real_t[nV];
    real_t* A_qp = new real_t[nC*nV];
    real_t* lbA = new real_t[nC];
    real_t* ubA = new real_t[nC];
    
    // Initialize
    for (int i = 0; i < nV*nV; ++i) H[i] = 0.0;
    for (int i = 0; i < nV; ++i) g[i] = 0.0;
    for (int i = 0; i < nC*nV; ++i) A_qp[i] = 0.0;
    for (int i = 0; i < nC; ++i) { lbA[i] = 0.0; ubA[i] = 0.0; }
    
    // Build block-diagonal Hessian
    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nx; ++j) {
                H[(k*nx + i)*nV + (k*nx + j)] = Q[i*nx + j];
            }
        }
    }
    
    int u_offset = nx*N;
    for (int k = 0; k < N-1; ++k) {
        for (int i = 0; i < nu; ++i) {
            for (int j = 0; j < nu; ++j) {
                H[(u_offset + k*nu + i)*nV + (u_offset + k*nu + j)] = R[i*nu + j];
            }
        }
    }
    
    // Build dynamics constraint matrix
    for (int k = 0; k < N-1; ++k) {
        int row_offset = k * nx;
        
        // -A block (current state)
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nx; ++j) {
                A_qp[(row_offset + i)*nV + (k*nx + j)] = -A[i*nx + j];
            }
        }
        
        // I block (next state)
        for (int i = 0; i < nx; ++i) {
            A_qp[(row_offset + i)*nV + ((k+1)*nx + i)] = 1.0;
        }
        
        // -B block (input)
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nu; ++j) {
                A_qp[(row_offset + i)*nV + (u_offset + k*nu + j)] = -B[i*nu + j];
            }
        }
    }
    
    // Storage for solutions
    real_t* x_vanilla = new real_t[nV];
    real_t* x_mpc = new real_t[nV];
    
    // ========================================================================
    // BENCHMARK VANILLA qpOASES
    // ========================================================================
    
    double vanilla_total_time = 0.0;
    int vanilla_total_iter = 0;
    int vanilla_success_count = 0;
    
    for (int run = 0; run < num_runs; ++run) {
        QProblem qp_vanilla(nV, nC);
        
        Options options_vanilla;
        options_vanilla.enableMPCRiccati = BT_FALSE;
        options_vanilla.printLevel = PL_NONE;
        qp_vanilla.setOptions(options_vanilla);
        
        Timer timer;
        timer.start();
        
        int nWSR = 1000;
        returnValue ret = qp_vanilla.init(H, g, A_qp, NULL, NULL, lbA, ubA, nWSR);
        
        vanilla_total_time += timer.elapsed_ms();
        vanilla_total_iter += nWSR;
        
        if (ret == SUCCESSFUL_RETURN) {
            vanilla_success_count++;
            if (run == 0) {
                qp_vanilla.getPrimalSolution(x_vanilla);
            }
        }
    }
    
    // ========================================================================
    // BENCHMARK MPC-AWARE qpOASES
    // ========================================================================
    
    double mpc_total_time = 0.0;
    int mpc_total_iter = 0;
    int mpc_success_count = 0;
    
    for (int run = 0; run < num_runs; ++run) {
        QProblem qp_mpc(nV, nC);
        
        Options options_mpc;
        options_mpc.enableMPCRiccati = BT_TRUE;
        options_mpc.printLevel = PL_NONE;
        qp_mpc.setOptions(options_mpc);
        qp_mpc.setupMPCStructure(N, nx, nu, A, B, Q, R);
        
        Timer timer;
        timer.start();
        
        int nWSR = 1000;
        returnValue ret = qp_mpc.init(H, g, A_qp, NULL, NULL, lbA, ubA, nWSR);
        
        mpc_total_time += timer.elapsed_ms();
        mpc_total_iter += nWSR;
        
        if (ret == SUCCESSFUL_RETURN) {
            mpc_success_count++;
            if (run == 0) {
                qp_mpc.getPrimalSolution(x_mpc);
            }
        }
    }
    
    // ========================================================================
    // COMPUTE RESULTS
    // ========================================================================
    
    if (vanilla_success_count == num_runs && mpc_success_count == num_runs) {
        result.success = true;
        result.vanilla_time_ms = vanilla_total_time / num_runs;
        result.mpc_time_ms = mpc_total_time / num_runs;
        result.vanilla_iterations = vanilla_total_iter / num_runs;
        result.mpc_iterations = mpc_total_iter / num_runs;
        result.solution_error = computeSolutionError(x_vanilla, x_mpc, nV);
    }
    
    // Cleanup
    delete[] x_mpc;
    delete[] x_vanilla;
    delete[] ubA;
    delete[] lbA;
    delete[] A_qp;
    delete[] g;
    delete[] H;
    delete[] R;
    delete[] Q;
    delete[] B;
    delete[] A;
    
    return result;
}

// ============================================================================
// MAIN BENCHMARK
// ============================================================================

int main() {
    printf("\n");
    printf("========================================================\n");
    printf("   HORIZON SWEEP BENCHMARK\n");
    printf("   Performance vs Planning Horizon\n");
    printf("========================================================\n");
    printf("\n");
    
    printf("Problem: Quadrotor Hover Stabilization\n");
    printf("  States (nx):  12\n");
    printf("  Inputs (nu):  4\n");
    printf("  Testing horizons: 5, 10, 15, 20, 25, 30\n");
    printf("\n");
    
    int num_runs = 20;  // Run each horizon 20 times for statistics
    int horizons[] = {5, 10, 15, 20, 25, 30};
    int num_horizons = 6;
    
    printf("Running %d trials per horizon...\n", num_runs);
    printf("\n");
    
    // Run benchmarks
    HorizonBenchmarkResult* results = new HorizonBenchmarkResult[num_horizons];
    
    for (int i = 0; i < num_horizons; ++i) {
        int N = horizons[i];
        printf("Testing N=%d...", N);
        fflush(stdout);
        
        results[i] = benchmarkHorizon(N, num_runs);
        
        if (results[i].success) {
            printf(" ✓ (Speedup: %.1fx, Error: %.2e)\n", 
                   results[i].vanilla_time_ms / results[i].mpc_time_ms,
                   results[i].solution_error);
        } else {
            printf(" ✗ FAILED\n");
        }
    }
    
    printf("\n");
    printf("========================================================\n");
    printf("   RESULTS\n");
    printf("========================================================\n");
    printf("\n");
    
    // Print table
    printf("%-8s | %-12s | %-12s | %-10s | %-12s | %-12s\n",
           "Horizon", "Vanilla (ms)", "MPC (ms)", "Speedup", "Solution Err", "Iterations");
    printf("---------|--------------|--------------|------------|--------------|------------\n");
    
    for (int i = 0; i < num_horizons; ++i) {
        if (results[i].success) {
            double speedup = results[i].vanilla_time_ms / results[i].mpc_time_ms;
            printf("%-8d | %12.2f | %12.2f | %10.1fx | %12.2e | %4d vs %4d\n",
                   results[i].N,
                   results[i].vanilla_time_ms,
                   results[i].mpc_time_ms,
                   speedup,
                   results[i].solution_error,
                   results[i].vanilla_iterations,
                   results[i].mpc_iterations);
        } else {
            printf("%-8d | FAILED\n", results[i].N);
        }
    }
    
    printf("\n");
    
    // Write CSV for plotting
    std::ofstream csv_file("benchmark_horizon_results.csv");
    csv_file << "Horizon,Vanilla_ms,MPC_ms,Speedup,Solution_Error,Vanilla_Iterations,MPC_Iterations\n";
    
    for (int i = 0; i < num_horizons; ++i) {
        if (results[i].success) {
            double speedup = results[i].vanilla_time_ms / results[i].mpc_time_ms;
            csv_file << results[i].N << ","
                    << results[i].vanilla_time_ms << ","
                    << results[i].mpc_time_ms << ","
                    << speedup << ","
                    << results[i].solution_error << ","
                    << results[i].vanilla_iterations << ","
                    << results[i].mpc_iterations << "\n";
        }
    }
    
    csv_file.close();
    printf("Results saved to: benchmark_horizon_results.csv\n");
    printf("\n");
    
    // Solution verification
    printf("========================================================\n");
    printf("   SOLUTION VERIFICATION\n");
    printf("========================================================\n");
    printf("\n");
    
    bool all_match = true;
    for (int i = 0; i < num_horizons; ++i) {
        if (results[i].success) {
            if (results[i].solution_error < 1e-6) {
                printf("N=%-3d: ✓ Solutions match (error: %.2e)\n", 
                       results[i].N, results[i].solution_error);
            } else {
                printf("N=%-3d: ✗ Solutions differ (error: %.2e)\n", 
                       results[i].N, results[i].solution_error);
                all_match = false;
            }
        }
    }
    
    printf("\n");
    if (all_match) {
        printf("✅ All solutions verified: MPC-aware and vanilla qpOASES produce identical results!\n");
    } else {
        printf("⚠️  Some solutions differ: check for numerical issues.\n");
    }
    printf("\n");
    
    delete[] results;
    
    return 0;
}
