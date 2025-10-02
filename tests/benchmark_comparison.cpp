/*
 *  Direct Performance Comparison: Vanilla vs MPC-Aware qpOASES
 *  
 *  Benchmarks the quadrotor hover stabilization problem with both methods
 *  to measure actual speedup achieved by MPC-aware implementation.
 */

#include "utils/test_problems.hpp"
#include <iostream>
#include <chrono>
#include <iomanip>

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
        return duration.count() / 1000.0;  // Convert to milliseconds
    }
};

// ============================================================================
// BENCHMARK FUNCTIONS
// ============================================================================

struct BenchmarkResult {
    double avg_time_ms;
    double min_time_ms;
    double max_time_ms;
    double std_dev_ms;
    int avg_iterations;
    bool success;
    real_t* solution;  // Store solution for verification
    int nV;            // Problem size
};

BenchmarkResult benchmarkQuadrotor(bool enableMPCRiccati, int num_runs = 10) {
    using namespace RealWorldProblems;
    
    const int nx = QuadrotorHover::nx;
    const int nu = QuadrotorHover::nu;
    const int N = 30;
    
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
    
    // Build Hessian and gradient for full QP
    real_t* H = new real_t[nV*nV];
    real_t* g = new real_t[nV];
    real_t* A_qp = new real_t[nC*nV];
    real_t* lbA = new real_t[nC];
    real_t* ubA = new real_t[nC];
    
    // Initialize with zeros
    for (int i = 0; i < nV*nV; ++i) H[i] = 0.0;
    for (int i = 0; i < nV; ++i) g[i] = 0.0;
    for (int i = 0; i < nC*nV; ++i) A_qp[i] = 0.0;
    for (int i = 0; i < nC; ++i) { lbA[i] = 0.0; ubA[i] = 0.0; }
    
    // Build block-diagonal Hessian (Q and R blocks)
    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nx; ++j) {
                int idx = (k*nx + i)*nV + (k*nx + j);
                H[idx] = Q[i*nx + j];
            }
        }
    }
    
    int u_offset = nx*N;
    for (int k = 0; k < N-1; ++k) {
        for (int i = 0; i < nu; ++i) {
            for (int j = 0; j < nu; ++j) {
                int idx = (u_offset + k*nu + i)*nV + (u_offset + k*nu + j);
                H[idx] = R[i*nu + j];
            }
        }
    }
    
    // Build constraint matrix: x_{k+1} = A*x_k + B*u_k
    for (int k = 0; k < N-1; ++k) {
        int row_offset = k * nx;
        
        // -A*x_k term
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nx; ++j) {
                int row = row_offset + i;
                int col = k*nx + j;
                A_qp[row*nV + col] = -A[i*nx + j];
            }
        }
        
        // +I*x_{k+1} term
        for (int i = 0; i < nx; ++i) {
            int row = row_offset + i;
            int col = (k+1)*nx + i;
            A_qp[row*nV + col] = 1.0;
        }
        
        // -B*u_k term
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nu; ++j) {
                int row = row_offset + i;
                int col = u_offset + k*nu + j;
                A_qp[row*nV + col] = -B[i*nu + j];
            }
        }
    }
    
    // Run multiple times for statistics
    double* times = new double[num_runs];
    int* iterations = new int[num_runs];
    int successful_runs = 0;
    
    // Allocate solution storage
    real_t* solution = new real_t[nV];
    for (int i = 0; i < nV; ++i) solution[i] = 0.0;
    
    for (int run = 0; run < num_runs; ++run) {
        QProblem qp(nV, nC);
        
        Options options;
        options.enableMPCRiccati = enableMPCRiccati ? BT_TRUE : BT_FALSE;
        options.printLevel = PL_NONE;
        qp.setOptions(options);
        
        // Setup MPC structure if using MPC-aware mode
        if (enableMPCRiccati) {
            qp.setupMPCStructure(N, nx, nu, A, B, Q, R);
        }
        
        Timer timer;
        timer.start();
        
        int nWSR = 1000;
        returnValue ret = qp.init(H, g, A_qp, NULL, NULL, lbA, ubA, nWSR);
        
        times[run] = timer.elapsed_ms();
        iterations[run] = nWSR;
        
        if (ret == SUCCESSFUL_RETURN) {
            successful_runs++;
            // Store solution from first successful run
            if (successful_runs == 1) {
                qp.getPrimalSolution(solution);
            }
        }
    }
    
    // Compute statistics
    BenchmarkResult result;
    result.success = (successful_runs == num_runs);
    result.solution = solution;  // Store solution pointer
    result.nV = nV;              // Store problem size
    
    if (successful_runs > 0) {
        double sum = 0.0, sum_sq = 0.0;
        int sum_iter = 0;
        result.min_time_ms = times[0];
        result.max_time_ms = times[0];
        
        for (int i = 0; i < num_runs; ++i) {
            sum += times[i];
            sum_sq += times[i] * times[i];
            sum_iter += iterations[i];
            if (times[i] < result.min_time_ms) result.min_time_ms = times[i];
            if (times[i] > result.max_time_ms) result.max_time_ms = times[i];
        }
        
        result.avg_time_ms = sum / num_runs;
        result.avg_iterations = sum_iter / num_runs;
        
        double variance = (sum_sq / num_runs) - (result.avg_time_ms * result.avg_time_ms);
        result.std_dev_ms = (variance > 0) ? sqrt(variance) : 0.0;
    } else {
        result.avg_time_ms = 0.0;
        result.min_time_ms = 0.0;
        result.max_time_ms = 0.0;
        result.std_dev_ms = 0.0;
        result.avg_iterations = 0;
    }
    
    // Cleanup
    delete[] iterations;
    delete[] times;
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
    printf("============================================\n");
    printf("   PERFORMANCE COMPARISON BENCHMARK\n");
    printf("   Vanilla vs MPC-Aware qpOASES\n");
    printf("============================================\n");
    printf("\n");
    
    printf("Problem: Quadrotor Hover Stabilization\n");
    printf("  States (nx):  12\n");
    printf("  Inputs (nu):  4\n");
    printf("  Horizon (N):  30\n");
    printf("  Variables:    360 + 116 = 476\n");
    printf("  Constraints:  348 (dynamics)\n");
    printf("\n");
    
    int num_runs = 50;  // Run each method 50 times for good statistics
    
    printf("Running %d trials for each method...\n", num_runs);
    printf("\n");
    
    // Benchmark vanilla qpOASES
    printf("Testing VANILLA qpOASES...\n");
    BenchmarkResult vanilla_result = benchmarkQuadrotor(false, num_runs);
    
    // Benchmark MPC-aware qpOASES
    printf("Testing MPC-AWARE qpOASES...\n");
    BenchmarkResult mpc_result = benchmarkQuadrotor(true, num_runs);
    
    printf("\n");
    printf("============================================\n");
    printf("   RESULTS\n");
    printf("============================================\n");
    printf("\n");
    
    // Vanilla results
    printf("VANILLA qpOASES:\n");
    printf("  Status:           %s\n", vanilla_result.success ? "SUCCESS" : "FAILED");
    if (vanilla_result.success) {
        printf("  Average time:     %.3f ms\n", vanilla_result.avg_time_ms);
        printf("  Min time:         %.3f ms\n", vanilla_result.min_time_ms);
        printf("  Max time:         %.3f ms\n", vanilla_result.max_time_ms);
        printf("  Std deviation:    %.3f ms\n", vanilla_result.std_dev_ms);
        printf("  Avg iterations:   %d\n", vanilla_result.avg_iterations);
    }
    printf("\n");
    
    // MPC-aware results
    printf("MPC-AWARE qpOASES:\n");
    printf("  Status:           %s\n", mpc_result.success ? "SUCCESS" : "FAILED");
    if (mpc_result.success) {
        printf("  Average time:     %.3f ms\n", mpc_result.avg_time_ms);
        printf("  Min time:         %.3f ms\n", mpc_result.min_time_ms);
        printf("  Max time:         %.3f ms\n", mpc_result.max_time_ms);
        printf("  Std deviation:    %.3f ms\n", mpc_result.std_dev_ms);
        printf("  Avg iterations:   %d\n", mpc_result.avg_iterations);
    }
    printf("\n");
    
    // Comparison
    if (vanilla_result.success && mpc_result.success) {
        double speedup = vanilla_result.avg_time_ms / mpc_result.avg_time_ms;
        double time_saved = vanilla_result.avg_time_ms - mpc_result.avg_time_ms;
        double time_saved_percent = (time_saved / vanilla_result.avg_time_ms) * 100.0;
        
        printf("============================================\n");
        printf("   PERFORMANCE IMPROVEMENT\n");
        printf("============================================\n");
        printf("\n");
        printf("  Speedup:          %.1fx faster\n", speedup);
        printf("  Time saved:       %.3f ms (%.1f%%)\n", time_saved, time_saved_percent);
        printf("  Iteration ratio:  %.2fx\n", 
               (double)vanilla_result.avg_iterations / mpc_result.avg_iterations);
        printf("\n");
        
        if (speedup >= 100.0) {
            printf("  🚀 OUTSTANDING: Over 100x speedup!\n");
        } else if (speedup >= 10.0) {
            printf("  ✅ EXCELLENT: 10x+ speedup achieved!\n");
        } else if (speedup >= 2.0) {
            printf("  ✓ GOOD: Significant improvement.\n");
        } else {
            printf("  ⚠️ MODEST: Limited improvement.\n");
        }
        
        printf("\n");
        printf("============================================\n");
        printf("   IMPACT ON REAL-TIME CONTROL\n");
        printf("============================================\n");
        printf("\n");
        printf("  Vanilla max rate:    %.0f Hz\n", 1000.0 / vanilla_result.avg_time_ms);
        printf("  MPC-aware max rate:  %.0f Hz\n", 1000.0 / mpc_result.avg_time_ms);
        printf("\n");
        printf("  For 100 Hz control (10ms budget):\n");
        printf("    Vanilla:   %s (%.1f%% of budget)\n",
               vanilla_result.avg_time_ms < 10.0 ? "✅ Feasible" : "❌ Too slow",
               (vanilla_result.avg_time_ms / 10.0) * 100.0);
        printf("    MPC-aware: %s (%.1f%% of budget)\n",
               mpc_result.avg_time_ms < 10.0 ? "✅ Feasible" : "❌ Too slow",
               (mpc_result.avg_time_ms / 10.0) * 100.0);
        printf("\n");
    }
    
    printf("============================================\n");
    printf("\n");
    
    // ========================================================================
    // SOLUTION VERIFICATION
    // ========================================================================
    
    if (vanilla_result.success && mpc_result.success && 
        vanilla_result.solution != NULL && mpc_result.solution != NULL) {
        
        printf("============================================\n");
        printf("   SOLUTION VERIFICATION\n");
        printf("============================================\n");
        printf("\n");
        
        // Compute maximum absolute difference
        double max_error = 0.0;
        double avg_error = 0.0;
        int nV = vanilla_result.nV;
        
        for (int i = 0; i < nV; ++i) {
            double error = fabs(vanilla_result.solution[i] - mpc_result.solution[i]);
            if (error > max_error) max_error = error;
            avg_error += error;
        }
        avg_error /= nV;
        
        printf("Problem size:      %d variables\n", nV);
        printf("Max error:         %.6e\n", max_error);
        printf("Average error:     %.6e\n", avg_error);
        printf("\n");
        
        if (max_error < 1e-6) {
            printf("✅ VERIFIED: Solutions match within tolerance (%.2e < 1e-6)\n", max_error);
            printf("   MPC-aware and vanilla qpOASES produce identical results!\n");
        } else if (max_error < 1e-3) {
            printf("⚠️  CLOSE: Solutions are similar but differ slightly (%.2e)\n", max_error);
            printf("   This may be due to numerical differences in the solution path.\n");
        } else {
            printf("❌ MISMATCH: Solutions differ significantly (%.2e)\n", max_error);
            printf("   WARNING: MPC-aware may have a bug or different optimal solution!\n");
        }
        
        printf("\n");
        printf("============================================\n");
        printf("\n");
    }
    
    // Cleanup
    if (vanilla_result.solution) delete[] vanilla_result.solution;
    if (mpc_result.solution) delete[] mpc_result.solution;
    
    return 0;
}
