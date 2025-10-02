/*
 *  Diagnostic Test: Verify Riccati Warm Start is Used
 *  
 *  This test explicitly checks if:
 *  1. Riccati code is executed
 *  2. LQR solution is set as warm start
 *  3. Dynamics constraints are marked active
 *  4. Working set iterations are reduced
 */

#include "utils/test_problems.hpp"
#include <iostream>

USING_NAMESPACE_QPOASES

int main() {
    using namespace RealWorldProblems;
    
    printf("\n");
    printf("============================================\n");
    printf("   WARM START DIAGNOSTIC TEST\n");
    printf("============================================\n");
    printf("\n");
    
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
    
    int nV = nx*N + nu*(N-1);
    int nC = (N-1)*nx;
    
    printf("Problem Setup:\n");
    printf("  States (nx): %d\n", nx);
    printf("  Inputs (nu): %d\n", nu);
    printf("  Horizon (N): %d\n", N);
    printf("  Variables:   %d\n", nV);
    printf("  Constraints: %d\n", nC);
    printf("\n");
    
    // Build QP matrices (same as benchmark)
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
    
    // Build Hessian
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
    
    // Build constraint matrix
    for (int k = 0; k < N-1; ++k) {
        int row_offset = k * nx;
        
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nx; ++j) {
                int row = row_offset + i;
                int col = k*nx + j;
                A_qp[row*nV + col] = -A[i*nx + j];
            }
        }
        
        for (int i = 0; i < nx; ++i) {
            int row = row_offset + i;
            int col = (k+1)*nx + i;
            A_qp[row*nV + col] = 1.0;
        }
        
        for (int i = 0; i < nx; ++i) {
            for (int j = 0; j < nu; ++j) {
                int row = row_offset + i;
                int col = u_offset + k*nu + j;
                A_qp[row*nV + col] = -B[i*nu + j];
            }
        }
    }
    
    printf("Testing MPC-AWARE qpOASES:\n");
    printf("--------------------------------------------\n");
    
    QProblem qp_mpc(nV, nC);
    
    Options options_mpc;
    options_mpc.enableMPCRiccati = BT_TRUE;
    options_mpc.printLevel = PL_HIGH;  // Enable output to see messages
    qp_mpc.setOptions(options_mpc);
    
    printf("\nCalling setupMPCStructure()...\n");
    returnValue ret_setup = qp_mpc.setupMPCStructure(N, nx, nu, A, B, Q, R);
    if (ret_setup == SUCCESSFUL_RETURN) {
        printf("✓ MPC structure setup successful\n");
    } else {
        printf("✗ MPC structure setup failed (code %d)\n", ret_setup);
    }
    
    printf("\nCalling init() with enableMPCRiccati=TRUE...\n");
    printf("(Watch for 'MPC-aware qpOASES: Riccati LQR solved' message)\n");
    printf("\n");
    
    int nWSR_mpc = 1000;
    returnValue ret_mpc = qp_mpc.init(H, g, A_qp, NULL, NULL, lbA, ubA, nWSR_mpc);
    
    printf("\n");
    printf("MPC-AWARE Results:\n");
    printf("  Status: %s\n", ret_mpc == SUCCESSFUL_RETURN ? "SUCCESS" : "FAILED");
    printf("  Iterations: %d\n", nWSR_mpc);
    printf("\n");
    
    printf("--------------------------------------------\n");
    printf("\nTesting VANILLA qpOASES:\n");
    printf("--------------------------------------------\n");
    
    QProblem qp_vanilla(nV, nC);
    
    Options options_vanilla;
    options_vanilla.enableMPCRiccati = BT_FALSE;
    options_vanilla.printLevel = PL_LOW;
    qp_vanilla.setOptions(options_vanilla);
    
    printf("\nCalling init() with enableMPCRiccati=FALSE...\n");
    
    int nWSR_vanilla = 1000;
    returnValue ret_vanilla = qp_vanilla.init(H, g, A_qp, NULL, NULL, lbA, ubA, nWSR_vanilla);
    
    printf("\n");
    printf("VANILLA Results:\n");
    printf("  Status: %s\n", ret_vanilla == SUCCESSFUL_RETURN ? "SUCCESS" : "FAILED");
    printf("  Iterations: %d\n", nWSR_vanilla);
    printf("\n");
    
    printf("============================================\n");
    printf("   COMPARISON\n");
    printf("============================================\n");
    printf("\n");
    printf("  Vanilla iterations:   %d\n", nWSR_vanilla);
    printf("  MPC-aware iterations: %d\n", nWSR_mpc);
    printf("  Reduction:            %d iterations\n", nWSR_vanilla - nWSR_mpc);
    printf("  Improvement:          %.1f%%\n", 
           100.0 * (nWSR_vanilla - nWSR_mpc) / (double)nWSR_vanilla);
    printf("\n");
    
    if (nWSR_mpc < nWSR_vanilla) {
        printf("✅ MPC-aware REDUCED iterations!\n");
    } else if (nWSR_mpc == nWSR_vanilla) {
        printf("❌ NO IMPROVEMENT: Same number of iterations\n");
        printf("   → Warm start not being used OR not helping\n");
    } else {
        printf("❌ MPC-aware INCREASED iterations (unexpected!)\n");
    }
    
    printf("\n");
    
    // Cleanup
    delete[] ubA;
    delete[] lbA;
    delete[] A_qp;
    delete[] g;
    delete[] H;
    delete[] R;
    delete[] Q;
    delete[] B;
    delete[] A;
    
    return 0;
}
