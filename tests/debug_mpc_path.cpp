/*
 *  Minimal test to debug why MPC-aware path is not taken
 */

#include <qpOASES.hpp>
#include <iostream>

USING_NAMESPACE_QPOASES

int main() {
    // Tiny problem: 2 states, 1 input, N=5
    const int nx = 2;
    const int nu = 1;
    const int N = 5;
    
    // Total variables: N*nx + (N-1)*nu = 5*2 + 4*1 = 14
    int nV = N*nx + (N-1)*nu;
    // Dynamics constraints: (N-1)*nx = 4*2 = 8
    int nC = (N-1)*nx;
    
    printf("\n=== MPC PATH DEBUG TEST ===\n");
    printf("Problem: nx=%d, nu=%d, N=%d\n", nx, nu, N);
    printf("Variables: nV=%d\n", nV);
    printf("Constraints: nC=%d\n\n", nC);
    
    // Simple system matrices
    real_t A[4] = {1.0, 0.0, 0.1, 1.0};  // Double integrator
    real_t B[2] = {0.005, 0.1};
    real_t Q[4] = {1.0, 0.0, 0.0, 1.0};
    real_t R[1] = {1.0};
    
    // Create QP
    QProblem qp(nV, nC);
    
    printf("Step 1: Calling setupMPCStructure()...\n");
    returnValue ret = qp.setupMPCStructure(N, nx, nu, A, B, Q, R);
    printf("  Result: %d (%s)\n\n", ret, ret == SUCCESSFUL_RETURN ? "SUCCESS" : "FAILED");
    
    if (ret != SUCCESSFUL_RETURN) {
        printf("ERROR: setupMPCStructure() failed!\n");
        return 1;
    }
    
    // Enable MPC mode AFTER setupMPCStructure
    printf("Step 1b: Setting MPC options...\n");
    Options options;
    options.enableMPCRiccati = BT_TRUE;
    options.printLevel = PL_HIGH;  // Maximum verbosity
    printf("  Before setOptions(): options.enableMPCRiccati = %d\n", options.enableMPCRiccati);
    qp.setOptions(options);
    
    // Verify the option was actually set
    Options verifyOptions = qp.getOptions();
    printf("  After setOptions(): qp.getOptions().enableMPCRiccati = %d\n", verifyOptions.enableMPCRiccati);
    printf("  Options set: enableMPCRiccati = %s\n\n", verifyOptions.enableMPCRiccati == BT_TRUE ? "TRUE" : "FALSE");
    
    // Build simple QP matrices
    real_t* H = new real_t[nV*nV];
    real_t* g = new real_t[nV];
    real_t* A_qp = new real_t[nC*nV];
    real_t* lbA = new real_t[nC];
    real_t* ubA = new real_t[nC];
    
    // Initialize to zero
    for (int i = 0; i < nV*nV; ++i) H[i] = 0.0;
    for (int i = 0; i < nV; ++i) g[i] = 0.0;
    for (int i = 0; i < nC*nV; ++i) A_qp[i] = 0.0;
    for (int i = 0; i < nC; ++i) { lbA[i] = 0.0; ubA[i] = 0.0; }
    
    // Build simple Hessian (Q blocks)
    for (int k = 0; k < N; ++k) {
        for (int i = 0; i < nx; ++i) {
            int idx = (k*nx + i)*nV + (k*nx + i);
            H[idx] = 1.0;  // Diagonal Q
        }
    }
    
    // Build simple R blocks
    int u_offset = nx*N;
    for (int k = 0; k < N-1; ++k) {
        int idx = (u_offset + k)*nV + (u_offset + k);
        H[idx] = 1.0;  // Diagonal R
    }
    
    // Build dynamics constraints (simple: x_{k+1} - x_k = 0)
    for (int k = 0; k < N-1; ++k) {
        for (int i = 0; i < nx; ++i) {
            int row = k*nx + i;
            A_qp[row*nV + k*nx + i] = -1.0;      // -x_k
            A_qp[row*nV + (k+1)*nx + i] = 1.0;   // +x_{k+1}
        }
    }
    
    printf("Step 2: Calling init()...\n");
    printf("  (Watch for 'MPC-aware qpOASES: Riccati LQR solved' message)\n");
    printf("  If you don't see it, the MPC path is NOT being taken!\n\n");
    printf("--- BEGIN INIT OUTPUT ---\n");
    
    int nWSR = 100;
    ret = qp.init(H, g, A_qp, NULL, NULL, lbA, ubA, nWSR);
    
    printf("--- END INIT OUTPUT ---\n\n");
    printf("Step 3: Results\n");
    printf("  Status: %d (%s)\n", ret, ret == SUCCESSFUL_RETURN ? "SUCCESS" : "FAILED");
    printf("  Iterations: %d\n\n", nWSR);
    
    if (ret == SUCCESSFUL_RETURN) {
        printf("✓ QP solved successfully\n");
    } else {
        printf("✗ QP solve failed\n");
    }
    
    // Cleanup
    delete[] ubA;
    delete[] lbA;
    delete[] A_qp;
    delete[] g;
    delete[] H;
    
    printf("\n=== END DEBUG TEST ===\n\n");
    
    return 0;
}
