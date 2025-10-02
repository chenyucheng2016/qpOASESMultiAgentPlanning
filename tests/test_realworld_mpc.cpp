/*
 *  Real-World MPC Test Suite
 *  
 *  Tests MPC-aware qpOASES on realistic control problems:
 *  - Quadrotor stabilization
 *  - Vehicle path tracking  
 *  - Inverted pendulum
 *  - Mass-spring-damper chain
 */

#include "utils/test_problems.hpp"
#include <iostream>
#include <chrono>

USING_NAMESPACE_QPOASES

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================

double getTime() {
    auto now = std::chrono::high_resolution_clock::now();
    auto duration = now.time_since_epoch();
    return std::chrono::duration<double, std::milli>(duration).count();
}

bool testMPCProblem(const char* name, int nx, int nu, int N, 
                    real_t* A, real_t* B, real_t* Q, real_t* R) {
    printf("\n========================================\n");
    printf("TEST: %s\n", name);
    printf("========================================\n");
    printf("Dimensions: nx=%d, nu=%d, N=%d\n", nx, nu, N);
    
    int nV = nx*N + nu*(N-1);
    int nC = (N-1)*nx;
    
    // Test with MPC-aware mode
    printf("\nMode: MPC-Aware (Riccati + O(N) TQ)\n");
    
    QProblem qp_mpc(nV, nC);
    Options options_mpc;
    options_mpc.enableMPCRiccati = BT_TRUE;
    qp_mpc.setOptions(options_mpc);
    qp_mpc.setupMPCStructure(N, nx, nu, A, B, Q, R);
    
    real_t* x_traj = new real_t[nV];
    for (int i = 0; i < nV; ++i) x_traj[i] = 0.0;
    
    double start_mpc = getTime();
    returnValue ret_mpc = qp_mpc.solveRiccatiLQR(x_traj, 0);
    double time_mpc = getTime() - start_mpc;
    
    bool success = false;
    
    if (ret_mpc == SUCCESSFUL_RETURN) {
        printf("  ✓ Riccati LQR solved successfully\n");
        printf("  ✓ Computation time: %.3f ms\n", time_mpc);
        success = true;
    } else {
        printf("  ✗ Riccati LQR failed (error code %d)\n", ret_mpc);
    }
    
    delete[] x_traj;
    
    return success;
}

// ============================================================================
// TEST 1: QUADROTOR HOVER STABILIZATION
// ============================================================================

void test_quadrotor() {
    using namespace RealWorldProblems;
    
    const int nx = QuadrotorHover::nx;
    const int nu = QuadrotorHover::nu;
    const int N = 30;
    
    real_t* A = new real_t[nx*nx];
    real_t* B = new real_t[nx*nu];
    real_t* Q = new real_t[nx*nx];
    real_t* R = new real_t[nu*nu];
    
    QuadrotorHover::getSystemMatrices(A, B, 0.1);
    QuadrotorHover::getCostMatrices(Q, R);
    
    bool pass = testMPCProblem("Quadrotor Hover Stabilization", nx, nu, N, A, B, Q, R);
    
    if (pass) {
        printf("\n** QUADROTOR TEST: PASS **\n");
    } else {
        printf("\n** QUADROTOR TEST: FAIL **\n");
    }
    
    delete[] R;
    delete[] Q;
    delete[] B;
    delete[] A;
}

// ============================================================================
// TEST 2: VEHICLE PATH TRACKING
// ============================================================================

void test_vehicle() {
    using namespace RealWorldProblems;
    
    const int nx = VehicleTracking::nx;
    const int nu = VehicleTracking::nu;
    const int N = 50;
    
    real_t* A = new real_t[nx*nx];
    real_t* B = new real_t[nx*nu];
    real_t* Q = new real_t[nx*nx];
    real_t* R = new real_t[nu*nu];
    
    VehicleTracking::getSystemMatrices(A, B, 0.1, 5.0);
    VehicleTracking::getCostMatrices(Q, R);
    
    bool pass = testMPCProblem("Vehicle Path Tracking", nx, nu, N, A, B, Q, R);
    
    if (pass) {
        printf("\n** VEHICLE TEST: PASS **\n");
    } else {
        printf("\n** VEHICLE TEST: FAIL **\n");
    }
    
    delete[] R;
    delete[] Q;
    delete[] B;
    delete[] A;
}

// ============================================================================
// TEST 3: INVERTED PENDULUM (CART-POLE)
// ============================================================================

void test_cartpole() {
    using namespace RealWorldProblems;
    
    const int nx = InvertedPendulum::nx;
    const int nu = InvertedPendulum::nu;
    const int N = 100;
    
    real_t* A = new real_t[nx*nx];
    real_t* B = new real_t[nx*nu];
    real_t* Q = new real_t[nx*nx];
    real_t* R = new real_t[nu*nu];
    
    InvertedPendulum::getSystemMatrices(A, B, 0.02);
    InvertedPendulum::getCostMatrices(Q, R);
    
    bool pass = testMPCProblem("Inverted Pendulum (Cart-Pole)", nx, nu, N, A, B, Q, R);
    
    if (pass) {
        printf("\n** CART-POLE TEST: PASS **\n");
    } else {
        printf("\n** CART-POLE TEST: FAIL **\n");
    }
    
    delete[] R;
    delete[] Q;
    delete[] B;
    delete[] A;
}

// ============================================================================
// TEST 4: MASS-SPRING-DAMPER CHAIN
// ============================================================================

void test_massSpring() {
    using namespace RealWorldProblems;
    
    const int nx = MassSpringDamper::nx;
    const int nu = MassSpringDamper::nu;
    const int N = 80;
    
    real_t* A = new real_t[nx*nx];
    real_t* B = new real_t[nx*nu];
    real_t* Q = new real_t[nx*nx];
    real_t* R = new real_t[nu*nu];
    
    MassSpringDamper::getSystemMatrices(A, B, 0.05);
    MassSpringDamper::getCostMatrices(Q, R);
    
    bool pass = testMPCProblem("Mass-Spring-Damper Chain", nx, nu, N, A, B, Q, R);
    
    if (pass) {
        printf("\n** MASS-SPRING TEST: PASS **\n");
    } else {
        printf("\n** MASS-SPRING TEST: FAIL **\n");
    }
    
    delete[] R;
    delete[] Q;
    delete[] B;
    delete[] A;
}

// ============================================================================
// MAIN TEST RUNNER
// ============================================================================

int main() {
    printf("\n");
    printf("============================================\n");
    printf("   REAL-WORLD MPC TEST SUITE\n");
    printf("============================================\n");
    printf("\n");
    printf("Testing MPC-aware qpOASES on:\n");
    printf("  1. Quadrotor hover stabilization (12 states)\n");
    printf("  2. Vehicle path tracking (4 states)\n");
    printf("  3. Inverted pendulum (4 states)\n");
    printf("  4. Mass-spring-damper chain (10 states)\n");
    printf("\n");
    
    test_quadrotor();
    test_vehicle();
    test_cartpole();
    test_massSpring();
    
    printf("\n");
    printf("============================================\n");
    printf("   ALL REAL-WORLD TESTS COMPLETED\n");
    printf("============================================\n");
    printf("\n");
    
    return 0;
}
