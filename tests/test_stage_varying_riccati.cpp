#include <qpOASES/StageVaryingRiccati.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
USING_NAMESPACE_QPOASES

int main()
{
    StageVaryingLqrProblem p;
    StageVaryingLqrSolution s;
    const int_t N = 6, nx = 2, nu = 1;
    p.horizon = N; p.nx = nx; p.nu = nu;
    p.x0.assign(nx, 0.0); p.x0[0] = 2.0; p.x0[1] = -0.5;
    p.A.assign(N * nx * nx, 0.0); p.B.assign(N * nx * nu, 0.0);
    p.c.assign(N * nx, 0.0); p.Q.assign(N * nx * nx, 0.0);
    p.R.assign(N, 0.0); p.q.assign(N * nx, 0.0); p.r.assign(N, 0.0);
    p.stateControl.assign(N * nx * nu, 0.0);
    p.Qterminal.assign(nx * nx, 0.0); p.qterminal.assign(nx, 0.0);
    int_t k, i, j;
    for (k = 0; k < N; ++k)
    {
        real_t* A = &p.A[k * nx * nx]; real_t* B = &p.B[k * nx];
        real_t* Q = &p.Q[k * nx * nx];
        A[0] = 1.0; A[1] = 0.1 + 0.01 * k; A[3] = 1.0;
        B[0] = 0.005; B[1] = 0.1; p.c[k * nx] = 0.01 * k;
        Q[0] = 2.0; Q[3] = 0.3; p.R[k] = 0.2 + 0.01 * k;
        p.stateControl[k * nx] = 0.02 + 0.005 * k;
        p.q[k * nx] = -0.4; p.r[k] = 0.03;
    }
    p.Qterminal[0] = 8.0; p.Qterminal[3] = 1.0; p.qterminal[0] = -1.2;
    if (!solveStageVaryingLqr(p, s)) return 1;
    real_t defect = 0.0, stationarityError = 0.0;
    for (k = 0; k < N; ++k)
    {
        const real_t* A = &p.A[k * nx * nx]; const real_t* B = &p.B[k * nx];
        const real_t* x = &s.states[k * nx]; const real_t* next = &s.states[(k + 1) * nx];
        for (i = 0; i < nx; ++i)
        {
            real_t predicted = p.c[k * nx + i] + B[i] * s.controls[k];
            for (j = 0; j < nx; ++j) predicted += A[i * nx + j] * x[j];
            defect = std::max(defect, std::fabs(predicted - next[i]));
        }
        real_t stationarity = p.R[k] * s.controls[k] + p.r[k];
        stationarity += p.stateControl[k * nx] * x[0];
        for (i = 0; i < nx; ++i) stationarity += B[i] * s.costates[(k + 1) * nx + i];
        stationarityError = std::max(stationarityError, std::fabs(stationarity));
    }
    std::printf("maximum dynamics defect: %.3e, control stationarity: %.3e\n", defect, stationarityError);
    return defect <= 1.0e-10 && stationarityError <= 1.0e-8 ? 0 : 1;
}
