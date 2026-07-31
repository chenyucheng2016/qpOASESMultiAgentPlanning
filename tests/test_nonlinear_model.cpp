#include <qpOASES/NonlinearModel.hpp>
#include <algorithm>
#include <cmath>
#include <cstdio>
USING_NAMESPACE_QPOASES

int main()
{
    UnicycleModel model(0.17);
    const real_t x[4] = {1.2, -0.4, 0.73, 2.1};
    const real_t u[2] = {-0.6, 0.31};
    real_t A[16], B[8], C[8], base[4], shifted[4];
    real_t xx[4] = {x[0], x[1], x[2], x[3]};
    real_t uu[2] = {u[0], u[1]};
    const real_t eps = 1.0e-7;
    real_t error = 0.0;
    int_t i, j;
    model.dynamics(x, u, base);
    model.linearizeDynamics(x, u, A, B);
    model.linearizePosition(x, C);
    for (j = 0; j < 4; ++j)
    {
        xx[j] += eps; model.dynamics(xx, u, shifted); xx[j] -= eps;
        for (i = 0; i < 4; ++i)
            error = std::max(error, std::fabs((shifted[i] - base[i]) / eps - A[i * 4 + j]));
    }
    for (j = 0; j < 2; ++j)
    {
        uu[j] += eps; model.dynamics(x, uu, shifted); uu[j] -= eps;
        for (i = 0; i < 4; ++i)
            error = std::max(error, std::fabs((shifted[i] - base[i]) / eps - B[i * 2 + j]));
    }
    real_t p[2], ps[2];
    model.position(x, p);
    for (j = 0; j < 4; ++j)
    {
        xx[j] += eps; model.position(xx, ps); xx[j] -= eps;
        for (i = 0; i < 2; ++i)
            error = std::max(error, std::fabs((ps[i] - p[i]) / eps - C[i * 4 + j]));
    }
    std::printf("maximum analytic Jacobian error: %.3e\n", error);
    return error <= 1.0e-6 ? 0 : 1;
}
