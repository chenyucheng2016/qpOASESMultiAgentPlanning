#include <qpOASES/NonlinearModel.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <vector>

USING_NAMESPACE_QPOASES

namespace
{

real_t maximumJacobianError(
    const NonlinearModel& model,
    const std::vector<real_t>& state,
    const std::vector<real_t>& control
)
{
    const int_t nx = model.stateDimension();
    const int_t nu = model.controlDimension();
    const int_t np = model.positionDimension();
    const real_t epsilon = 1.0e-7;
    std::vector<real_t> A(nx * nx, 0.0), B(nx * nu, 0.0);
    std::vector<real_t> C(np * nx, 0.0), base(nx, 0.0), shifted(nx, 0.0);
    std::vector<real_t> x(state), u(control), p(np, 0.0), shiftedP(np, 0.0);
    real_t maximum = 0.0;
    model.dynamics(&state[0], &control[0], &base[0]);
    model.linearizeDynamics(&state[0], &control[0], &A[0], &B[0]);
    model.position(&state[0], &p[0]);
    model.linearizePosition(&state[0], &C[0]);

    for (int_t column = 0; column < nx; ++column)
    {
        x[column] += epsilon;
        model.dynamics(&x[0], &control[0], &shifted[0]);
        model.position(&x[0], &shiftedP[0]);
        x[column] -= epsilon;
        for (int_t row = 0; row < nx; ++row)
            maximum = std::max(maximum, std::fabs(
                (shifted[row] - base[row]) / epsilon
                - A[row * nx + column]
            ));
        for (int_t row = 0; row < np; ++row)
            maximum = std::max(maximum, std::fabs(
                (shiftedP[row] - p[row]) / epsilon
                - C[row * nx + column]
            ));
    }
    for (int_t column = 0; column < nu; ++column)
    {
        u[column] += epsilon;
        model.dynamics(&state[0], &u[0], &shifted[0]);
        u[column] -= epsilon;
        for (int_t row = 0; row < nx; ++row)
            maximum = std::max(maximum, std::fabs(
                (shifted[row] - base[row]) / epsilon
                - B[row * nu + column]
            ));
    }
    return maximum;
}

}

int main()
{
    BicycleModel bicycle(0.13, 2.7);
    QuadcopterModel quadcopter(0.08, 1.3, 9.81);
    const real_t bicycleStateData[] = {1.1, -0.7, 0.62, 3.2, 0.21};
    const real_t bicycleControlData[] = {-0.4, 0.17};
    const real_t quadStateData[] = {
        0.3, -1.2, 2.4, 0.7, -0.2, 0.15, 0.17, -0.13, 0.41
    };
    const real_t quadControlData[] = {13.1, -0.08, 0.11, 0.19};
    const real_t bicycleError = maximumJacobianError(
        bicycle,
        std::vector<real_t>(bicycleStateData, bicycleStateData + 5),
        std::vector<real_t>(bicycleControlData, bicycleControlData + 2)
    );
    const real_t quadcopterError = maximumJacobianError(
        quadcopter,
        std::vector<real_t>(quadStateData, quadStateData + 9),
        std::vector<real_t>(quadControlData, quadControlData + 4)
    );
    std::printf("bicycle Jacobian error %.3e, quadcopter Jacobian error %.3e\n",
        bicycleError, quadcopterError);
    return bicycleError <= 1.0e-6 && quadcopterError <= 1.0e-6 ? 0 : 1;
}
