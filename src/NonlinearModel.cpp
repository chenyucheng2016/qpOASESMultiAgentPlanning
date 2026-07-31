#include <qpOASES/NonlinearModel.hpp>

#include <cmath>

BEGIN_NAMESPACE_QPOASES

UnicycleModel::UnicycleModel(real_t timeStep) : dt(timeStep) {}
int_t UnicycleModel::stateDimension() const { return 4; }
int_t UnicycleModel::controlDimension() const { return 2; }
int_t UnicycleModel::positionDimension() const { return 2; }

void UnicycleModel::dynamics(
    const real_t* x, const real_t* u, real_t* xNext
) const
{
    const real_t heading = x[2];
    const real_t speed = x[3];
    xNext[0] = x[0] + dt * speed * std::cos(heading);
    xNext[1] = x[1] + dt * speed * std::sin(heading);
    xNext[2] = x[2] + dt * u[1];
    xNext[3] = x[3] + dt * u[0];
}

void UnicycleModel::linearizeDynamics(
    const real_t* x, const real_t*, real_t* A, real_t* B
) const
{
    const real_t heading = x[2];
    const real_t speed = x[3];
    int_t i;
    for (i = 0; i < 16; ++i) A[i] = 0.0;
    for (i = 0; i < 8; ++i) B[i] = 0.0;

    A[0] = 1.0;
    A[2] = -dt * speed * std::sin(heading);
    A[3] = dt * std::cos(heading);
    A[5] = 1.0;
    A[6] = dt * speed * std::cos(heading);
    A[7] = dt * std::sin(heading);
    A[10] = 1.0;
    A[15] = 1.0;
    B[5] = dt;
    B[6] = dt;
}

void UnicycleModel::position(const real_t* x, real_t* p) const
{
    p[0] = x[0];
    p[1] = x[1];
}

void UnicycleModel::linearizePosition(const real_t*, real_t* C) const
{
    int_t i;
    for (i = 0; i < 8; ++i) C[i] = 0.0;
    C[0] = 1.0;
    C[5] = 1.0;
}

real_t UnicycleModel::timeStep() const { return dt; }

END_NAMESPACE_QPOASES
