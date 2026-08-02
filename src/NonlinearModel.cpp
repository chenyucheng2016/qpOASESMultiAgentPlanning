#include <qpOASES/NonlinearModel.hpp>

#include <cmath>

BEGIN_NAMESPACE_QPOASES

void NonlinearModel::collisionPoint(
    const real_t* x,
    real_t longitudinalOffset,
    real_t lateralOffset,
    real_t* p
) const
{
    position(x, p);
    if (positionDimension() >= 1) p[0] += longitudinalOffset;
    if (positionDimension() >= 2) p[1] += lateralOffset;
}

void NonlinearModel::linearizeCollisionPoint(
    const real_t* x,
    real_t,
    real_t,
    real_t* C
) const
{
    linearizePosition(x, C);
}

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

BicycleModel::BicycleModel(real_t timeStep, real_t wheelbaseLength)
    : dt(timeStep), wheelbase(wheelbaseLength) {}
int_t BicycleModel::stateDimension() const { return 5; }
int_t BicycleModel::controlDimension() const { return 2; }
int_t BicycleModel::positionDimension() const { return 2; }

void BicycleModel::dynamics(
    const real_t* x, const real_t* u, real_t* xNext
) const
{
    const real_t heading = x[2];
    const real_t speed = x[3];
    const real_t steering = x[4];
    xNext[0] = x[0] + dt * speed * std::cos(heading);
    xNext[1] = x[1] + dt * speed * std::sin(heading);
    xNext[2] = x[2] + dt * speed * std::tan(steering) / wheelbase;
    xNext[3] = x[3] + dt * u[0];
    xNext[4] = x[4] + dt * u[1];
}

void BicycleModel::linearizeDynamics(
    const real_t* x, const real_t*, real_t* A, real_t* B
) const
{
    const real_t heading = x[2];
    const real_t speed = x[3];
    const real_t steering = x[4];
    const real_t cosineSteering = std::cos(steering);
    for (int_t i = 0; i < 25; ++i) A[i] = 0.0;
    for (int_t i = 0; i < 10; ++i) B[i] = 0.0;
    for (int_t i = 0; i < 5; ++i) A[i * 5 + i] = 1.0;
    A[2] = -dt * speed * std::sin(heading);
    A[3] = dt * std::cos(heading);
    A[7] = dt * speed * std::cos(heading);
    A[8] = dt * std::sin(heading);
    A[13] = dt * std::tan(steering) / wheelbase;
    A[14] = dt * speed
        / (wheelbase * cosineSteering * cosineSteering);
    B[6] = dt;
    B[9] = dt;
}

void BicycleModel::position(const real_t* x, real_t* p) const
{
    p[0] = x[0];
    p[1] = x[1];
}

void BicycleModel::linearizePosition(const real_t*, real_t* C) const
{
    for (int_t i = 0; i < 10; ++i) C[i] = 0.0;
    C[0] = 1.0;
    C[6] = 1.0;
}

real_t BicycleModel::timeStep() const { return dt; }
real_t BicycleModel::wheelbaseLength() const { return wheelbase; }

FrontSteeringModel::FrontSteeringModel(
    real_t timeStep,
    real_t wheelbaseLength
) : dt(timeStep), wheelbase(wheelbaseLength) {}

int_t FrontSteeringModel::stateDimension() const { return 4; }
int_t FrontSteeringModel::controlDimension() const { return 2; }
int_t FrontSteeringModel::positionDimension() const { return 2; }

void FrontSteeringModel::dynamics(
    const real_t* x,
    const real_t* u,
    real_t* xNext
) const
{
    const real_t heading = x[2];
    const real_t steering = x[3];
    const real_t speed = u[0];
    xNext[0] = x[0] + dt * speed * std::cos(heading);
    xNext[1] = x[1] + dt * speed * std::sin(heading);
    xNext[2] = x[2] + dt * speed * std::tan(steering) / wheelbase;
    xNext[3] = x[3] + dt * u[1];
}

void FrontSteeringModel::linearizeDynamics(
    const real_t* x,
    const real_t* u,
    real_t* A,
    real_t* B
) const
{
    const real_t heading = x[2];
    const real_t steering = x[3];
    const real_t speed = u[0];
    const real_t cosineSteering = std::cos(steering);
    for (int_t i = 0; i < 16; ++i) A[i] = 0.0;
    for (int_t i = 0; i < 8; ++i) B[i] = 0.0;
    for (int_t i = 0; i < 4; ++i) A[i * 4 + i] = 1.0;
    A[2] = -dt * speed * std::sin(heading);
    A[6] = dt * speed * std::cos(heading);
    A[11] = dt * speed
        / (wheelbase * cosineSteering * cosineSteering);
    B[0] = dt * std::cos(heading);
    B[2] = dt * std::sin(heading);
    B[4] = dt * std::tan(steering) / wheelbase;
    B[7] = dt;
}

void FrontSteeringModel::position(const real_t* x, real_t* p) const
{
    p[0] = x[0];
    p[1] = x[1];
}

void FrontSteeringModel::linearizePosition(const real_t*, real_t* C) const
{
    for (int_t i = 0; i < 8; ++i) C[i] = 0.0;
    C[0] = 1.0;
    C[5] = 1.0;
}

void FrontSteeringModel::collisionPoint(
    const real_t* x,
    real_t longitudinalOffset,
    real_t lateralOffset,
    real_t* p
) const
{
    const real_t cosine = std::cos(x[2]);
    const real_t sine = std::sin(x[2]);
    p[0] = x[0] + cosine * longitudinalOffset - sine * lateralOffset;
    p[1] = x[1] + sine * longitudinalOffset + cosine * lateralOffset;
}

void FrontSteeringModel::linearizeCollisionPoint(
    const real_t* x,
    real_t longitudinalOffset,
    real_t lateralOffset,
    real_t* C
) const
{
    const real_t cosine = std::cos(x[2]);
    const real_t sine = std::sin(x[2]);
    for (int_t i = 0; i < 8; ++i) C[i] = 0.0;
    C[0] = 1.0;
    C[2] = -sine * longitudinalOffset - cosine * lateralOffset;
    C[5] = 1.0;
    C[6] = cosine * longitudinalOffset - sine * lateralOffset;
}

real_t FrontSteeringModel::timeStep() const { return dt; }
real_t FrontSteeringModel::wheelbaseLength() const { return wheelbase; }

QuadcopterModel::QuadcopterModel(
    real_t timeStep, real_t vehicleMassValue, real_t gravityValue
) : dt(timeStep), mass(vehicleMassValue), gravity(gravityValue) {}
int_t QuadcopterModel::stateDimension() const { return 9; }
int_t QuadcopterModel::controlDimension() const { return 4; }
int_t QuadcopterModel::positionDimension() const { return 3; }

void QuadcopterModel::dynamics(
    const real_t* x, const real_t* u, real_t* xNext
) const
{
    const real_t roll = x[6];
    const real_t pitch = x[7];
    const real_t yaw = x[8];
    const real_t cRoll = std::cos(roll);
    const real_t sRoll = std::sin(roll);
    const real_t cPitch = std::cos(pitch);
    const real_t sPitch = std::sin(pitch);
    const real_t cYaw = std::cos(yaw);
    const real_t sYaw = std::sin(yaw);
    const real_t acceleration = u[0] / mass;
    const real_t bodyZx = cRoll * sPitch * cYaw + sRoll * sYaw;
    const real_t bodyZy = cRoll * sPitch * sYaw - sRoll * cYaw;
    const real_t bodyZz = cRoll * cPitch;

    xNext[0] = x[0] + dt * x[3];
    xNext[1] = x[1] + dt * x[4];
    xNext[2] = x[2] + dt * x[5];
    xNext[3] = x[3] + dt * acceleration * bodyZx;
    xNext[4] = x[4] + dt * acceleration * bodyZy;
    xNext[5] = x[5] + dt * (acceleration * bodyZz - gravity);
    xNext[6] = x[6] + dt * u[1];
    xNext[7] = x[7] + dt * u[2];
    xNext[8] = x[8] + dt * u[3];
}

void QuadcopterModel::linearizeDynamics(
    const real_t* x, const real_t* u, real_t* A, real_t* B
) const
{
    const real_t roll = x[6];
    const real_t pitch = x[7];
    const real_t yaw = x[8];
    const real_t cRoll = std::cos(roll);
    const real_t sRoll = std::sin(roll);
    const real_t cPitch = std::cos(pitch);
    const real_t sPitch = std::sin(pitch);
    const real_t cYaw = std::cos(yaw);
    const real_t sYaw = std::sin(yaw);
    const real_t acceleration = u[0] / mass;
    const real_t bodyZ[3] = {
        cRoll * sPitch * cYaw + sRoll * sYaw,
        cRoll * sPitch * sYaw - sRoll * cYaw,
        cRoll * cPitch
    };
    const real_t derivativeRoll[3] = {
        -sRoll * sPitch * cYaw + cRoll * sYaw,
        -sRoll * sPitch * sYaw - cRoll * cYaw,
        -sRoll * cPitch
    };
    const real_t derivativePitch[3] = {
        cRoll * cPitch * cYaw,
        cRoll * cPitch * sYaw,
        -cRoll * sPitch
    };
    const real_t derivativeYaw[3] = {
        -cRoll * sPitch * sYaw + sRoll * cYaw,
        cRoll * sPitch * cYaw + sRoll * sYaw,
        0.0
    };

    for (int_t i = 0; i < 81; ++i) A[i] = 0.0;
    for (int_t i = 0; i < 36; ++i) B[i] = 0.0;
    for (int_t i = 0; i < 9; ++i) A[i * 9 + i] = 1.0;
    A[3] = dt;
    A[13] = dt;
    A[23] = dt;
    for (int_t axis = 0; axis < 3; ++axis)
    {
        const int_t row = 3 + axis;
        A[row * 9 + 6] = dt * acceleration * derivativeRoll[axis];
        A[row * 9 + 7] = dt * acceleration * derivativePitch[axis];
        A[row * 9 + 8] = dt * acceleration * derivativeYaw[axis];
        B[row * 4] = dt * bodyZ[axis] / mass;
    }
    B[25] = dt;
    B[30] = dt;
    B[35] = dt;
}

void QuadcopterModel::position(const real_t* x, real_t* p) const
{
    p[0] = x[0];
    p[1] = x[1];
    p[2] = x[2];
}

void QuadcopterModel::linearizePosition(const real_t*, real_t* C) const
{
    for (int_t i = 0; i < 27; ++i) C[i] = 0.0;
    C[0] = 1.0;
    C[10] = 1.0;
    C[20] = 1.0;
}

real_t QuadcopterModel::timeStep() const { return dt; }
real_t QuadcopterModel::vehicleMass() const { return mass; }
real_t QuadcopterModel::gravityAcceleration() const { return gravity; }
real_t QuadcopterModel::hoverThrust() const { return mass * gravity; }

END_NAMESPACE_QPOASES
