/* Nonlinear dynamics interfaces used by TurboSCP. */
#ifndef QPOASES_NONLINEAR_MODEL_HPP
#define QPOASES_NONLINEAR_MODEL_HPP

#include <qpOASES/Types.hpp>

BEGIN_NAMESPACE_QPOASES

/** Discrete nonlinear dynamics and collision-position output. Row-major Jacobians. */
class NonlinearModel
{
public:
    virtual ~NonlinearModel() {}
    virtual int_t stateDimension() const = 0;
    virtual int_t controlDimension() const = 0;
    virtual int_t positionDimension() const = 0;
    virtual void dynamics(const real_t* x, const real_t* u, real_t* xNext) const = 0;
    virtual void linearizeDynamics(
        const real_t* x, const real_t* u, real_t* A, real_t* B
    ) const = 0;
    virtual void position(const real_t* x, real_t* p) const = 0;
    virtual void linearizePosition(const real_t* x, real_t* C) const = 0;
};

/**
 * Forward-Euler unicycle. State is [px, py, heading, speed] and control is
 * [acceleration, yaw_rate].
 */
class UnicycleModel : public NonlinearModel
{
public:
    explicit UnicycleModel(real_t timeStep);
    int_t stateDimension() const;
    int_t controlDimension() const;
    int_t positionDimension() const;
    void dynamics(const real_t* x, const real_t* u, real_t* xNext) const;
    void linearizeDynamics(
        const real_t* x, const real_t* u, real_t* A, real_t* B
    ) const;
    void position(const real_t* x, real_t* p) const;
    void linearizePosition(const real_t* x, real_t* C) const;
    real_t timeStep() const;

private:
    real_t dt;
};

/** Kinematic bicycle with acceleration and steering-rate inputs. */
class BicycleModel : public NonlinearModel
{
public:
    BicycleModel(real_t timeStep, real_t wheelbase);
    int_t stateDimension() const;
    int_t controlDimension() const;
    int_t positionDimension() const;
    void dynamics(const real_t* x, const real_t* u, real_t* xNext) const;
    void linearizeDynamics(
        const real_t* x, const real_t* u, real_t* A, real_t* B
    ) const;
    void position(const real_t* x, real_t* p) const;
    void linearizePosition(const real_t* x, real_t* C) const;
    real_t timeStep() const;
    real_t wheelbaseLength() const;

private:
    real_t dt;
    real_t wheelbase;
};

/**
 * Reduced-order 3D quadcopter. State is
 * [px, py, pz, vx, vy, vz, roll, pitch, yaw] and control is
 * [collective_thrust, roll_rate, pitch_rate, yaw_rate].
 */
class QuadcopterModel : public NonlinearModel
{
public:
    QuadcopterModel(
        real_t timeStep,
        real_t mass = 1.0,
        real_t gravity = 9.81
    );
    int_t stateDimension() const;
    int_t controlDimension() const;
    int_t positionDimension() const;
    void dynamics(const real_t* x, const real_t* u, real_t* xNext) const;
    void linearizeDynamics(
        const real_t* x, const real_t* u, real_t* A, real_t* B
    ) const;
    void position(const real_t* x, real_t* p) const;
    void linearizePosition(const real_t* x, real_t* C) const;
    real_t timeStep() const;
    real_t vehicleMass() const;
    real_t gravityAcceleration() const;
    real_t hoverThrust() const;

private:
    real_t dt;
    real_t mass;
    real_t gravity;
};

END_NAMESPACE_QPOASES
#endif
