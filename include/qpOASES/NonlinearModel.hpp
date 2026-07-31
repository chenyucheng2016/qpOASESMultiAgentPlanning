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

END_NAMESPACE_QPOASES
#endif
