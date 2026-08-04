#ifndef QPOASES_NONLINEAR_TRAJECTORY_VALIDATOR_HPP
#define QPOASES_NONLINEAR_TRAJECTORY_VALIDATOR_HPP

#include <qpOASES/NonlinearTurboADMM.hpp>

BEGIN_NAMESPACE_QPOASES

struct NonlinearValidationOptions
{
    int_t interpolationSubsteps;
    real_t dynamicsTolerance;
    real_t terminalPositionTolerance;
    real_t terminalStateTolerance;
    NonlinearValidationOptions();
};

struct NonlinearValidationResult
{
    bool success;
    std::string status;
    real_t minimumPairwiseClearance;
    real_t minimumObstacleClearance;
    real_t maximumDynamicsDefect;
    real_t maximumTerminalPositionError;
    real_t maximumTerminalStateError;
    NonlinearValidationResult();
};

/** Independent post-solve checks on nonlinear rollout and interpolated geometry. */
NonlinearValidationResult validateNonlinearTrajectories(
    const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<ConvexPolygonObstacle>& obstacles,
    const std::vector<NonlinearTrajectory>& trajectories,
    const NonlinearTurboOptions& solverOptions,
    const NonlinearValidationOptions& validationOptions = NonlinearValidationOptions()
);

END_NAMESPACE_QPOASES
#endif
