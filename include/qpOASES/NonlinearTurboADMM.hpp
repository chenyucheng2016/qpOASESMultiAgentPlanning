#ifndef QPOASES_NONLINEAR_TURBO_ADMM_HPP
#define QPOASES_NONLINEAR_TURBO_ADMM_HPP

#include <qpOASES/NonlinearModel.hpp>
#include <string>
#include <vector>

BEGIN_NAMESPACE_QPOASES

enum NonlinearCoordinationMethod
{
    NCM_DISTRIBUTED_ADMM = 0,
    NCM_CENTRALIZED_SCP
};

/**
 * Static convex polygon in the world x-y plane. Vertices are stored as
 * [x0, y0, x1, y1, ...] in clockwise or counter-clockwise boundary order.
 * For 3D agents the polygon is treated as a vertically extruded prism.
 */
struct ConvexPolygonObstacle
{
    std::vector<real_t> vertices;
};

struct NonlinearAgentProblem
{
    const NonlinearModel* model;
    int_t horizon;
    /** Negative uses half of NonlinearTurboOptions::safetyDistance. */
    real_t collisionRadius;
    /** Negative uses NonlinearTurboOptions::obstacleSafetyDistance. */
    real_t obstacleSafetyDistance;
    std::vector<real_t> initialState;
    std::vector<real_t> stateReference;
    std::vector<real_t> controlReference;
    std::vector<real_t> stateWeights;
    std::vector<real_t> terminalWeights;
    std::vector<real_t> controlWeights;
    std::vector<real_t> stateLowerBounds;
    std::vector<real_t> stateUpperBounds;
    std::vector<real_t> controlLowerBounds;
    std::vector<real_t> controlUpperBounds;
    std::vector<real_t> initialControls;
    NonlinearAgentProblem();
};

struct NonlinearTurboOptions
{
    NonlinearCoordinationMethod coordinationMethod;
    int_t maxScpIterations;
    int_t maxAdmmIterations;
    int_t maxWorkingSetRecalculations;
    int_t maxLineSearchSteps;
    real_t rho;
    real_t safetyDistance;
    real_t obstacleSafetyDistance;
    real_t controlTrustRegion;
    real_t admmPrimalTolerance;
    real_t admmDualTolerance;
    real_t scpStepTolerance;
    real_t collisionTolerance;
    real_t meritPenalty;
    NonlinearTurboOptions();
};

struct NonlinearTrajectory
{
    std::vector<real_t> states;
    std::vector<real_t> controls;
};

struct NonlinearTurboStatistics
{
    int_t scpIterations;
    int_t admmIterations;
    int_t qpSolves;
    int_t qpWorkingSetRecalculations;
    int_t coldStarts;
    int_t matrixHotstarts;
    int_t vectorHotstarts;
    real_t primalResidual;
    real_t dualResidual;
    real_t minimumDistance;
    real_t minimumPairwiseClearance;
    real_t minimumObstacleDistance;
    real_t minimumObstacleClearance;
    real_t maximumDynamicsDefect;
    real_t objective;
    real_t solveTimeMilliseconds;
    NonlinearTurboStatistics();
};

struct NonlinearTurboResult
{
    bool success;
    bool converged;
    std::string status;
    std::vector<NonlinearTrajectory> trajectories;
    NonlinearTurboStatistics statistics;
    NonlinearTurboResult();
};

/** Nonlinear agent-agent and convex-polygon avoidance with SCP hotstarts. */
class NonlinearTurboADMM
{
public:
    NonlinearTurboResult solve(
        const std::vector<NonlinearAgentProblem>& agents,
        const NonlinearTurboOptions& options = NonlinearTurboOptions()
    ) const;
    NonlinearTurboResult solve(
        const std::vector<NonlinearAgentProblem>& agents,
        const std::vector<ConvexPolygonObstacle>& obstacles,
        const NonlinearTurboOptions& options = NonlinearTurboOptions()
    ) const;
};

END_NAMESPACE_QPOASES
#endif
