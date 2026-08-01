#ifndef QPOASES_NONLINEAR_TURBO_ADMM_HPP
#define QPOASES_NONLINEAR_TURBO_ADMM_HPP

#include <qpOASES/NonlinearModel.hpp>
#include <string>
#include <vector>

BEGIN_NAMESPACE_QPOASES

enum NonlinearCoordinationMethod
{
    NCM_DISTRIBUTED_ADMM = 0,
    NCM_CENTRALIZED_SCP,
    NCM_CENTRALIZED_OSQP
};

/** Distributed-QP state retained across the nested ADMM/SCP iterations. */
enum NonlinearContinuationMode
{
    NCONT_COLD = 0,
    NCONT_INNER_ADMM,
    NCONT_QP,
    NCONT_FULL
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
    /** Negative uses NonlinearTurboOptions::terminalPositionTolerance. */
    real_t terminalPositionTolerance;
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
    NonlinearContinuationMode continuationMode;
    bool useRiccatiWarmStart;
    bool parallelAgentSolves;
    /** Zero selects the frozen scale-aware policy; positive fixes the team size. */
    int_t parallelAgentThreads;
    int_t collisionSamplesPerInterval;
    int_t maxScpIterations;
    int_t maxAdmmIterations;
    int_t maxWorkingSetRecalculations;
    int_t maxLineSearchSteps;
    real_t rho;
    bool adaptiveRho;
    int_t adaptiveRhoMinimumAgents;
    int_t adaptiveRhoInterval;
    real_t adaptiveRhoImbalance;
    real_t adaptiveRhoScale;
    real_t minimumRho;
    real_t maximumRho;
    int_t inexactAdmmScpIterations;
    real_t inexactAdmmToleranceMultiplier;
    real_t safetyDistance;
    real_t obstacleSafetyDistance;
    real_t controlTrustRegion;
    real_t admmPrimalTolerance;
    real_t admmDualTolerance;
    real_t admmRelativeTolerance;
    real_t admmRelaxation;
    real_t scpStepTolerance;
    real_t collisionTolerance;
    real_t terminalPositionTolerance;
    real_t meritPenalty;
    /** Negative keeps the complete pair graph; nonnegative activates proximity screening. */
    real_t pairActivationDistance;
    /** Negative keeps every obstacle in every local QP. */
    real_t obstacleActivationDistance;
    /** Minimum dot product between old and new collision normals for transport. */
    real_t continuationMinimumNormalDot;
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
    int_t backendIterations;
    int_t coldStarts;
    int_t matrixHotstarts;
    int_t vectorHotstarts;
    int_t transportedPairStages;
    int_t resetPairStages;
    int_t parallelQpBatches;
    int_t parallelAgentThreads;
    int_t rhoUpdates;
    int_t admmConvergedSubproblems;
    int_t admmIterationLimitSubproblems;
    int_t maximumActivePairs;
    int_t initialActivePairs;
    int_t finalActivePairs;
    int_t maximumPotentialPairs;
    int_t maximumAgentDegree;
    int_t maximumActiveObstaclesPerAgent;
    int_t maximumPotentialObstaclesPerAgent;
    int_t maximumLocalQpVariables;
    int_t maximumLocalQpConstraints;
    int_t centralizedQpVariables;
    int_t centralizedQpConstraints;
    real_t primalResidual;
    real_t dualResidual;
    real_t minimumAdmmRho;
    real_t maximumAdmmRho;
    real_t finalAdmmRho;
    real_t minimumDistance;
    real_t primalStoppingThreshold;
    real_t dualStoppingThreshold;
    real_t minimumPairwiseClearance;
    real_t minimumObstacleDistance;
    real_t minimumObstacleClearance;
    real_t maximumDynamicsDefect;
    real_t maximumTerminalPositionError;
    real_t objective;
    real_t solveTimeMilliseconds;
    real_t maximumLocalQpSolveTimeMilliseconds;
    real_t qpBuildTimeMilliseconds;
    real_t pairBuildTimeMilliseconds;
    real_t admmAssemblyTimeMilliseconds;
    real_t localQpBatchTimeMilliseconds;
    real_t localQpSolveTimeMilliseconds;
    real_t consensusTimeMilliseconds;
    real_t globalizationTimeMilliseconds;
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
