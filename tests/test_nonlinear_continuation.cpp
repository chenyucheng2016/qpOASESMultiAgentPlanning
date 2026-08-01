#include <qpOASES/NonlinearTrajectoryValidator.hpp>

#include <cmath>
#include <cstdio>
#include <vector>

USING_NAMESPACE_QPOASES

namespace
{

NonlinearAgentProblem makeAgent(const UnicycleModel& model, bool reverse)
{
    const int_t horizon = 30;
    NonlinearAgentProblem problem;
    problem.model = &model;
    problem.horizon = horizon;
    problem.collisionRadius = reverse ? 0.85 : 0.35;
    problem.initialState.resize(4);
    problem.initialState[0] = reverse ? 4.0 : -4.0;
    problem.initialState[1] = 0.0;
    problem.initialState[2] = reverse ? 3.14159265358979323846 : 0.0;
    problem.initialState[3] = 1.4;
    problem.stateReference.assign((horizon + 1) * 4, 0.0);
    problem.controlReference.assign(horizon * 2, 0.0);
    problem.initialControls.assign(horizon * 2, 0.0);
    for (int_t k = 0; k <= horizon; ++k)
    {
        const real_t fraction = static_cast<real_t>(k) / horizon;
        problem.stateReference[k * 4] = reverse
            ? 4.0 - 8.0 * fraction : -4.0 + 8.0 * fraction;
        problem.stateReference[k * 4 + 2] = reverse
            ? 3.14159265358979323846 : 0.0;
        problem.stateReference[k * 4 + 3] = 1.4;
        if (k < 7) problem.initialControls[k * 2 + 1] = 0.45;
        else if (k < 14) problem.initialControls[k * 2 + 1] = -0.45;
    }
    const real_t stateWeights[] = {2.0, 2.0, 0.15, 0.1};
    const real_t terminalWeights[] = {20.0, 20.0, 1.0, 0.5};
    const real_t controlWeights[] = {0.1, 0.25};
    const real_t stateLower[] = {-20.0, -20.0, -10.0, 0.0};
    const real_t stateUpper[] = {20.0, 20.0, 10.0, 3.0};
    const real_t controlLower[] = {-2.0, -1.0};
    const real_t controlUpper[] = {2.0, 1.0};
    problem.stateWeights.assign(stateWeights, stateWeights + 4);
    problem.terminalWeights.assign(terminalWeights, terminalWeights + 4);
    problem.controlWeights.assign(controlWeights, controlWeights + 2);
    problem.stateLowerBounds.assign(stateLower, stateLower + 4);
    problem.stateUpperBounds.assign(stateUpper, stateUpper + 4);
    problem.controlLowerBounds.assign(controlLower, controlLower + 2);
    problem.controlUpperBounds.assign(controlUpper, controlUpper + 2);
    return problem;
}

NonlinearTurboOptions makeOptions(NonlinearContinuationMode mode, bool parallel)
{
    NonlinearTurboOptions options;
    options.coordinationMethod = NCM_DISTRIBUTED_ADMM;
    options.continuationMode = mode;
    options.parallelAgentSolves = parallel;
    options.safetyDistance = 1.0;
    options.rho = 40.0;
    options.maxScpIterations = 8;
    options.maxAdmmIterations = 50;
    options.controlTrustRegion = 0.8;
    options.admmPrimalTolerance = 2.0e-3;
    options.admmDualTolerance = 2.0e-3;
    return options;
}

bool validateResult(
    const std::vector<NonlinearAgentProblem>& agents,
    const NonlinearTurboOptions& options,
    const NonlinearTurboResult& result)
{
    NonlinearValidationOptions validation;
    validation.interpolationSubsteps = 20;
    validation.terminalPositionTolerance = 2.0;
    const NonlinearValidationResult checked = validateNonlinearTrajectories(
        agents,
        std::vector<ConvexPolygonObstacle>(),
        result.trajectories,
        options,
        validation
    );
    if (!result.success || !checked.success)
    {
        std::printf("validation failed: solver=%s validator=%s pair=%.6f obstacle=%.6f defect=%.3e terminal=%.6f\n",
            result.status.c_str(), checked.status.c_str(), checked.minimumPairwiseClearance,
            checked.minimumObstacleClearance, checked.maximumDynamicsDefect,
            checked.maximumTerminalPositionError);
        return false;
    }
    return true;
}

bool modesHaveExpectedStateReuse(const std::vector<NonlinearAgentProblem>& agents)
{
    const NonlinearTurboOptions coldOptions = makeOptions(NCONT_COLD, false);
    const NonlinearTurboResult cold = NonlinearTurboADMM().solve(agents, coldOptions);
    if (!validateResult(agents, coldOptions, cold)
        || cold.statistics.coldStarts != cold.statistics.qpSolves
        || cold.statistics.vectorHotstarts != 0
        || cold.statistics.matrixHotstarts != 0
        || cold.statistics.transportedPairStages != 0)
        return false;

    const NonlinearTurboOptions innerOptions = makeOptions(NCONT_INNER_ADMM, false);
    const NonlinearTurboResult inner = NonlinearTurboADMM().solve(agents, innerOptions);
    if (!validateResult(agents, innerOptions, inner)
        || inner.statistics.vectorHotstarts <= 0
        || inner.statistics.matrixHotstarts != 0
        || inner.statistics.transportedPairStages != 0)
        return false;

    const NonlinearTurboOptions qpOptions = makeOptions(NCONT_QP, false);
    const NonlinearTurboResult qp = NonlinearTurboADMM().solve(agents, qpOptions);
    if (!validateResult(agents, qpOptions, qp)
        || qp.statistics.matrixHotstarts <= 0
        || qp.statistics.transportedPairStages != 0)
        return false;

    const NonlinearTurboOptions fullOptions = makeOptions(NCONT_FULL, false);
    const NonlinearTurboResult full = NonlinearTurboADMM().solve(agents, fullOptions);
    if (!validateResult(agents, fullOptions, full)
        || full.statistics.matrixHotstarts <= 0
        || full.statistics.transportedPairStages <= 0)
        return false;

    std::printf(
        "continuation QP work: cold=%d inner=%d qp=%d full=%d, transported=%d reset=%d\n",
        cold.statistics.qpWorkingSetRecalculations,
        inner.statistics.qpWorkingSetRecalculations,
        qp.statistics.qpWorkingSetRecalculations,
        full.statistics.qpWorkingSetRecalculations,
        full.statistics.transportedPairStages,
        full.statistics.resetPairStages
    );
    return true;
}

bool parallelMatchesSerial(const std::vector<NonlinearAgentProblem>& agents)
{
    const NonlinearTurboOptions serialOptions = makeOptions(NCONT_FULL, false);
    const NonlinearTurboOptions parallelOptions = makeOptions(NCONT_FULL, true);
    const NonlinearTurboResult serial = NonlinearTurboADMM().solve(agents, serialOptions);
    const NonlinearTurboResult parallel = NonlinearTurboADMM().solve(agents, parallelOptions);
    return validateResult(agents, serialOptions, serial)
        && validateResult(agents, parallelOptions, parallel)
        && parallel.statistics.parallelQpBatches > 0
        && std::fabs(serial.statistics.objective - parallel.statistics.objective) <= 1.0e-8
        && std::fabs(serial.statistics.minimumPairwiseClearance
            - parallel.statistics.minimumPairwiseClearance) <= 1.0e-10;
}

bool validatorDetectsCorruption(const std::vector<NonlinearAgentProblem>& agents)
{
    const NonlinearTurboOptions options = makeOptions(NCONT_FULL, false);
    const NonlinearTurboResult result = NonlinearTurboADMM().solve(agents, options);
    if (!validateResult(agents, options, result)) return false;
    std::vector<NonlinearTrajectory> corrupted = result.trajectories;
    corrupted[0].states[4] += 0.25;
    NonlinearValidationOptions validation;
    validation.terminalPositionTolerance = 2.0;
    const NonlinearValidationResult checked = validateNonlinearTrajectories(
        agents,
        std::vector<ConvexPolygonObstacle>(),
        corrupted,
        options,
        validation
    );
    return !checked.success
        && checked.maximumDynamicsDefect > validation.dynamicsTolerance;
}

}

int main()
{
    UnicycleModel model(0.2);
    std::vector<NonlinearAgentProblem> agents;
    agents.push_back(makeAgent(model, false));
    agents.push_back(makeAgent(model, true));
    return modesHaveExpectedStateReuse(agents)
        && parallelMatchesSerial(agents)
        && validatorDetectsCorruption(agents) ? 0 : 1;
}
