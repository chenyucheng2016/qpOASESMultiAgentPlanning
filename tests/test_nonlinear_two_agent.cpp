#include <qpOASES/NonlinearTurboADMM.hpp>
#include <cmath>
#include <cstdio>
#include <vector>
USING_NAMESPACE_QPOASES

namespace
{
NonlinearAgentProblem makeAgent(const UnicycleModel& model, bool reverse)
{
    const int_t N = 30;
    NonlinearAgentProblem p; p.model = &model; p.horizon = N;
    p.initialState.resize(4); p.initialState[0] = reverse ? 4.0 : -4.0;
    p.initialState[1] = 0.0; p.initialState[2] = reverse ? 3.14159265358979323846 : 0.0;
    p.initialState[3] = 1.4;
    p.stateReference.assign((N + 1) * 4, 0.0);
    p.controlReference.assign(N * 2, 0.0); p.initialControls.assign(N * 2, 0.0);
    for (int_t k = 0; k <= N; ++k)
    {
        const real_t fraction = static_cast<real_t>(k) / N;
        p.stateReference[k * 4] = reverse ? 4.0 - 8.0 * fraction : -4.0 + 8.0 * fraction;
        p.stateReference[k * 4 + 2] = reverse ? 3.14159265358979323846 : 0.0;
        p.stateReference[k * 4 + 3] = 1.4;
        if (k < 7) p.initialControls[k * 2 + 1] = 0.45;
        else if (k < 14) p.initialControls[k * 2 + 1] = -0.45;
    }
    const real_t q[] = {2.0, 2.0, 0.15, 0.1};
    const real_t qn[] = {20.0, 20.0, 1.0, 0.5};
    const real_t r[] = {0.1, 0.25};
    p.stateWeights.assign(q, q + 4); p.terminalWeights.assign(qn, qn + 4); p.controlWeights.assign(r, r + 2);
    const real_t xl[] = {-20.0, -20.0, -10.0, 0.0};
    const real_t xu[] = {20.0, 20.0, 10.0, 3.0};
    const real_t ul[] = {-2.0, -1.0}; const real_t uu[] = {2.0, 1.0};
    p.stateLowerBounds.assign(xl, xl + 4); p.stateUpperBounds.assign(xu, xu + 4);
    p.controlLowerBounds.assign(ul, ul + 2); p.controlUpperBounds.assign(uu, uu + 2);
    return p;
}

real_t finalError(const NonlinearAgentProblem& p, const NonlinearTrajectory& t)
{
    const int_t k = p.horizon * 4;
    const real_t dx = t.states[k] - p.stateReference[k];
    const real_t dy = t.states[k + 1] - p.stateReference[k + 1];
    return std::sqrt(dx * dx + dy * dy);
}

bool runMethod(NonlinearCoordinationMethod method, const std::vector<NonlinearAgentProblem>& agents)
{
    NonlinearTurboOptions options; options.coordinationMethod = method;
    options.safetyDistance = 1.0; options.rho = 40.0; options.maxScpIterations = 8;
    options.maxAdmmIterations = 50; options.controlTrustRegion = 0.8;
    options.admmPrimalTolerance = 2.0e-3; options.admmDualTolerance = 2.0e-3;
    NonlinearTurboADMM solver; const NonlinearTurboResult result = solver.solve(agents, options);
    const real_t e0 = finalError(agents[0], result.trajectories[0]);
    const real_t e1 = finalError(agents[1], result.trajectories[1]);
    std::printf("%s: %s, %.2f ms, SCP %d, ADMM %d, QPs %d, distance %.3f, errors %.3f/%.3f\n",
        method == NCM_DISTRIBUTED_ADMM ? "distributed" : "centralized", result.status.c_str(),
        result.statistics.solveTimeMilliseconds, result.statistics.scpIterations,
        result.statistics.admmIterations, result.statistics.qpSolves,
        result.statistics.minimumDistance, e0, e1);
    return result.success && result.statistics.maximumDynamicsDefect <= 1.0e-10
        && e0 <= 2.0 && e1 <= 2.0
        && (method != NCM_DISTRIBUTED_ADMM || result.statistics.vectorHotstarts > 0);
}
}

int main()
{
    UnicycleModel model(0.2);
    std::vector<NonlinearAgentProblem> agents;
    agents.push_back(makeAgent(model, false)); agents.push_back(makeAgent(model, true));
    return runMethod(NCM_DISTRIBUTED_ADMM, agents) && runMethod(NCM_CENTRALIZED_SCP, agents) ? 0 : 1;
}
