#include <qpOASES/NonlinearTurboADMM.hpp>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

USING_NAMESPACE_QPOASES

namespace
{

NonlinearAgentProblem makeUnicycle(const UnicycleModel& model, int_t horizon)
{
    NonlinearAgentProblem p;
    p.model = &model;
    p.horizon = horizon;
    const real_t initial[] = {-2.5, 0.0, 0.0, 1.25};
    p.initialState.assign(initial, initial + 4);
    p.stateReference.assign((horizon + 1) * 4, 0.0);
    p.controlReference.assign(horizon * 2, 0.0);
    p.initialControls.assign(horizon * 2, 0.0);
    for (int_t k = 0; k <= horizon; ++k)
    {
        const real_t alpha = static_cast<real_t>(k) / horizon;
        p.stateReference[k * 4] = -2.5 + 5.0 * alpha;
        p.stateReference[k * 4 + 3] = 1.25;
        if (k < 4) p.initialControls[k * 2 + 1] = 0.55;
        else if (k < 8) p.initialControls[k * 2 + 1] = -0.55;
    }
    const real_t q[] = {2.0, 2.0, 0.2, 0.1};
    const real_t qTerminal[] = {15.0, 15.0, 0.8, 0.3};
    const real_t r[] = {0.1, 0.25};
    const real_t lowerState[] = {-10.0, -10.0, -7.0, 0.0};
    const real_t upperState[] = {10.0, 10.0, 7.0, 3.0};
    const real_t lowerControl[] = {-2.0, -1.0};
    const real_t upperControl[] = {2.0, 1.0};
    p.stateWeights.assign(q, q + 4);
    p.terminalWeights.assign(qTerminal, qTerminal + 4);
    p.controlWeights.assign(r, r + 2);
    p.stateLowerBounds.assign(lowerState, lowerState + 4);
    p.stateUpperBounds.assign(upperState, upperState + 4);
    p.controlLowerBounds.assign(lowerControl, lowerControl + 2);
    p.controlUpperBounds.assign(upperControl, upperControl + 2);
    return p;
}

NonlinearAgentProblem makeBicycle(const BicycleModel& model, int_t horizon)
{
    NonlinearAgentProblem p;
    p.model = &model;
    p.horizon = horizon;
    const real_t pi = 3.14159265358979323846;
    const real_t initial[] = {2.5, 0.0, pi, 1.25, 0.0};
    p.initialState.assign(initial, initial + 5);
    p.stateReference.assign((horizon + 1) * 5, 0.0);
    p.controlReference.assign(horizon * 2, 0.0);
    p.initialControls.assign(horizon * 2, 0.0);
    for (int_t k = 0; k <= horizon; ++k)
    {
        const real_t alpha = static_cast<real_t>(k) / horizon;
        p.stateReference[k * 5] = 2.5 - 5.0 * alpha;
        p.stateReference[k * 5 + 2] = pi;
        p.stateReference[k * 5 + 3] = 1.25;
        if (k < 2) p.initialControls[k * 2 + 1] = 0.65;
        else if (k >= 5 && k < 7) p.initialControls[k * 2 + 1] = -0.65;
    }
    const real_t q[] = {2.0, 2.0, 0.2, 0.1, 0.1};
    const real_t qTerminal[] = {15.0, 15.0, 0.8, 0.3, 0.2};
    const real_t r[] = {0.1, 0.25};
    const real_t lowerState[] = {-10.0, -10.0, -7.0, 0.0, -0.6};
    const real_t upperState[] = {10.0, 10.0, 7.0, 3.0, 0.6};
    const real_t lowerControl[] = {-2.0, -1.0};
    const real_t upperControl[] = {2.0, 1.0};
    p.stateWeights.assign(q, q + 5);
    p.terminalWeights.assign(qTerminal, qTerminal + 5);
    p.controlWeights.assign(r, r + 2);
    p.stateLowerBounds.assign(lowerState, lowerState + 5);
    p.stateUpperBounds.assign(upperState, upperState + 5);
    p.controlLowerBounds.assign(lowerControl, lowerControl + 2);
    p.controlUpperBounds.assign(upperControl, upperControl + 2);
    return p;
}

NonlinearAgentProblem makeQuadcopter(
    const QuadcopterModel& model,
    int_t horizon
)
{
    NonlinearAgentProblem p;
    p.model = &model;
    p.horizon = horizon;
    const real_t initial[] = {
        0.0, -2.5, 0.6, 0.0, 1.25, 0.0, 0.0, 0.0, 0.0
    };
    p.initialState.assign(initial, initial + 9);
    p.stateReference.assign((horizon + 1) * 9, 0.0);
    p.controlReference.assign(horizon * 4, 0.0);
    p.initialControls.assign(horizon * 4, 0.0);
    for (int_t k = 0; k <= horizon; ++k)
    {
        const real_t alpha = static_cast<real_t>(k) / horizon;
        p.stateReference[k * 9 + 1] = -2.5 + 5.0 * alpha;
        p.stateReference[k * 9 + 2] = 0.6;
        p.stateReference[k * 9 + 4] = 1.25;
        if (k < horizon)
        {
            p.controlReference[k * 4] = model.hoverThrust();
            p.initialControls[k * 4] = model.hoverThrust();
        }
    }
    const real_t q[] = {
        2.0, 2.0, 3.0, 0.2, 0.2, 0.2, 0.2, 0.2, 0.1
    };
    const real_t qTerminal[] = {
        12.0, 12.0, 15.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.2
    };
    const real_t r[] = {0.03, 0.2, 0.2, 0.1};
    const real_t lowerState[] = {
        -10.0, -10.0, 0.5, -3.0, -3.0, -3.0, -0.6, -0.6, -7.0
    };
    const real_t upperState[] = {
        10.0, 10.0, 4.0, 3.0, 3.0, 3.0, 0.6, 0.6, 7.0
    };
    const real_t lowerControl[] = {0.0, -1.0, -1.0, -1.0};
    const real_t upperControl[] = {20.0, 1.0, 1.0, 1.0};
    p.stateWeights.assign(q, q + 9);
    p.terminalWeights.assign(qTerminal, qTerminal + 9);
    p.controlWeights.assign(r, r + 4);
    p.stateLowerBounds.assign(lowerState, lowerState + 9);
    p.stateUpperBounds.assign(upperState, upperState + 9);
    p.controlLowerBounds.assign(lowerControl, lowerControl + 4);
    p.controlUpperBounds.assign(upperControl, upperControl + 4);
    return p;
}

real_t finalPositionError(
    const NonlinearAgentProblem& p,
    const NonlinearTrajectory& trajectory
)
{
    const int_t nx = p.model->stateDimension();
    const int_t np = p.model->positionDimension();
    std::vector<real_t> actual(np, 0.0), reference(np, 0.0);
    p.model->position(&trajectory.states[p.horizon * nx], &actual[0]);
    p.model->position(&p.stateReference[p.horizon * nx], &reference[0]);
    real_t squared = 0.0;
    for (int_t i = 0; i < np; ++i)
        squared += (actual[i] - reference[i]) * (actual[i] - reference[i]);
    return std::sqrt(squared);
}

bool runMethod(
    NonlinearCoordinationMethod method,
    const std::vector<NonlinearAgentProblem>& agents,
    std::FILE* csv
)
{
    NonlinearTurboOptions options;
    options.coordinationMethod = method;
    options.safetyDistance = 0.8;
    options.rho = 35.0;
    options.maxScpIterations = 8;
    options.maxAdmmIterations = 50;
    options.controlTrustRegion = 1.0;
    options.admmPrimalTolerance = 3.0e-3;
    options.admmDualTolerance = 3.0e-3;
    const NonlinearTurboResult result = NonlinearTurboADMM().solve(
        agents,
        options
    );
    if (result.trajectories.size() != agents.size()) return false;
    std::printf(
        "%s heterogeneous: %s, %.2f ms, distance %.3f, defects %.3e",
        method == NCM_DISTRIBUTED_ADMM ? "distributed" : "centralized",
        result.status.c_str(),
        result.statistics.solveTimeMilliseconds,
        result.statistics.minimumDistance,
        result.statistics.maximumDynamicsDefect
    );
    bool trackingPassed = true;
    std::vector<real_t> finalErrors(agents.size(), 0.0);
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        const real_t error = finalPositionError(agents[a], result.trajectories[a]);
        finalErrors[a] = error;
        std::printf(", e%u %.3f", static_cast<unsigned>(a), error);
        trackingPassed = trackingPassed && error <= 1.0;
        trackingPassed = trackingPassed
            && result.trajectories[a].states.size()
                == static_cast<std::size_t>((agents[a].horizon + 1)
                    * agents[a].model->stateDimension());
    }
    std::printf("\n");
    if (csv != NULL)
    {
        std::fprintf(
            csv,
            "%s,%d,%d,\"%s\",%.9g,%.9g,%.9g,%.9g,%d,%d,%d,%d,%d,%d,%d,%.9g,%.9g,%.9g\n",
            method == NCM_DISTRIBUTED_ADMM ? "distributed_admm" : "centralized_scp",
            result.success ? 1 : 0,
            result.converged ? 1 : 0,
            result.status.c_str(),
            result.statistics.solveTimeMilliseconds,
            result.statistics.objective,
            result.statistics.minimumDistance,
            result.statistics.maximumDynamicsDefect,
            result.statistics.scpIterations,
            result.statistics.admmIterations,
            result.statistics.qpSolves,
            result.statistics.qpWorkingSetRecalculations,
            result.statistics.coldStarts,
            result.statistics.matrixHotstarts,
            result.statistics.vectorHotstarts,
            finalErrors[0],
            finalErrors[1],
            finalErrors[2]
        );
    }
    return result.success
        && trackingPassed
        && result.statistics.minimumDistance >= 0.79
        && result.statistics.maximumDynamicsDefect <= 1.0e-9;
}

}

int main(int argc, char** argv)
{
    std::FILE* csv = NULL;
    if (argc == 3 && std::string(argv[1]) == "--csv")
    {
        csv = std::fopen(argv[2], "w");
        if (csv == NULL)
        {
            std::fprintf(stderr, "cannot open benchmark CSV: %s\n", argv[2]);
            return 1;
        }
        std::fprintf(
            csv,
            "method,success,converged,status,solve_time_ms,objective,"
            "minimum_distance,maximum_dynamics_defect,scp_iterations,"
            "admm_iterations,qp_solves,qp_working_set_recalculations,"
            "cold_starts,matrix_hotstarts,vector_hotstarts,"
            "unicycle_final_error,bicycle_final_error,quadcopter_final_error\n"
        );
    }
    else if (argc != 1)
    {
        std::fprintf(stderr, "usage: %s [--csv output.csv]\n", argv[0]);
        return 1;
    }

    const int_t horizon = 16;
    UnicycleModel unicycle(0.25);
    BicycleModel bicycle(0.25, 2.0);
    QuadcopterModel quadcopter(0.25, 1.0, 9.81);
    std::vector<NonlinearAgentProblem> agents;
    agents.push_back(makeUnicycle(unicycle, horizon));
    agents.push_back(makeBicycle(bicycle, horizon));
    agents.push_back(makeQuadcopter(quadcopter, horizon));
    const bool distributedPassed = runMethod(NCM_DISTRIBUTED_ADMM, agents, csv);
    const bool centralizedPassed = runMethod(NCM_CENTRALIZED_SCP, agents, csv);
    if (csv != NULL) std::fclose(csv);
    return distributedPassed && centralizedPassed ? 0 : 1;
}
