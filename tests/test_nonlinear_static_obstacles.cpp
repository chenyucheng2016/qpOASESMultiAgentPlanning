#include <qpOASES/NonlinearTurboADMM.hpp>

#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

USING_NAMESPACE_QPOASES

namespace
{

NonlinearAgentProblem makeAgent(const UnicycleModel& model, int_t horizon)
{
    NonlinearAgentProblem p;
    p.model = &model;
    p.horizon = horizon;
    const real_t initial[] = {-3.0, 0.0, 0.0, 1.0};
    p.initialState.assign(initial, initial + 4);
    p.stateReference.assign((horizon + 1) * 4, 0.0);
    p.controlReference.assign(horizon * 2, 0.0);
    p.initialControls.assign(horizon * 2, 0.0);
    for (int_t k = 0; k <= horizon; ++k)
    {
        const real_t alpha = static_cast<real_t>(k) / horizon;
        p.stateReference[k * 4] = -3.0 + 6.0 * alpha;
        p.stateReference[k * 4 + 3] = 1.0;
    }
    const real_t q[] = {2.0, 4.0, 0.3, 0.1};
    const real_t qTerminal[] = {20.0, 20.0, 1.0, 0.2};
    const real_t r[] = {0.1, 0.2};
    const real_t lowerState[] = {-10.0, -4.0, -7.0, 0.0};
    const real_t upperState[] = {10.0, 4.0, 7.0, 3.0};
    const real_t lowerControl[] = {-2.0, -1.2};
    const real_t upperControl[] = {2.0, 1.2};
    p.stateWeights.assign(q, q + 4);
    p.terminalWeights.assign(qTerminal, qTerminal + 4);
    p.controlWeights.assign(r, r + 2);
    p.stateLowerBounds.assign(lowerState, lowerState + 4);
    p.stateUpperBounds.assign(upperState, upperState + 4);
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
    p.obstacleSafetyDistance = 0.4;
    const real_t initial[] = {
        1.2, -3.0, 1.5, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0
    };
    p.initialState.assign(initial, initial + 9);
    p.stateReference.assign((horizon + 1) * 9, 0.0);
    p.controlReference.assign(horizon * 4, 0.0);
    p.initialControls.assign(horizon * 4, 0.0);
    for (int_t k = 0; k <= horizon; ++k)
    {
        const real_t alpha = static_cast<real_t>(k) / horizon;
        p.stateReference[k * 9] = 1.2 * std::fabs(2.0 * alpha - 1.0);
        p.stateReference[k * 9 + 1] = -3.0 + 6.0 * alpha;
        p.stateReference[k * 9 + 2] = 1.5;
        p.stateReference[k * 9 + 4] = 1.0;
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
        -6.0, -6.0, 0.5, -3.0, -3.0, -3.0, -0.6, -0.6, -7.0
    };
    const real_t upperState[] = {
        6.0, 6.0, 3.0, 3.0, 3.0, 3.0, 0.6, 0.6, 7.0
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

ConvexPolygonObstacle makeObstacle()
{
    ConvexPolygonObstacle obstacle;
    const real_t vertices[] = {
        -0.8, 0.0,
         0.0, 0.5,
         0.8, 0.0,
         0.0, -0.5
    };
    obstacle.vertices.assign(vertices, vertices + 8);
    return obstacle;
}

ConvexPolygonObstacle translatedObstacle(real_t x, real_t y)
{
    ConvexPolygonObstacle obstacle = makeObstacle();
    for (std::size_t vertex = 0;
         vertex < obstacle.vertices.size() / 2;
         ++vertex)
    {
        obstacle.vertices[2 * vertex] += x;
        obstacle.vertices[2 * vertex + 1] += y;
    }
    return obstacle;
}

std::vector<ConvexPolygonObstacle> makeNarrowPassage()
{
    std::vector<ConvexPolygonObstacle> obstacles(2);
    const real_t lowerVertices[] = {
        -1.5, -2.0,
         1.5, -2.0,
         1.5, -0.35,
        -1.5, -0.35
    };
    const real_t upperVertices[] = {
        -1.5, 0.35,
         1.5, 0.35,
         1.5, 2.0,
        -1.5, 2.0
    };
    obstacles[0].vertices.assign(
        lowerVertices,
        lowerVertices + 8
    );
    obstacles[1].vertices.assign(
        upperVertices,
        upperVertices + 8
    );
    return obstacles;
}

real_t finalPositionError(
    const NonlinearAgentProblem& problem,
    const NonlinearTrajectory& trajectory
)
{
    const int_t nx = problem.model->stateDimension();
    const real_t dx = trajectory.states[problem.horizon * nx]
        - problem.stateReference[problem.horizon * nx];
    const real_t dy = trajectory.states[problem.horizon * nx + 1]
        - problem.stateReference[problem.horizon * nx + 1];
    return std::sqrt(dx * dx + dy * dy);
}

bool runMethod(
    NonlinearCoordinationMethod method,
    const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<ConvexPolygonObstacle>& obstacles
)
{
    NonlinearTurboOptions options;
    options.coordinationMethod = method;
    options.obstacleSafetyDistance = 0.3;
    options.maxScpIterations = 10;
    options.maxAdmmIterations = 20;
    options.controlTrustRegion = 0.8;
    const NonlinearTurboResult result = NonlinearTurboADMM().solve(
        agents,
        obstacles,
        options
    );
    if (result.trajectories.size() != agents.size()) return false;
    const real_t terminalError = finalPositionError(
        agents[0],
        result.trajectories[0]
    );
    std::printf(
        "%s obstacle: %s, distance %.3f, margin %.3f, defect %.3e, terminal %.3f\n",
        method == NCM_DISTRIBUTED_ADMM ? "distributed" : "centralized",
        result.status.c_str(),
        result.statistics.minimumObstacleDistance,
        result.statistics.minimumObstacleClearance,
        result.statistics.maximumDynamicsDefect,
        terminalError
    );
    return result.success
        && result.statistics.minimumObstacleDistance >= 0.29
        && result.statistics.minimumObstacleClearance >= -0.01
        && result.statistics.maximumDynamicsDefect <= 1.0e-9
        && terminalError <= 0.5;
}

bool runHeterogeneous(
    const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<ConvexPolygonObstacle>& obstacles
)
{
    NonlinearTurboOptions options;
    options.coordinationMethod = NCM_DISTRIBUTED_ADMM;
    options.safetyDistance = 0.8;
    options.obstacleSafetyDistance = 0.3;
    options.maxScpIterations = 8;
    options.maxAdmmIterations = 40;
    options.rho = 35.0;
    options.controlTrustRegion = 0.8;
    const NonlinearTurboResult result = NonlinearTurboADMM().solve(
        agents,
        obstacles,
        options
    );
    std::printf(
        "heterogeneous obstacle: %s, pair %.3f, distance %.3f, margin %.3f, defect %.3e\n",
        result.status.c_str(),
        result.statistics.minimumDistance,
        result.statistics.minimumObstacleDistance,
        result.statistics.minimumObstacleClearance,
        result.statistics.maximumDynamicsDefect
    );
    return result.success
        && result.trajectories.size() == agents.size()
        && result.statistics.minimumDistance >= 0.79
        && result.statistics.minimumObstacleDistance >= 0.29
        && result.statistics.minimumObstacleClearance >= -0.01
        && result.statistics.maximumDynamicsDefect <= 1.0e-9
        && result.trajectories[1].states.size()
            == static_cast<std::size_t>(
                (agents[1].horizon + 1)
                    * agents[1].model->stateDimension()
            );
}

bool invalidPolygonIsRejected(
    const std::vector<NonlinearAgentProblem>& agents
)
{
    ConvexPolygonObstacle concave;
    const real_t vertices[] = {
        -1.0, -1.0,
         1.0, -1.0,
         0.0, 0.0,
         1.0, 1.0,
        -1.0, 1.0
    };
    concave.vertices.assign(vertices, vertices + 10);
    std::vector<ConvexPolygonObstacle> obstacles(1, concave);
    const NonlinearTurboResult result = NonlinearTurboADMM().solve(
        agents,
        obstacles
    );
    return !result.success
        && result.status.find("convex polygon") != std::string::npos;
}

}

int main()
{
    const int_t horizon = 24;
    UnicycleModel model(0.25);
    QuadcopterModel quadcopter(0.25, 1.0, 9.81);
    std::vector<NonlinearAgentProblem> agents;
    agents.push_back(makeAgent(model, horizon));
    std::vector<ConvexPolygonObstacle> obstacles;
    obstacles.push_back(makeObstacle());

    if (!invalidPolygonIsRejected(agents))
    {
        std::fprintf(stderr, "concave obstacle was not rejected\n");
        return 1;
    }
    if (!runMethod(NCM_DISTRIBUTED_ADMM, agents, obstacles)) return 1;
    if (!runMethod(NCM_CENTRALIZED_SCP, agents, obstacles)) return 1;
    std::vector<NonlinearAgentProblem> heterogeneousAgents(agents);
    heterogeneousAgents.push_back(
        makeQuadcopter(quadcopter, horizon)
    );
    if (!runHeterogeneous(heterogeneousAgents, obstacles)) return 1;
    std::vector<ConvexPolygonObstacle> multipleObstacles;
    multipleObstacles.push_back(translatedObstacle(-1.0, 0.0));
    multipleObstacles.push_back(translatedObstacle(1.0, 0.0));
    if (!runMethod(
            NCM_DISTRIBUTED_ADMM, agents, multipleObstacles)) return 1;
    if (!runMethod(
            NCM_CENTRALIZED_SCP, agents, multipleObstacles)) return 1;
    const std::vector<ConvexPolygonObstacle> narrowPassage =
        makeNarrowPassage();
    if (!runMethod(
            NCM_DISTRIBUTED_ADMM, agents, narrowPassage)) return 1;
    if (!runMethod(
            NCM_CENTRALIZED_SCP, agents, narrowPassage)) return 1;
    return 0;
}
