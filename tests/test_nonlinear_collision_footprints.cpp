#include <qpOASES/NonlinearTrajectoryValidator.hpp>

#include <cmath>
#include <cstdio>
#include <vector>

USING_NAMESPACE_QPOASES

namespace
{

NonlinearAgentProblem makeAgent(
    const FrontSteeringModel& model,
    real_t x)
{
    NonlinearAgentProblem agent;
    agent.model = &model;
    agent.horizon = 1;
    agent.initialState.assign(4, 0.0);
    agent.initialState[0] = x;
    agent.stateReference.assign(8, 0.0);
    agent.stateReference[0] = x;
    agent.stateReference[4] = x;
    agent.controlReference.assign(2, 0.0);
    agent.stateWeights.assign(4, 0.0);
    agent.terminalWeights.assign(4, 0.0);
    agent.controlWeights.assign(2, 0.0);
    agent.collisionCircles.push_back(
        CollisionCircle(1.25, 0.0, 1.25));
    agent.collisionCircles.push_back(
        CollisionCircle(-0.25, 0.0, 1.25));
    agent.obstacleSafetyDistance = 0.0;
    return agent;
}

NonlinearTrajectory stationaryTrajectory(real_t x)
{
    NonlinearTrajectory trajectory;
    trajectory.states.assign(8, 0.0);
    trajectory.states[0] = x;
    trajectory.states[4] = x;
    trajectory.controls.assign(2, 0.0);
    return trajectory;
}

}

int main()
{
    FrontSteeringModel model(0.1, 1.0);
    NonlinearTurboOptions solverOptions;
    solverOptions.obstacleSafetyDistance = 0.0;
    solverOptions.collisionTolerance = 1.0e-8;
    NonlinearValidationOptions validationOptions;
    validationOptions.interpolationSubsteps = 4;
    validationOptions.terminalPositionTolerance = 1.0e-8;

    std::vector<NonlinearAgentProblem> pairAgents;
    pairAgents.push_back(makeAgent(model, 0.0));
    pairAgents.push_back(makeAgent(model, 3.9));
    std::vector<NonlinearTrajectory> pairTrajectories;
    pairTrajectories.push_back(stationaryTrajectory(0.0));
    pairTrajectories.push_back(stationaryTrajectory(3.9));
    const NonlinearValidationResult pairResult =
        validateNonlinearTrajectories(
            pairAgents,
            std::vector<ConvexPolygonObstacle>(),
            pairTrajectories,
            solverOptions,
            validationOptions
        );

    std::vector<NonlinearAgentProblem> obstacleAgents(
        1, makeAgent(model, 0.0));
    std::vector<NonlinearTrajectory> obstacleTrajectories(
        1, stationaryTrajectory(0.0));
    ConvexPolygonObstacle obstacle;
    obstacle.vertices.push_back(2.45);
    obstacle.vertices.push_back(-0.1);
    obstacle.vertices.push_back(2.55);
    obstacle.vertices.push_back(-0.1);
    obstacle.vertices.push_back(2.55);
    obstacle.vertices.push_back(0.1);
    obstacle.vertices.push_back(2.45);
    obstacle.vertices.push_back(0.1);
    const NonlinearValidationResult obstacleResult =
        validateNonlinearTrajectories(
            obstacleAgents,
            std::vector<ConvexPolygonObstacle>(1, obstacle),
            obstacleTrajectories,
            solverOptions,
            validationOptions
        );

    std::printf(
        "two-disc pair clearance %.6f, obstacle clearance %.6f\n",
        pairResult.minimumPairwiseClearance,
        obstacleResult.minimumObstacleClearance
    );
    return !pairResult.success
        && pairResult.status == "interpolated pairwise collision"
        && std::fabs(pairResult.minimumPairwiseClearance + 0.1) <= 1.0e-8
        && !obstacleResult.success
        && obstacleResult.status == "interpolated obstacle collision"
        && std::fabs(obstacleResult.minimumObstacleClearance + 0.05) <= 1.0e-8
        ? 0 : 1;
}
