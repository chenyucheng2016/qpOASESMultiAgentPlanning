#include <qpOASES/NonlinearTrajectoryValidator.hpp>
#include <qpOASES/Constants.hpp>

#include <algorithm>
#include <cmath>
#include <sstream>

BEGIN_NAMESPACE_QPOASES

namespace
{

real_t collisionRadius(
    const NonlinearAgentProblem& agent,
    const NonlinearTurboOptions& options)
{
    return agent.collisionRadius >= 0.0
        ? agent.collisionRadius : 0.5 * options.safetyDistance;
}

real_t obstacleClearance(
    const NonlinearAgentProblem& agent,
    const NonlinearTurboOptions& options)
{
    return agent.obstacleSafetyDistance >= 0.0
        ? agent.obstacleSafetyDistance : options.obstacleSafetyDistance;
}

int_t worldDimension(const std::vector<NonlinearAgentProblem>& agents)
{
    int_t dimension = 0;
    for (std::size_t a = 0; a < agents.size(); ++a)
        dimension = std::max(dimension, agents[a].model->positionDimension());
    return dimension;
}

void positionAt(
    const NonlinearAgentProblem& agent,
    const NonlinearTrajectory& trajectory,
    int_t stage,
    int_t dimension,
    std::vector<real_t>& position)
{
    position.assign(dimension, 0.0);
    agent.model->position(
        &trajectory.states[stage * agent.model->stateDimension()],
        &position[0]
    );
}

real_t polygonAreaTwice(const ConvexPolygonObstacle& obstacle)
{
    real_t area = 0.0;
    const std::size_t count = obstacle.vertices.size() / 2;
    for (std::size_t i = 0; i < count; ++i)
    {
        const std::size_t next = (i + 1) % count;
        area += obstacle.vertices[2 * i] * obstacle.vertices[2 * next + 1]
            - obstacle.vertices[2 * i + 1] * obstacle.vertices[2 * next];
    }
    return area;
}

real_t signedPolygonDistance(
    real_t px,
    real_t py,
    const ConvexPolygonObstacle& obstacle)
{
    const std::size_t count = obstacle.vertices.size() / 2;
    const real_t orientation = polygonAreaTwice(obstacle) >= 0.0 ? 1.0 : -1.0;
    real_t minimumSquared = INFTY;
    bool inside = true;
    for (std::size_t i = 0; i < count; ++i)
    {
        const std::size_t next = (i + 1) % count;
        const real_t ax = obstacle.vertices[2 * i];
        const real_t ay = obstacle.vertices[2 * i + 1];
        const real_t ex = obstacle.vertices[2 * next] - ax;
        const real_t ey = obstacle.vertices[2 * next + 1] - ay;
        if (orientation * (ex * (py - ay) - ey * (px - ax)) < -1.0e-12)
            inside = false;
        const real_t lengthSquared = ex * ex + ey * ey;
        const real_t fraction = std::max(
            0.0,
            std::min(1.0, ((px - ax) * ex + (py - ay) * ey) / lengthSquared)
        );
        const real_t dx = px - (ax + fraction * ex);
        const real_t dy = py - (ay + fraction * ey);
        minimumSquared = std::min(minimumSquared, dx * dx + dy * dy);
    }
    const real_t distance = std::sqrt(std::max(0.0, minimumSquared));
    return inside ? -distance : distance;
}

bool validInput(
    const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<ConvexPolygonObstacle>& obstacles,
    const std::vector<NonlinearTrajectory>& trajectories,
    const NonlinearValidationOptions& options,
    std::string& error)
{
    if (agents.empty() || agents.size() != trajectories.size())
    {
        error = "validator requires one trajectory per agent";
        return false;
    }
    if (options.interpolationSubsteps <= 0
        || !std::isfinite(options.dynamicsTolerance)
        || options.dynamicsTolerance < 0.0
        || !std::isfinite(options.terminalPositionTolerance)
        || options.terminalPositionTolerance < 0.0)
    {
        error = "invalid validation options";
        return false;
    }
    const int_t horizon = agents[0].horizon;
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        if (agents[a].model == 0 || agents[a].horizon != horizon)
        {
            error = "validator requires valid models and a common horizon";
            return false;
        }
        const int_t nx = agents[a].model->stateDimension();
        const int_t nu = agents[a].model->controlDimension();
        if (trajectories[a].states.size()
                != static_cast<std::size_t>((horizon + 1) * nx)
            || trajectories[a].controls.size()
                != static_cast<std::size_t>(horizon * nu)
            || agents[a].stateReference.size()
                != static_cast<std::size_t>((horizon + 1) * nx))
        {
            error = "trajectory dimensions are inconsistent";
            return false;
        }
    }
    for (std::size_t obstacle = 0; obstacle < obstacles.size(); ++obstacle)
    {
        if (obstacles[obstacle].vertices.size() < 6
            || obstacles[obstacle].vertices.size() % 2 != 0
            || std::fabs(polygonAreaTwice(obstacles[obstacle])) <= 1.0e-12)
        {
            error = "validator received an invalid polygon";
            return false;
        }
        const std::size_t count = obstacles[obstacle].vertices.size() / 2;
        for (std::size_t vertex = 0; vertex < count; ++vertex)
        {
            const std::size_t next = (vertex + 1) % count;
            const real_t x = obstacles[obstacle].vertices[2 * vertex];
            const real_t y = obstacles[obstacle].vertices[2 * vertex + 1];
            const real_t dx = obstacles[obstacle].vertices[2 * next] - x;
            const real_t dy = obstacles[obstacle].vertices[2 * next + 1] - y;
            if (!std::isfinite(x) || !std::isfinite(y)
                || dx * dx + dy * dy <= 1.0e-18)
            {
                error = "validator received an invalid polygon";
                return false;
            }
        }
    }
    return true;
}

}

NonlinearValidationOptions::NonlinearValidationOptions()
    : interpolationSubsteps(10), dynamicsTolerance(1.0e-8),
      terminalPositionTolerance(2.0) {}

NonlinearValidationResult::NonlinearValidationResult()
    : success(false), minimumPairwiseClearance(INFTY),
      minimumObstacleClearance(INFTY), maximumDynamicsDefect(0.0),
      maximumTerminalPositionError(0.0) {}

NonlinearValidationResult validateNonlinearTrajectories(
    const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<ConvexPolygonObstacle>& obstacles,
    const std::vector<NonlinearTrajectory>& trajectories,
    const NonlinearTurboOptions& solverOptions,
    const NonlinearValidationOptions& validationOptions)
{
    NonlinearValidationResult result;
    if (!validInput(agents, obstacles, trajectories, validationOptions, result.status))
        return result;
    if (!std::isfinite(solverOptions.collisionTolerance)
        || solverOptions.collisionTolerance < 0.0)
    {
        result.status = "invalid solver tolerance for validation";
        return result;
    }

    const int_t horizon = agents[0].horizon;
    const int_t dimension = worldDimension(agents);
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        const int_t nx = agents[a].model->stateDimension();
        const int_t nu = agents[a].model->controlDimension();
        std::vector<real_t> predicted(nx, 0.0);
        for (int_t k = 0; k < horizon; ++k)
        {
            agents[a].model->dynamics(
                &trajectories[a].states[k * nx],
                &trajectories[a].controls[k * nu],
                &predicted[0]
            );
            for (int_t i = 0; i < nx; ++i)
                result.maximumDynamicsDefect = std::max(
                    result.maximumDynamicsDefect,
                    std::fabs(predicted[i] - trajectories[a].states[(k + 1) * nx + i])
                );
        }

        const int_t nativeDimension = agents[a].model->positionDimension();
        std::vector<real_t> terminal(nativeDimension, 0.0);
        std::vector<real_t> reference(nativeDimension, 0.0);
        agents[a].model->position(
            &trajectories[a].states[horizon * nx],
            &terminal[0]
        );
        agents[a].model->position(
            &agents[a].stateReference[horizon * nx],
            &reference[0]
        );
        real_t terminalSquared = 0.0;
        for (int_t i = 0; i < nativeDimension; ++i)
            terminalSquared += (terminal[i] - reference[i])
                * (terminal[i] - reference[i]);
        result.maximumTerminalPositionError = std::max(
            result.maximumTerminalPositionError,
            std::sqrt(terminalSquared)
        );
    }

    std::vector<std::vector<real_t> > start(agents.size());
    std::vector<std::vector<real_t> > finish(agents.size());
    for (int_t k = 0; k < horizon; ++k)
    {
        for (std::size_t a = 0; a < agents.size(); ++a)
        {
            positionAt(agents[a], trajectories[a], k, dimension, start[a]);
            positionAt(agents[a], trajectories[a], k + 1, dimension, finish[a]);
        }
        for (int_t substep = 0; substep <= validationOptions.interpolationSubsteps; ++substep)
        {
            const real_t alpha = static_cast<real_t>(substep)
                / validationOptions.interpolationSubsteps;
            for (std::size_t first = 0; first < agents.size(); ++first)
            for (std::size_t second = first + 1; second < agents.size(); ++second)
            {
                real_t distanceSquared = 0.0;
                for (int_t i = 0; i < dimension; ++i)
                {
                    const real_t firstPosition = start[first][i]
                        + alpha * (finish[first][i] - start[first][i]);
                    const real_t secondPosition = start[second][i]
                        + alpha * (finish[second][i] - start[second][i]);
                    const real_t difference = firstPosition - secondPosition;
                    distanceSquared += difference * difference;
                }
                result.minimumPairwiseClearance = std::min(
                    result.minimumPairwiseClearance,
                    std::sqrt(distanceSquared)
                        - collisionRadius(agents[first], solverOptions)
                        - collisionRadius(agents[second], solverOptions)
                );
            }
            for (std::size_t a = 0; a < agents.size(); ++a)
            for (std::size_t obstacle = 0; obstacle < obstacles.size(); ++obstacle)
            {
                const real_t px = start[a][0] + alpha * (finish[a][0] - start[a][0]);
                const real_t py = start[a][1] + alpha * (finish[a][1] - start[a][1]);
                result.minimumObstacleClearance = std::min(
                    result.minimumObstacleClearance,
                    signedPolygonDistance(px, py, obstacles[obstacle])
                        - obstacleClearance(agents[a], solverOptions)
                );
            }
        }
    }

    if (result.minimumPairwiseClearance < -solverOptions.collisionTolerance)
        result.status = "interpolated pairwise collision";
    else if (result.minimumObstacleClearance < -solverOptions.collisionTolerance)
        result.status = "interpolated obstacle collision";
    else if (result.maximumDynamicsDefect > validationOptions.dynamicsTolerance)
        result.status = "nonlinear dynamics defect exceeds tolerance";
    else if (result.maximumTerminalPositionError
        > validationOptions.terminalPositionTolerance)
        result.status = "terminal position error exceeds tolerance";
    else
    {
        result.success = true;
        result.status = "validated";
    }
    return result;
}

END_NAMESPACE_QPOASES
