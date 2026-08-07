#include <qpOASES/NonlinearTurboADMM.hpp>
#include <qpOASES/Constants.hpp>
#include <qpOASES/Options.hpp>
#include <qpOASES/SQProblem.hpp>
#include <qpOASES/StageVaryingRiccati.hpp>

#ifdef QPOASES_WITH_OSQP
#include <osqp/osqp.h>
#endif
#include <algorithm>
#include <chrono>
#include <cmath>
#include <sstream>

#if defined(_OPENMP)
#include <omp.h>
#endif

BEGIN_NAMESPACE_QPOASES

namespace
{

struct AgentQp
{
    int_t N, nx, nu, np, nV, nC, activeObstacleCount, corridorRows;
    int_t trajectoryVariableCount, slackVariableCount;
    std::vector<real_t> Hbase, gbase, H, g, A, lb, ub, lbA, ubA;
    std::vector<real_t> dynamicsA, dynamicsB, dynamicsC;
    std::vector<real_t> positionC, positionD, solution;
};

struct PairData
{
    int_t first, second, firstCircle, secondCircle;
    int_t samplesPerInterval;
    real_t safetyDistance;
    std::vector<real_t> normal, firstPosition, secondPosition;
    std::vector<real_t> firstAuxiliary, secondAuxiliary;
    std::vector<real_t> firstDual, secondDual;
    std::vector<real_t> previousFirstAuxiliary, previousSecondAuxiliary;
    std::vector<real_t> firstC, firstD, secondC, secondD;
};

struct ActiveObstacle
{
    std::size_t obstacle;
    int_t circle;
    real_t requiredDistance;
};

struct CollisionLinearization
{
    std::vector<real_t> positions;
    std::vector<real_t> C;
    std::vector<real_t> D;
};

struct SolverPool
{
    std::vector<SQProblem*> solvers;
    std::vector<int_t> variableCounts;
    std::vector<int_t> constraintCounts;
    void reset()
    {
        for (std::size_t i = 0; i < solvers.size(); ++i) delete solvers[i];
        solvers.clear();
        variableCounts.clear();
        constraintCounts.clear();
    }
    ~SolverPool() { reset(); }
};

struct CentralContext
{
    SQProblem* solver;
    std::vector<real_t> H, g, A, lb, ub, lbA, ubA;
    std::vector<real_t> osqpWarmStart;
    CentralContext() : solver(0) {}
    void reset()
    {
        delete solver;
        solver = 0;
        H.clear(); g.clear(); A.clear(); lb.clear(); ub.clear();
        lbA.clear(); ubA.clear(); osqpWarmStart.clear();
    }
    ~CentralContext() { reset(); }
};

struct PolygonProjection
{
    real_t closest[2];
    real_t normal[2];
    real_t signedDistance;
};

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

bool validateObstacles(
    const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<ConvexPolygonObstacle>& obstacles,
    std::string& error
)
{
    if (obstacles.empty()) return true;
    for (std::size_t a = 0; a < agents.size(); ++a)
        if (agents[a].model->positionDimension() < 2)
        {
            error = "polygon obstacles require two-dimensional position outputs";
            return false;
        }
    for (std::size_t obstacleIndex = 0;
         obstacleIndex < obstacles.size();
         ++obstacleIndex)
    {
        const ConvexPolygonObstacle& obstacle = obstacles[obstacleIndex];
        if (obstacle.vertices.size() < 6 || obstacle.vertices.size() % 2 != 0)
        {
            error = "each obstacle requires at least three x-y vertices";
            return false;
        }
        const real_t area = polygonAreaTwice(obstacle);
        if (!std::isfinite(area) || std::fabs(area) <= 1.0e-12)
        {
            error = "obstacle vertices must define a finite nonzero-area polygon";
            return false;
        }
        const real_t orientation = area > 0.0 ? 1.0 : -1.0;
        const std::size_t count = obstacle.vertices.size() / 2;
        for (std::size_t i = 0; i < count; ++i)
        {
            const std::size_t next = (i + 1) % count;
            const std::size_t after = (i + 2) % count;
            const real_t ax = obstacle.vertices[2 * i];
            const real_t ay = obstacle.vertices[2 * i + 1];
            const real_t bx = obstacle.vertices[2 * next];
            const real_t by = obstacle.vertices[2 * next + 1];
            const real_t cx = obstacle.vertices[2 * after];
            const real_t cy = obstacle.vertices[2 * after + 1];
            const real_t edgeSquared =
                (bx - ax) * (bx - ax) + (by - ay) * (by - ay);
            const real_t turn =
                (bx - ax) * (cy - by) - (by - ay) * (cx - bx);
            if (!std::isfinite(ax) || !std::isfinite(ay)
                || edgeSquared <= 1.0e-18
                || orientation * turn < -1.0e-12)
            {
                error = "obstacle vertices must form an ordered convex polygon";
                return false;
            }
        }
    }
    return true;
}

PolygonProjection projectToPolygon(
    real_t px,
    real_t py,
    const ConvexPolygonObstacle& obstacle
)
{
    PolygonProjection projection;
    real_t bestSquared = INFTY;
    std::size_t bestEdge = 0;
    bool inside = true;
    const real_t area = polygonAreaTwice(obstacle);
    const real_t orientation = area > 0.0 ? 1.0 : -1.0;
    const std::size_t count = obstacle.vertices.size() / 2;
    for (std::size_t i = 0; i < count; ++i)
    {
        const std::size_t next = (i + 1) % count;
        const real_t ax = obstacle.vertices[2 * i];
        const real_t ay = obstacle.vertices[2 * i + 1];
        const real_t ex = obstacle.vertices[2 * next] - ax;
        const real_t ey = obstacle.vertices[2 * next + 1] - ay;
        if (orientation * (ex * (py - ay) - ey * (px - ax)) < -1.0e-12)
            inside = false;
        const real_t edgeSquared = ex * ex + ey * ey;
        const real_t t = std::max(
            0.0,
            std::min(1.0, ((px - ax) * ex + (py - ay) * ey) / edgeSquared)
        );
        const real_t qx = ax + t * ex;
        const real_t qy = ay + t * ey;
        const real_t distanceSquared =
            (px - qx) * (px - qx) + (py - qy) * (py - qy);
        if (distanceSquared < bestSquared)
        {
            bestSquared = distanceSquared;
            bestEdge = i;
            projection.closest[0] = qx;
            projection.closest[1] = qy;
        }
    }
    const real_t distance = std::sqrt(std::max(0.0, bestSquared));
    if (!inside && distance > 1.0e-12)
    {
        projection.normal[0] = (px - projection.closest[0]) / distance;
        projection.normal[1] = (py - projection.closest[1]) / distance;
    }
    else
    {
        const std::size_t next = (bestEdge + 1) % count;
        const real_t ex = obstacle.vertices[2 * next]
            - obstacle.vertices[2 * bestEdge];
        const real_t ey = obstacle.vertices[2 * next + 1]
            - obstacle.vertices[2 * bestEdge + 1];
        const real_t inverseLength = 1.0 / std::sqrt(ex * ex + ey * ey);
        projection.normal[0] = orientation * ey * inverseLength;
        projection.normal[1] = -orientation * ex * inverseLength;
    }
    projection.signedDistance = inside ? -distance : distance;
    return projection;
}

struct ObstacleBypass
{
    bool active;
    real_t direction[2];
    real_t normal[2];
    real_t minimumTravel;
    real_t maximumTravel;
    ObstacleBypass() : active(false), minimumTravel(0.0),
        maximumTravel(0.0)
    {
        direction[0] = direction[1] = 0.0;
        normal[0] = normal[1] = 0.0;
    }
};

void worldCollisionPosition(
    const NonlinearAgentProblem& agent,
    int_t circle,
    const real_t* state,
    int_t worldDimension,
    real_t* position
);

real_t polygonSupport(
    const ConvexPolygonObstacle& obstacle,
    const real_t* normal
)
{
    real_t support = -INFTY;
    for (std::size_t vertex = 0;
         vertex < obstacle.vertices.size() / 2;
         ++vertex)
        support = std::max(
            support,
            normal[0] * obstacle.vertices[2 * vertex]
                + normal[1] * obstacle.vertices[2 * vertex + 1]
        );
    return support;
}

ObstacleBypass buildObstacleBypass(
    const NonlinearAgentProblem& agent,
    int_t circle,
    const ConvexPolygonObstacle& obstacle,
    real_t requiredDistance
)
{
    ObstacleBypass bypass;
    const int_t nx = agent.model->stateDimension();
    const int_t np = agent.model->positionDimension();
    std::vector<real_t> position(np, 0.0);
    real_t start[2], goal[2];
    worldCollisionPosition(
        agent, circle, &agent.stateReference[0], np, &position[0]);
    start[0] = position[0]; start[1] = position[1];
    worldCollisionPosition(
        agent, circle,
        &agent.stateReference[agent.horizon * nx],
        np,
        &position[0]
    );
    goal[0] = position[0]; goal[1] = position[1];
    const real_t dx = goal[0] - start[0];
    const real_t dy = goal[1] - start[1];
    const real_t length = std::sqrt(dx * dx + dy * dy);
    if (length <= 1.0e-9) return bypass;
    bypass.direction[0] = dx / length;
    bypass.direction[1] = dy / length;
    const real_t candidates[2][2] = {
        {-bypass.direction[1], bypass.direction[0]},
        {bypass.direction[1], -bypass.direction[0]}
    };
    real_t costs[2] = {0.0, 0.0};
    bool intersects = false;
    for (int_t k = 0; k <= agent.horizon; ++k)
    {
        worldCollisionPosition(
            agent, circle,
            &agent.stateReference[k * nx],
            np,
            &position[0]
        );
        if (projectToPolygon(
                position[0], position[1], obstacle
            ).signedDistance < requiredDistance)
            intersects = true;
        for (int_t candidate = 0; candidate < 2; ++candidate)
        {
            const real_t violation = std::max(
                0.0,
                polygonSupport(obstacle, candidates[candidate])
                    + requiredDistance
                    - candidates[candidate][0] * position[0]
                    - candidates[candidate][1] * position[1]
            );
            costs[candidate] += violation * violation;
        }
    }
    if (!intersects) return bypass;
    const int_t selected = costs[1] < costs[0] ? 1 : 0;
    bypass.active = true;
    bypass.normal[0] = candidates[selected][0];
    bypass.normal[1] = candidates[selected][1];
    bypass.minimumTravel = INFTY;
    bypass.maximumTravel = -INFTY;
    for (std::size_t vertex = 0;
         vertex < obstacle.vertices.size() / 2;
         ++vertex)
    {
        const real_t travel =
            bypass.direction[0] * obstacle.vertices[2 * vertex]
            + bypass.direction[1] * obstacle.vertices[2 * vertex + 1];
        bypass.minimumTravel = std::min(bypass.minimumTravel, travel);
        bypass.maximumTravel = std::max(bypass.maximumTravel, travel);
    }
    bypass.minimumTravel -= requiredDistance;
    bypass.maximumTravel += requiredDistance;
    return bypass;
}

bool bypassHalfspaceAt(
    const ObstacleBypass& bypass,
    const ConvexPolygonObstacle& obstacle,
    real_t requiredDistance,
    real_t travel,
    real_t* normal,
    real_t& support
)
{
    if (!bypass.active
        || travel < bypass.minimumTravel
        || travel > bypass.maximumTravel)
        return false;

    const real_t oppositeNormal[2] = {
        -bypass.normal[0], -bypass.normal[1]
    };
    const real_t maximumTransverse = polygonSupport(
        obstacle,
        bypass.normal
    );
    const real_t minimumTransverse = -polygonSupport(
        obstacle,
        oppositeNormal
    );
    real_t outside = maximumTransverse + requiredDistance + 1.0e-9;
    const real_t searchRange = maximumTransverse - minimumTransverse
        + 2.0 * requiredDistance + 1.0e-6;
    real_t inside = outside;
    bool foundInside = false;
    for (int_t sample = 1; sample <= 64; ++sample)
    {
        const real_t transverse = outside
            - searchRange * static_cast<real_t>(sample) / 64.0;
        const real_t x = travel * bypass.direction[0]
            + transverse * bypass.normal[0];
        const real_t y = travel * bypass.direction[1]
            + transverse * bypass.normal[1];
        if (projectToPolygon(x, y, obstacle).signedDistance
            < requiredDistance)
        {
            inside = transverse;
            foundInside = true;
            break;
        }
    }
    if (!foundInside) return false;

    for (int_t iteration = 0; iteration < 40; ++iteration)
    {
        const real_t transverse = 0.5 * (inside + outside);
        const real_t x = travel * bypass.direction[0]
            + transverse * bypass.normal[0];
        const real_t y = travel * bypass.direction[1]
            + transverse * bypass.normal[1];
        if (projectToPolygon(x, y, obstacle).signedDistance
            < requiredDistance)
            inside = transverse;
        else
            outside = transverse;
    }
    const real_t boundaryX = travel * bypass.direction[0]
        + outside * bypass.normal[0];
    const real_t boundaryY = travel * bypass.direction[1]
        + outside * bypass.normal[1];
    const PolygonProjection projection = projectToPolygon(
        boundaryX,
        boundaryY,
        obstacle
    );
    normal[0] = projection.normal[0];
    normal[1] = projection.normal[1];
    support = normal[0] * projection.closest[0]
        + normal[1] * projection.closest[1];
    return true;
}

int_t stateIndex(const AgentQp& qp, int_t k)
{
    return k == qp.N ? qp.N * (qp.nx + qp.nu) : k * (qp.nx + qp.nu);
}

int_t controlIndex(const AgentQp& qp, int_t k)
{
    return k * (qp.nx + qp.nu) + qp.nx;
}

real_t optionalBound(const std::vector<real_t>& values, int_t i, real_t fallback)
{
    return values.empty() ? fallback : values[i];
}

real_t obstacleDistanceForAgent(
    const NonlinearAgentProblem& agent,
    const NonlinearTurboOptions& options
)
{
    return agent.obstacleSafetyDistance >= 0.0
        ? agent.obstacleSafetyDistance : options.obstacleSafetyDistance;
}

real_t terminalToleranceForAgent(
    const NonlinearAgentProblem& agent,
    const NonlinearTurboOptions& options
)
{
    return agent.terminalPositionTolerance >= 0.0
        ? agent.terminalPositionTolerance : options.terminalPositionTolerance;
}

real_t collisionRadiusForAgent(
    const NonlinearAgentProblem& agent,
    const NonlinearTurboOptions& options
)
{
    return agent.collisionRadius >= 0.0
        ? agent.collisionRadius : 0.5 * options.safetyDistance;
}

real_t pairSafetyDistance(
    const NonlinearAgentProblem& first,
    const NonlinearAgentProblem& second,
    const NonlinearTurboOptions& options
)
{
    return collisionRadiusForAgent(first, options)
        + collisionRadiusForAgent(second, options);
}
void configureSolver(SQProblem& solver, bool reliable = false)
{
    Options options;
    if (reliable)
    {
        options.setToReliable();
        options.enableRegularisation = BT_TRUE;
        options.numRegularisationSteps = 2;
        options.enableEqualities = BT_TRUE;
    }
    else options.setToMPC();
    options.printLevel = PL_NONE;
    solver.setOptions(options);
}

bool validateProblems(const std::vector<NonlinearAgentProblem>& agents, std::string& error)
{
    if (agents.empty()) { error = "at least one agent is required"; return false; }
    const int_t N = agents[0].horizon;
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        const NonlinearAgentProblem& p = agents[a];
        if (!p.model) { error = "each agent requires a nonlinear model"; return false; }
        if (!std::isfinite(p.collisionRadius)
            || !std::isfinite(p.obstacleSafetyDistance)
            || !std::isfinite(p.terminalPositionTolerance))
        {
            error = "agent collision clearances must be finite";
            return false;
        }
        const int_t nx = p.model->stateDimension();
        const int_t nu = p.model->controlDimension();
        if (p.horizon <= 0 || p.horizon != N
            || p.model->positionDimension() <= 0)
        {
            error = "agents require a common positive horizon and position outputs";
            return false;
        }
        if (p.initialState.size() != static_cast<std::size_t>(nx)
            || p.stateReference.size() != static_cast<std::size_t>((N + 1) * nx)
            || p.controlReference.size() != static_cast<std::size_t>(N * nu)
            || p.stateWeights.size() != static_cast<std::size_t>(nx)
            || p.terminalWeights.size() != static_cast<std::size_t>(nx)
            || p.controlWeights.size() != static_cast<std::size_t>(nu)
            || (!p.controlDifferenceWeights.empty()
                && p.controlDifferenceWeights.size() != static_cast<std::size_t>(nu)))
        {
            error = "trajectory and weight dimensions are inconsistent";
            return false;
        }
        if ((!p.stateLowerBounds.empty() && p.stateLowerBounds.size() != static_cast<std::size_t>(nx))
            || (!p.stateUpperBounds.empty() && p.stateUpperBounds.size() != static_cast<std::size_t>(nx))
            || (!p.controlLowerBounds.empty() && p.controlLowerBounds.size() != static_cast<std::size_t>(nu))
            || (!p.controlUpperBounds.empty() && p.controlUpperBounds.size() != static_cast<std::size_t>(nu))
            || (!p.initialStates.empty()
                && p.initialStates.size()
                    != static_cast<std::size_t>((N + 1) * nx))
            || (!p.initialControls.empty()
                && p.initialControls.size() != static_cast<std::size_t>(N * nu)))
        {
            error = "bound or initial-control dimensions are inconsistent";
            return false;
        }
        const int_t corridorCircleCount = p.collisionCircles.empty()
            ? 1 : static_cast<int_t>(p.collisionCircles.size());
        const std::size_t corridorSize = static_cast<std::size_t>(
            (N + 1) * corridorCircleCount * p.model->positionDimension());
        if (p.collisionCorridorLowerBounds.empty()
            != p.collisionCorridorUpperBounds.empty()
            || (!p.collisionCorridorLowerBounds.empty()
                && (p.collisionCorridorLowerBounds.size() != corridorSize
                    || p.collisionCorridorUpperBounds.size() != corridorSize)))
        {
            error = "collision-corridor dimensions are inconsistent";
            return false;
        }
        for (std::size_t corridor = 0;
             corridor < p.collisionCorridorLowerBounds.size(); ++corridor)
        {
            const real_t lower = p.collisionCorridorLowerBounds[corridor];
            const real_t upper = p.collisionCorridorUpperBounds[corridor];
            if (!std::isfinite(lower) || !std::isfinite(upper)
                || lower > upper)
            {
                error = "collision corridors require finite ordered bounds";
                return false;
            }
        }
        if (!p.terminalStateConstraintMask.empty()
            && p.terminalStateConstraintMask.size()
                != static_cast<std::size_t>(nx))
        {
            error = "terminal-state constraint mask has the wrong dimension";
            return false;
        }
        for (std::size_t i = 0; i < p.controlDifferenceWeights.size(); ++i)
            if (!std::isfinite(p.controlDifferenceWeights[i])
                || p.controlDifferenceWeights[i] < 0.0)
            {
                error = "control-difference weights must be finite and nonnegative";
                return false;
            }
        if (!p.collisionCircles.empty() && p.model->positionDimension() != 2)
        {
            error = "body-fixed collision circles require a planar model";
            return false;
        }
        for (std::size_t circle = 0; circle < p.collisionCircles.size(); ++circle)
        {
            const CollisionCircle& geometry = p.collisionCircles[circle];
            if (!std::isfinite(geometry.longitudinalOffset)
                || !std::isfinite(geometry.lateralOffset)
                || !std::isfinite(geometry.radius)
                || geometry.radius <= 0.0)
            {
                error = "collision circles require finite offsets and positive radii";
                return false;
            }
        }
    }
    return true;
}

int_t worldPositionDimension(
    const std::vector<NonlinearAgentProblem>& agents
)
{
    int_t dimension = 0;
    for (std::size_t a = 0; a < agents.size(); ++a)
        dimension = std::max(
            dimension,
            agents[a].model->positionDimension()
        );
    return dimension;
}

void worldPosition(
    const NonlinearModel& model,
    const real_t* state,
    int_t worldDimension,
    real_t* position
)
{
    for (int_t i = 0; i < worldDimension; ++i) position[i] = 0.0;
    model.position(state, position);
}

int_t collisionCircleCount(const NonlinearAgentProblem& agent)
{
    return agent.collisionCircles.empty()
        ? 1 : static_cast<int_t>(agent.collisionCircles.size());
}

CollisionCircle collisionCircleForAgent(
    const NonlinearAgentProblem& agent,
    int_t circle,
    const NonlinearTurboOptions& options)
{
    if (!agent.collisionCircles.empty()) return agent.collisionCircles[circle];
    return CollisionCircle(0.0, 0.0, collisionRadiusForAgent(agent, options));
}

real_t obstacleDistanceForCircle(
    const NonlinearAgentProblem& agent,
    const CollisionCircle& circle,
    const NonlinearTurboOptions& options)
{
    if (agent.collisionCircles.empty())
        return obstacleDistanceForAgent(agent, options);
    const real_t extra = agent.obstacleSafetyDistance >= 0.0
        ? agent.obstacleSafetyDistance : options.obstacleSafetyDistance;
    return circle.radius + extra;
}

void worldCollisionPosition(
    const NonlinearAgentProblem& agent,
    int_t circle,
    const real_t* state,
    int_t worldDimension,
    real_t* position)
{
    for (int_t i = 0; i < worldDimension; ++i) position[i] = 0.0;
    if (agent.collisionCircles.empty())
    {
        agent.model->position(state, position);
        return;
    }
    const CollisionCircle& geometry = agent.collisionCircles[circle];
    agent.model->collisionPoint(
        state,
        geometry.longitudinalOffset,
        geometry.lateralOffset,
        position
    );
}

real_t endpointFeasiblePairBuffer(
    const NonlinearAgentProblem& first,
    int_t firstCircle,
    const NonlinearAgentProblem& second,
    int_t secondCircle,
    const NonlinearTurboOptions& options)
{
    real_t buffer = options.pairSafetyBuffer;
    if (buffer <= 0.0) return 0.0;
    const int_t dimension = std::max(
        first.model->positionDimension(), second.model->positionDimension());
    const CollisionCircle firstGeometry = collisionCircleForAgent(
        first, firstCircle, options);
    const CollisionCircle secondGeometry = collisionCircleForAgent(
        second, secondCircle, options);
    const real_t physicalDistance =
        firstGeometry.radius + secondGeometry.radius;
    std::vector<real_t> firstPosition(dimension, 0.0);
    std::vector<real_t> secondPosition(dimension, 0.0);
    const int_t firstNx = first.model->stateDimension();
    const int_t secondNx = second.model->stateDimension();
    for (int_t endpoint = 0; endpoint < 2; ++endpoint)
    {
        const real_t* firstState = endpoint == 0
            ? &first.initialState[0]
            : &first.stateReference[first.horizon * firstNx];
        const real_t* secondState = endpoint == 0
            ? &second.initialState[0]
            : &second.stateReference[second.horizon * secondNx];
        worldCollisionPosition(
            first, firstCircle, firstState, dimension, &firstPosition[0]);
        worldCollisionPosition(
            second, secondCircle, secondState, dimension, &secondPosition[0]);
        real_t squaredDistance = 0.0;
        for (int_t coordinate = 0; coordinate < dimension; ++coordinate)
            squaredDistance += (firstPosition[coordinate]
                - secondPosition[coordinate]) * (firstPosition[coordinate]
                - secondPosition[coordinate]);
        buffer = std::min(buffer, std::max(
            0.0, std::sqrt(squaredDistance) - physicalDistance));
    }
    return buffer;
}

void linearizeCollisionTrajectory(
    const NonlinearAgentProblem& agent,
    int_t circle,
    const std::vector<real_t>& states,
    int_t worldDimension,
    CollisionLinearization& linearization)
{
    const int_t N = agent.horizon;
    const int_t nx = agent.model->stateDimension();
    linearization.positions.assign((N + 1) * worldDimension, 0.0);
    linearization.C.assign((N + 1) * worldDimension * nx, 0.0);
    linearization.D.assign((N + 1) * worldDimension, 0.0);
    for (int_t k = 0; k <= N; ++k)
    {
        const real_t* state = &states[k * nx];
        real_t* position = &linearization.positions[k * worldDimension];
        real_t* C = &linearization.C[k * worldDimension * nx];
        worldCollisionPosition(agent, circle, state, worldDimension, position);
        if (agent.collisionCircles.empty())
            agent.model->linearizePosition(state, C);
        else
        {
            const CollisionCircle& geometry = agent.collisionCircles[circle];
            agent.model->linearizeCollisionPoint(
                state,
                geometry.longitudinalOffset,
                geometry.lateralOffset,
                C
            );
        }
        real_t* D = &linearization.D[k * worldDimension];
        for (int_t coordinate = 0; coordinate < worldDimension; ++coordinate)
        {
            D[coordinate] = position[coordinate];
            for (int_t stateIndex = 0; stateIndex < nx; ++stateIndex)
                D[coordinate] -= C[coordinate * nx + stateIndex]
                    * state[stateIndex];
        }
    }
}

void clampControls(const NonlinearAgentProblem& p, std::vector<real_t>& controls)
{
    const int_t nu = p.model->controlDimension();
    for (int_t k = 0; k < p.horizon; ++k)
        for (int_t j = 0; j < nu; ++j)
            controls[k * nu + j] = std::max(
                optionalBound(p.controlLowerBounds, j, -INFTY),
                std::min(optionalBound(p.controlUpperBounds, j, INFTY), controls[k * nu + j])
            );
}

void rollout(
    const NonlinearAgentProblem& p,
    const std::vector<real_t>& controls,
    std::vector<real_t>& states
)
{
    const int_t nx = p.model->stateDimension();
    const int_t nu = p.model->controlDimension();
    states.assign((p.horizon + 1) * nx, 0.0);
    std::copy(p.initialState.begin(), p.initialState.end(), states.begin());
    for (int_t k = 0; k < p.horizon; ++k)
        p.model->dynamics(&states[k * nx], &controls[k * nu], &states[(k + 1) * nx]);
}

void sampleStages(
    int_t sample,
    int_t samplesPerInterval,
    int_t horizon,
    int_t& firstStage,
    int_t& secondStage,
    real_t& alpha);

std::vector<ActiveObstacle> selectActiveObstacles(
    const NonlinearAgentProblem& p,
    const std::vector<real_t>& nominalStates,
    const std::vector<ConvexPolygonObstacle>& obstacles,
    real_t safetyDistance,
    real_t activationDistance,
    int_t samplesPerInterval)
{
    std::vector<ActiveObstacle> active;
    const int_t nx = p.model->stateDimension();
    const int_t worldDimension = p.model->positionDimension();
    const int_t sampleCount = p.horizon * samplesPerInterval;
    for (int_t circle = 0; circle < collisionCircleCount(p); ++circle)
    {
        const real_t requiredDistance = p.collisionCircles.empty()
            ? safetyDistance : p.collisionCircles[circle].radius + safetyDistance;
        std::vector<real_t> knots((p.horizon + 1) * worldDimension, 0.0);
        for (int_t stage = 0; stage <= p.horizon; ++stage)
            worldCollisionPosition(
                p,
                circle,
                &nominalStates[stage * nx],
                worldDimension,
                &knots[stage * worldDimension]
            );

        for (std::size_t obstacle = 0; obstacle < obstacles.size(); ++obstacle)
        {
            bool near = activationDistance < 0.0;
            for (int_t sample = 0; sample <= sampleCount && !near; ++sample)
            {
                int_t firstStage = 0, secondStage = 0;
                real_t alpha = 0.0;
                sampleStages(sample, samplesPerInterval, p.horizon,
                    firstStage, secondStage, alpha);
                const real_t x = (1.0 - alpha)
                    * knots[firstStage * worldDimension]
                    + alpha * knots[secondStage * worldDimension];
                const real_t y = (1.0 - alpha)
                    * knots[firstStage * worldDimension + 1]
                    + alpha * knots[secondStage * worldDimension + 1];
                near = projectToPolygon(
                    x, y, obstacles[obstacle]).signedDistance
                    <= requiredDistance + activationDistance;
            }
            if (near)
            {
                ActiveObstacle selected;
                selected.obstacle = obstacle;
                selected.circle = circle;
                selected.requiredDistance = requiredDistance;
                active.push_back(selected);
            }
        }
    }
    return active;
}

void buildAgentQp(
    const NonlinearAgentProblem& p,
    const std::vector<real_t>& nominalStates,
    const std::vector<real_t>& homotopyStates,
    const std::vector<real_t>& nominalControls,
    real_t trustRegion,
    bool elasticConstraints,
    real_t slackPenalty,
    const std::vector<ConvexPolygonObstacle>& obstacles,
    real_t obstacleSafetyDistance,
    real_t obstacleActivationDistance,
    real_t terminalPositionTolerance,
    int_t samplesPerInterval,
    int_t worldDimension,
    AgentQp& qp
)
{
    qp.N = p.horizon;
    qp.nx = p.model->stateDimension();
    qp.nu = p.model->controlDimension();
    qp.np = worldDimension;
    qp.trajectoryVariableCount = (qp.N + 1) * qp.nx + qp.N * qp.nu;
    const int_t dynamicsRows = qp.N * qp.nx;
    const int_t obstacleSamples = qp.N * samplesPerInterval;
    const int_t circleCount = collisionCircleCount(p);
    const int_t nativePositionDimension = p.model->positionDimension();
    const bool useCorridors = !p.collisionCorridorLowerBounds.empty();
    std::vector<ActiveObstacle> activeObstacles;
    if (!useCorridors)
        activeObstacles = selectActiveObstacles(
            p, nominalStates, obstacles, obstacleSafetyDistance,
            obstacleActivationDistance, samplesPerInterval);
    qp.activeObstacleCount = static_cast<int_t>(activeObstacles.size());

    const int_t obstacleRows = obstacleSamples * qp.activeObstacleCount;
    qp.corridorRows = useCorridors
        ? (qp.N + 1) * circleCount * nativePositionDimension : 0;
    const int_t corridorConstraintRows = elasticConstraints
        ? 2 * qp.corridorRows : qp.corridorRows;
    const int_t terminalRows = p.model->positionDimension();
    const int_t obstacleSlackCount = qp.N * qp.activeObstacleCount;
    const int_t corridorSlackCount = qp.corridorRows > 0 ? 1 : 0;
    qp.slackVariableCount = elasticConstraints
        ? terminalRows + obstacleSlackCount + corridorSlackCount : 0;
    qp.nV = qp.trajectoryVariableCount + qp.slackVariableCount;
    qp.nC = dynamicsRows + obstacleRows + corridorConstraintRows
        + (elasticConstraints ? 2 * terminalRows : terminalRows);
    qp.Hbase.assign(qp.nV * qp.nV, 0.0);
    qp.gbase.assign(qp.nV, 0.0);
    qp.A.assign(qp.nC * qp.nV, 0.0);
    qp.lb.assign(qp.nV, -INFTY); qp.ub.assign(qp.nV, INFTY);
    qp.lbA.assign(qp.nC, 0.0); qp.ubA.assign(qp.nC, 0.0);
    qp.dynamicsA.assign(qp.N * qp.nx * qp.nx, 0.0);
    qp.dynamicsB.assign(qp.N * qp.nx * qp.nu, 0.0);
    qp.dynamicsC.assign(qp.N * qp.nx, 0.0);
    qp.positionC.assign((qp.N + 1) * qp.np * qp.nx, 0.0);
    qp.positionD.assign((qp.N + 1) * qp.np, 0.0);
    qp.solution.assign(qp.nV, 0.0);
    for (int_t k = 0; k <= qp.N; ++k)
    {
        std::copy(
            nominalStates.begin() + k * qp.nx,
            nominalStates.begin() + (k + 1) * qp.nx,
            qp.solution.begin() + stateIndex(qp, k)
        );
        if (k < qp.N)
            std::copy(nominalControls.begin() + k * qp.nu,
                nominalControls.begin() + (k + 1) * qp.nu,
                qp.solution.begin() + controlIndex(qp, k));
    }
    for (int_t i = 0; i < qp.nV; ++i) qp.Hbase[i * qp.nV + i] = 1.0e-9;
    for (int_t slack = 0; slack < qp.slackVariableCount; ++slack)
    {
        const int_t index = qp.trajectoryVariableCount + slack;
        qp.gbase[index] = slackPenalty;
        qp.lb[index] = 0.0;
        qp.ub[index] = INFTY;
    }

    for (int_t k = 0; k <= qp.N; ++k)
    {
        const int_t offset = stateIndex(qp, k);
        const std::vector<real_t>& weights = k == qp.N ? p.terminalWeights : p.stateWeights;
        for (int_t i = 0; i < qp.nx; ++i)
        {
            qp.Hbase[(offset + i) * qp.nV + offset + i] += weights[i];
            qp.gbase[offset + i] -= weights[i] * p.stateReference[k * qp.nx + i];
            qp.lb[offset + i] = optionalBound(p.stateLowerBounds, i, -INFTY);
            qp.ub[offset + i] = optionalBound(p.stateUpperBounds, i, INFTY);
        }
        real_t* C = &qp.positionC[k * qp.np * qp.nx];
        real_t* d = &qp.positionD[k * qp.np];
        const int_t nativeDimension = p.model->positionDimension();
        std::vector<real_t> position(nativeDimension, 0.0);
        std::vector<real_t> nativeC(nativeDimension * qp.nx, 0.0);
        p.model->position(
            &nominalStates[k * qp.nx],
            &position[0]
        );
        p.model->linearizePosition(
            &nominalStates[k * qp.nx],
            &nativeC[0]
        );
        for (int_t i = 0; i < nativeDimension; ++i)
        {
            d[i] = position[i];
            for (int_t j = 0; j < qp.nx; ++j)
            {
                C[i * qp.nx + j] = nativeC[i * qp.nx + j];
                d[i] -= C[i * qp.nx + j]
                    * nominalStates[k * qp.nx + j];
            }
        }
    }
    for (int_t i = 0; i < qp.nx; ++i) qp.lb[i] = qp.ub[i] = p.initialState[i];

    if (p.enforceTerminalState)
    {
        const int_t terminalDecision = stateIndex(qp, qp.N);
        for (int_t i = 0; i < qp.nx; ++i)
        {
            if (!p.terminalStateConstraintMask.empty()
                && !p.terminalStateConstraintMask[i])
                continue;
            qp.lb[terminalDecision + i] = qp.ub[terminalDecision + i]
                = p.stateReference[qp.N * qp.nx + i];
        }
    }


    std::vector<real_t> terminalReference(terminalRows, 0.0);
    p.model->position(
        &p.stateReference[qp.N * qp.nx],
        &terminalReference[0]
    );
    const real_t terminalCoordinateTolerance = terminalPositionTolerance
        / std::sqrt(static_cast<real_t>(terminalRows));
    const int_t terminalOffset = dynamicsRows
        + obstacleSamples * qp.activeObstacleCount + corridorConstraintRows;
    const int_t terminalState = stateIndex(qp, qp.N);
    const real_t* terminalC = &qp.positionC[qp.N * qp.np * qp.nx];
    const real_t* terminalD = &qp.positionD[qp.N * qp.np];
    for (int_t coordinate = 0; coordinate < terminalRows; ++coordinate)
    {
        const real_t lower = terminalReference[coordinate]
            - terminalCoordinateTolerance - terminalD[coordinate];
        const real_t upper = terminalReference[coordinate]
            + terminalCoordinateTolerance - terminalD[coordinate];
        if (!elasticConstraints)
        {
            const int_t row = terminalOffset + coordinate;
            for (int_t state = 0; state < qp.nx; ++state)
                qp.A[row * qp.nV + terminalState + state]
                    = terminalC[coordinate * qp.nx + state];
            qp.lbA[row] = lower;
            qp.ubA[row] = upper;
            continue;
        }
        const int_t lowerRow = terminalOffset + 2 * coordinate;
        const int_t upperRow = lowerRow + 1;
        const int_t slack = qp.trajectoryVariableCount + coordinate;
        for (int_t state = 0; state < qp.nx; ++state)
        {
            const real_t value = terminalC[coordinate * qp.nx + state];
            qp.A[lowerRow * qp.nV + terminalState + state] = value;
            qp.A[upperRow * qp.nV + terminalState + state] = value;
        }
        qp.A[lowerRow * qp.nV + slack] = 1.0;
        qp.lbA[lowerRow] = lower;
        qp.ubA[lowerRow] = INFTY;
        qp.A[upperRow * qp.nV + slack] = -1.0;
        qp.lbA[upperRow] = -INFTY;
        qp.ubA[upperRow] = upper;
    }
    for (int_t k = 0; k < qp.N; ++k)
    {
        const int_t uOffset = controlIndex(qp, k);
        for (int_t i = 0; i < qp.nu; ++i)
        {
            qp.Hbase[(uOffset + i) * qp.nV + uOffset + i] += p.controlWeights[i];
            qp.gbase[uOffset + i] -= p.controlWeights[i] * p.controlReference[k * qp.nu + i];
            real_t lower = optionalBound(p.controlLowerBounds, i, -INFTY);
            real_t upper = optionalBound(p.controlUpperBounds, i, INFTY);
            if (trustRegion > 0.0)
            {
                lower = std::max(lower, nominalControls[k * qp.nu + i] - trustRegion);
                upper = std::min(upper, nominalControls[k * qp.nu + i] + trustRegion);
            }
            qp.lb[uOffset + i] = lower; qp.ub[uOffset + i] = upper;
        }
        real_t* Ak = &qp.dynamicsA[k * qp.nx * qp.nx];
        real_t* Bk = &qp.dynamicsB[k * qp.nx * qp.nu];
        real_t* ck = &qp.dynamicsC[k * qp.nx];
        std::vector<real_t> next(qp.nx, 0.0);
        p.model->dynamics(&nominalStates[k * qp.nx], &nominalControls[k * qp.nu], &next[0]);
        p.model->linearizeDynamics(&nominalStates[k * qp.nx], &nominalControls[k * qp.nu], Ak, Bk);
        for (int_t i = 0; i < qp.nx; ++i)
        {
            ck[i] = next[i];
            for (int_t j = 0; j < qp.nx; ++j) ck[i] -= Ak[i * qp.nx + j] * nominalStates[k * qp.nx + j];
            for (int_t j = 0; j < qp.nu; ++j) ck[i] -= Bk[i * qp.nu + j] * nominalControls[k * qp.nu + j];
            const int_t row = k * qp.nx + i;
            const int_t xOffset = stateIndex(qp, k);
            const int_t nextOffset = stateIndex(qp, k + 1);
            for (int_t j = 0; j < qp.nx; ++j) qp.A[row * qp.nV + xOffset + j] = Ak[i * qp.nx + j];
            for (int_t j = 0; j < qp.nu; ++j) qp.A[row * qp.nV + uOffset + j] = Bk[i * qp.nu + j];
            qp.A[row * qp.nV + nextOffset + i] = -1.0;
            qp.lbA[row] = qp.ubA[row] = -ck[i];
        }
    }
    for (int_t k = 0; k + 1 < qp.N; ++k)
    {
        const int_t firstControl = controlIndex(qp, k);
        const int_t secondControl = controlIndex(qp, k + 1);
        for (int_t i = 0; i < qp.nu; ++i)
        {
            const real_t weight = p.controlDifferenceWeights.empty()
                ? 0.0 : p.controlDifferenceWeights[i];
            qp.Hbase[(firstControl + i) * qp.nV + firstControl + i] += weight;
            qp.Hbase[(secondControl + i) * qp.nV + secondControl + i] += weight;
            qp.Hbase[(firstControl + i) * qp.nV + secondControl + i] -= weight;
            qp.Hbase[(secondControl + i) * qp.nV + firstControl + i] -= weight;
        }
    }
    std::vector<CollisionLinearization> nominalCollisions(circleCount);
    std::vector<CollisionLinearization> homotopyCollisions(circleCount);
    std::vector<CollisionLinearization> referenceCollisions(circleCount);
    if (!activeObstacles.empty() || useCorridors)
    {
    for (int_t circle = 0; circle < circleCount; ++circle)
    {
        linearizeCollisionTrajectory(
            p, circle, nominalStates, qp.np, nominalCollisions[circle]);
        linearizeCollisionTrajectory(
            p, circle, homotopyStates, qp.np, homotopyCollisions[circle]);
        linearizeCollisionTrajectory(
            p, circle, p.stateReference, qp.np, referenceCollisions[circle]);
    }
    }
    for (std::size_t activeObstacle = 0;
         activeObstacle < activeObstacles.size();
         ++activeObstacle)
    {
        const ActiveObstacle& active = activeObstacles[activeObstacle];
        const std::size_t obstacleIndex = active.obstacle;
        const CollisionLinearization& nominalCollision =
            nominalCollisions[active.circle];
        const CollisionLinearization& homotopyCollision =
            homotopyCollisions[active.circle];
        const CollisionLinearization& referenceCollision =
            referenceCollisions[active.circle];
        bool homotopyIsObstacleFeasible = true;
        for (int_t witnessSample = 1;
             witnessSample <= obstacleSamples;
             ++witnessSample)
        {
            int_t firstStage = 0, secondStage = 0;
            real_t alpha = 0.0;
            sampleStages(
                witnessSample,
                samplesPerInterval,
                qp.N,
                firstStage,
                secondStage,
                alpha
            );
            const int_t stages[2] = {firstStage, secondStage};
            const real_t weights[2] = {1.0 - alpha, alpha};
            std::vector<real_t> witnessPosition(qp.np, 0.0);
            for (int_t endpoint = 0; endpoint < 2; ++endpoint)
            {
                if (weights[endpoint] <= 0.0) continue;
                for (int_t i = 0; i < qp.np; ++i)
                    witnessPosition[i] += weights[endpoint]
                        * homotopyCollision.positions[
                            stages[endpoint] * qp.np + i];
            }
            if (projectToPolygon(
                    witnessPosition[0], witnessPosition[1],
                    obstacles[obstacleIndex]).signedDistance
                < active.requiredDistance - 1.0e-9)
            {
                homotopyIsObstacleFeasible = false;
                break;
            }
        }
        const ObstacleBypass bypass = buildObstacleBypass(
            p,
            active.circle,
            obstacles[obstacleIndex],
            active.requiredDistance
        );
        for (int_t sample = 1; sample <= obstacleSamples; ++sample)
        {
            int_t firstStage = 0, secondStage = 0;
            real_t alpha = 0.0;
            sampleStages(
                sample,
                samplesPerInterval,
                qp.N,
                firstStage,
                secondStage,
                alpha
            );
            const int_t stages[2] = {firstStage, secondStage};
            const real_t weights[2] = {1.0 - alpha, alpha};
            std::vector<real_t> nominalPosition(qp.np, 0.0);
            std::vector<real_t> referencePosition(qp.np, 0.0);
            for (int_t endpoint = 0; endpoint < 2; ++endpoint)
            {
                if (weights[endpoint] <= 0.0) continue;
                for (int_t i = 0; i < qp.np; ++i)
                {
                    nominalPosition[i] += weights[endpoint]
                        * nominalCollision.positions[
                            stages[endpoint] * qp.np + i];
                    referencePosition[i] += weights[endpoint]
                        * referenceCollision.positions[
                            stages[endpoint] * qp.np + i];
                }
            }
            const PolygonProjection projection = projectToPolygon(
                nominalPosition[0],
                nominalPosition[1],
                obstacles[obstacleIndex]
            );
            const real_t referenceTravel =
                bypass.direction[0] * referencePosition[0]
                + bypass.direction[1] * referencePosition[1];
            real_t normal[2] = {
                projection.normal[0], projection.normal[1]
            };
            real_t support = normal[0] * projection.closest[0]
                + normal[1] * projection.closest[1];
            real_t bypassNormal[2] = {normal[0], normal[1]};
            real_t bypassSupport = support;
            const bool hasBypass = bypassHalfspaceAt(
                bypass,
                obstacles[obstacleIndex],
                active.requiredDistance,
                referenceTravel,
                bypassNormal,
                bypassSupport
            );
            const real_t nominalBypassValue =
                bypassNormal[0] * nominalPosition[0]
                + bypassNormal[1] * nominalPosition[1];
            if (hasBypass
                && (!homotopyIsObstacleFeasible
                    || nominalBypassValue
                        >= bypassSupport + active.requiredDistance - 1.0e-9))
            {
                normal[0] = bypassNormal[0];
                normal[1] = bypassNormal[1];
                support = bypassSupport;
            }
            const int_t row = dynamicsRows
                + static_cast<int_t>(activeObstacle) * obstacleSamples + sample - 1;
            real_t affine = 0.0;
            for (int_t endpoint = 0; endpoint < 2; ++endpoint)
            {
                if (weights[endpoint] <= 0.0) continue;
                const int_t stage = stages[endpoint];
                const int_t offset = stateIndex(qp, stage);
                const real_t* C = &nominalCollision.C[
                    stage * qp.np * qp.nx];
                const real_t* d = &nominalCollision.D[
                    stage * qp.np];
                for (int_t j = 0; j < qp.nx; ++j)
                    qp.A[row * qp.nV + offset + j] += weights[endpoint]
                        * (normal[0] * C[j] + normal[1] * C[qp.nx + j]);
                affine += weights[endpoint]
                    * (normal[0] * d[0] + normal[1] * d[1]);
            }
            qp.lbA[row] = support + active.requiredDistance - affine;
            qp.ubA[row] = INFTY;
            if (elasticConstraints)
            {
                const int_t interval = (sample - 1) / samplesPerInterval;
                const int_t slack = qp.trajectoryVariableCount + terminalRows
                    + static_cast<int_t>(activeObstacle) * qp.N + interval;
                qp.A[row * qp.nV + slack] = 1.0;
            }
        }
    }
    if (useCorridors)
    {
        const int_t corridorOffset = dynamicsRows + obstacleRows;
        for (int_t circle = 0; circle < circleCount; ++circle)
        for (int_t stage = 0; stage <= qp.N; ++stage)
        {
            const CollisionLinearization& collision = nominalCollisions[circle];
            const int_t stateOffset = stateIndex(qp, stage);
            const real_t* C = &collision.C[stage * qp.np * qp.nx];
            const real_t* d = &collision.D[stage * qp.np];
            for (int_t coordinate = 0;
                 coordinate < nativePositionDimension; ++coordinate)
            {
                const int_t corridorIndex =
                    (stage * circleCount + circle) * nativePositionDimension
                    + coordinate;
                const real_t lower =
                    p.collisionCorridorLowerBounds[corridorIndex]
                        - d[coordinate];
                const real_t upper =
                    p.collisionCorridorUpperBounds[corridorIndex]
                        - d[coordinate];
                if (!elasticConstraints)
                {
                    const int_t row = corridorOffset + corridorIndex;
                    for (int_t state = 0; state < qp.nx; ++state)
                        qp.A[row * qp.nV + stateOffset + state] =
                            C[coordinate * qp.nx + state];
                    qp.lbA[row] = lower;
                    qp.ubA[row] = upper;
                    continue;
                }
                const int_t lowerRow = corridorOffset + 2 * corridorIndex;
                const int_t upperRow = lowerRow + 1;
                const int_t slackOffset = qp.trajectoryVariableCount
                    + terminalRows + obstacleSlackCount;
                for (int_t state = 0; state < qp.nx; ++state)
                {
                    const real_t value = C[coordinate * qp.nx + state];
                    qp.A[lowerRow * qp.nV + stateOffset + state] = value;
                    qp.A[upperRow * qp.nV + stateOffset + state] = value;
                }
                qp.A[lowerRow * qp.nV + slackOffset] = 1.0;
                qp.lbA[lowerRow] = lower;
                qp.ubA[lowerRow] = INFTY;
                qp.A[upperRow * qp.nV + slackOffset] = -1.0;
                qp.lbA[upperRow] = -INFTY;
                qp.ubA[upperRow] = upper;
            }
        }
    }
    qp.H = qp.Hbase; qp.g = qp.gbase;
}

real_t maximumRestorationSlack(const AgentQp& qp)
{
    real_t maximum = 0.0;
    for (int_t slack = 0; slack < qp.slackVariableCount; ++slack)
        maximum = std::max(maximum,
            std::max(0.0, qp.solution[qp.trajectoryVariableCount + slack]));
    return maximum;
}

void positionFromDecision(
    const AgentQp& qp,
    const std::vector<real_t>& CValues,
    const std::vector<real_t>& DValues,
    int_t k, const std::vector<real_t>& z, real_t* position
)
{
    const real_t* C = &CValues[k * qp.np * qp.nx];
    const real_t* d = &DValues[k * qp.np];
    const int_t offset = stateIndex(qp, k);
    for (int_t i = 0; i < qp.np; ++i)
    {
        position[i] = d[i];
        for (int_t j = 0; j < qp.nx; ++j) position[i] += C[i * qp.nx + j] * z[offset + j];
    }
}

void sampleStages(
    int_t sample,
    int_t samplesPerInterval,
    int_t horizon,
    int_t& firstStage,
    int_t& secondStage,
    real_t& alpha)
{
    if (sample >= horizon * samplesPerInterval)
    {
        firstStage = horizon;
        secondStage = horizon;
        alpha = 0.0;
        return;
    }
    firstStage = sample / samplesPerInterval;
    secondStage = firstStage + 1;
    alpha = static_cast<real_t>(sample % samplesPerInterval)
        / samplesPerInterval;
}

void positionFromDecisionSample(
    const AgentQp& qp,
    const std::vector<real_t>& CValues,
    const std::vector<real_t>& DValues,
    int_t sample,
    int_t samplesPerInterval,
    const std::vector<real_t>& decision,
    real_t* position)
{
    int_t firstStage = 0, secondStage = 0;
    real_t alpha = 0.0;
    sampleStages(
        sample,
        samplesPerInterval,
        qp.N,
        firstStage,
        secondStage,
        alpha
    );
    std::vector<real_t> first(qp.np, 0.0), second(qp.np, 0.0);
    positionFromDecision(
        qp, CValues, DValues, firstStage, decision, &first[0]);
    positionFromDecision(
        qp, CValues, DValues, secondStage, decision, &second[0]);
    for (int_t i = 0; i < qp.np; ++i)
        position[i] = (1.0 - alpha) * first[i] + alpha * second[i];
}

std::vector<PairData> buildPairs(
    const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<std::vector<real_t> >& states,
    int_t np,
    const NonlinearTurboOptions& options,
    const std::vector<PairData>* previousPairs,
    NonlinearTurboStatistics& stats)
{
    std::vector<PairData> pairs;
    const int_t N = agents[0].horizon;
    int_t potentialPairs = 0;
    for (std::size_t first = 0; first < agents.size(); ++first)
        for (std::size_t second = first + 1; second < agents.size(); ++second)
            potentialPairs += collisionCircleCount(agents[first])
                * collisionCircleCount(agents[second]);
    stats.maximumPotentialPairs = std::max(stats.maximumPotentialPairs, potentialPairs);
    for (std::size_t first = 0; first < agents.size(); ++first)
    for (std::size_t second = first + 1; second < agents.size(); ++second)
    {
      for (int_t firstCircle = 0; firstCircle < collisionCircleCount(agents[first]); ++firstCircle)
      for (int_t secondCircle = 0; secondCircle < collisionCircleCount(agents[second]); ++secondCircle)
      {
        PairData pair; pair.first = static_cast<int_t>(first); pair.second = static_cast<int_t>(second);
        pair.firstCircle = firstCircle; pair.secondCircle = secondCircle;
        pair.samplesPerInterval = options.collisionSamplesPerInterval;
        const CollisionCircle firstGeometry = collisionCircleForAgent(
            agents[first], firstCircle, options);
        const CollisionCircle secondGeometry = collisionCircleForAgent(
            agents[second], secondCircle, options);
        pair.safetyDistance = firstGeometry.radius + secondGeometry.radius
            + endpointFeasiblePairBuffer(
                agents[first], firstCircle, agents[second], secondCircle, options);
        const PairData* previous = 0;
        if (previousPairs != 0)
            for (std::size_t oldPair = 0; oldPair < previousPairs->size(); ++oldPair)
            {
                const PairData& candidate = (*previousPairs)[oldPair];
                if (candidate.first == pair.first && candidate.second == pair.second
                    && candidate.firstCircle == pair.firstCircle
                    && candidate.secondCircle == pair.secondCircle)
                {
                    previous = &candidate;
                    break;
                }
            }
        const int_t sampleCount = N * pair.samplesPerInterval + 1;
        const int_t count = sampleCount * np;
        pair.normal.assign(count, 0.0);
        pair.firstPosition.assign(count, 0.0); pair.secondPosition.assign(count, 0.0);
        pair.firstAuxiliary.assign(count, 0.0); pair.secondAuxiliary.assign(count, 0.0);
        pair.firstDual.assign(count, 0.0); pair.secondDual.assign(count, 0.0);
        pair.previousFirstAuxiliary.assign(count, 0.0); pair.previousSecondAuxiliary.assign(count, 0.0);
        CollisionLinearization firstLinearization, secondLinearization;
        linearizeCollisionTrajectory(
            agents[first], firstCircle, states[first], np, firstLinearization);
        linearizeCollisionTrajectory(
            agents[second], secondCircle, states[second], np, secondLinearization);
        const std::vector<real_t>& firstKnots = firstLinearization.positions;
        const std::vector<real_t>& secondKnots = secondLinearization.positions;
        pair.firstC.swap(firstLinearization.C);
        pair.firstD.swap(firstLinearization.D);
        pair.secondC.swap(secondLinearization.C);
        pair.secondD.swap(secondLinearization.D);
        bool active = options.pairActivationDistance < 0.0;
        if (!active)
        {
            const real_t activationDistance = pair.safetyDistance
                + options.pairActivationDistance;
            const real_t activationSquared = activationDistance * activationDistance;
            for (int_t sample = 0; sample < sampleCount && !active; ++sample)
            {
                int_t firstStage = 0, secondStage = 0;
                real_t alpha = 0.0;
                sampleStages(sample, pair.samplesPerInterval, N,
                    firstStage, secondStage, alpha);
                real_t distanceSquared = 0.0;
                for (int_t i = 0; i < np; ++i)
                {
                    const real_t firstPosition =
                        (1.0 - alpha) * firstKnots[firstStage * np + i]
                            + alpha * firstKnots[secondStage * np + i];
                    const real_t secondPosition =
                        (1.0 - alpha) * secondKnots[firstStage * np + i]
                            + alpha * secondKnots[secondStage * np + i];
                    const real_t difference = firstPosition - secondPosition;
                    distanceSquared += difference * difference;
                }
                active = distanceSquared <= activationSquared;
            }
        }
        if (!active) continue;
        for (int_t sample = 0; sample < sampleCount; ++sample)
        {
            int_t firstStage = 0, secondStage = 0;
            real_t alpha = 0.0;
            sampleStages(sample, pair.samplesPerInterval, N, firstStage, secondStage, alpha);
            real_t* pf = &pair.firstPosition[sample * np];
            real_t* ps = &pair.secondPosition[sample * np];
            for (int_t i = 0; i < np; ++i)
            {
                pf[i] = (1.0 - alpha) * firstKnots[firstStage * np + i]
                    + alpha * firstKnots[secondStage * np + i];
                ps[i] = (1.0 - alpha) * secondKnots[firstStage * np + i]
                    + alpha * secondKnots[secondStage * np + i];
            }
            real_t norm = 0.0;
            for (int_t i = 0; i < np; ++i)
            {
                pair.normal[sample * np + i] = pf[i] - ps[i];
                norm += pair.normal[sample * np + i] * pair.normal[sample * np + i];
            }
            norm = std::sqrt(norm);
            if (norm < 1.0e-9)
            {
                if (sample > 0) for (int_t i = 0; i < np; ++i)
                    pair.normal[sample * np + i] = pair.normal[(sample - 1) * np + i];
                else pair.normal[0] = 1.0;
            }
            else for (int_t i = 0; i < np; ++i) pair.normal[sample * np + i] /= norm;
            real_t signedDistance = 0.0;
            for (int_t i = 0; i < np; ++i)
                signedDistance += pair.normal[sample * np + i] * (pf[i] - ps[i]);
            const real_t correction = 0.5 * std::max(0.0, pair.safetyDistance - signedDistance);
            for (int_t i = 0; i < np; ++i)
            {
                pair.firstAuxiliary[sample * np + i] = pf[i] + correction * pair.normal[sample * np + i];
                pair.secondAuxiliary[sample * np + i] = ps[i] - correction * pair.normal[sample * np + i];
            }

            bool transported = false;
            if (previous != 0
                && previous->normal.size() == static_cast<std::size_t>(count)
                && previous->firstAuxiliary.size() == static_cast<std::size_t>(count)
                && previous->secondAuxiliary.size() == static_cast<std::size_t>(count)
                && previous->firstDual.size() == static_cast<std::size_t>(count)
                && previous->secondDual.size() == static_cast<std::size_t>(count))
            {
                real_t alignment = 0.0;
                real_t firstMultiplier = 0.0;
                real_t secondMultiplier = 0.0;
                for (int_t i = 0; i < np; ++i)
                {
                    const int_t index = sample * np + i;
                    alignment += previous->normal[index] * pair.normal[index];
                    firstMultiplier += previous->firstDual[index]
                        * previous->normal[index];
                    secondMultiplier += previous->secondDual[index]
                        * previous->normal[index];
                }
                if (std::isfinite(alignment)
                    && alignment >= options.continuationMinimumNormalDot)
                {
                    real_t transportedDistance = 0.0;
                    for (int_t i = 0; i < np; ++i)
                    {
                        const int_t index = sample * np + i;
                        pair.firstAuxiliary[index] = previous->firstAuxiliary[index];
                        pair.secondAuxiliary[index] = previous->secondAuxiliary[index];
                        transportedDistance += pair.normal[index]
                            * (pair.firstAuxiliary[index] - pair.secondAuxiliary[index]);
                    }
                    const real_t transportCorrection = 0.5 * std::max(
                        0.0,
                        pair.safetyDistance - transportedDistance
                    );
                    for (int_t i = 0; i < np; ++i)
                    {
                        const int_t index = sample * np + i;
                        pair.firstAuxiliary[index] += transportCorrection * pair.normal[index];
                        pair.secondAuxiliary[index] -= transportCorrection * pair.normal[index];
                        pair.firstDual[index] = firstMultiplier * pair.normal[index];
                        pair.secondDual[index] = secondMultiplier * pair.normal[index];
                        pair.previousFirstAuxiliary[index] = pair.firstAuxiliary[index];
                        pair.previousSecondAuxiliary[index] = pair.secondAuxiliary[index];
                    }
                    transported = true;
                    ++stats.transportedPairStages;
                }
            }
            if (previous != 0 && !transported) ++stats.resetPairStages;
        }
        pairs.push_back(pair);
      }
    }
    stats.maximumActivePairs = std::max(
        stats.maximumActivePairs, static_cast<int_t>(pairs.size()));
    std::vector<int_t> degrees(agents.size(), 0);
    for (std::size_t p = 0; p < pairs.size(); ++p)
    {
        ++degrees[pairs[p].first];
        ++degrees[pairs[p].second];
    }
    for (std::size_t a = 0; a < degrees.size(); ++a)
        stats.maximumAgentDegree = std::max(stats.maximumAgentDegree, degrees[a]);
    return pairs;
}

void addAdmmTerms(int_t agentIndex, const std::vector<PairData>& pairs,
    const std::vector<std::size_t>& incidentPairs, real_t rho,
    bool includeHessian, AgentQp& qp)
{
    for (std::size_t incident = 0; incident < incidentPairs.size(); ++incident)
    {
        const std::size_t pairIndex = incidentPairs[incident];
        const PairData& pair = pairs[pairIndex];
        const bool isFirst = pair.first == agentIndex;
        if (!isFirst && pair.second != agentIndex) continue;
        const std::vector<real_t>& v = isFirst ? pair.firstAuxiliary : pair.secondAuxiliary;
        const std::vector<real_t>& dual = isFirst ? pair.firstDual : pair.secondDual;
        const std::vector<real_t>& collisionC =
            isFirst ? pair.firstC : pair.secondC;
        const std::vector<real_t>& collisionD =
            isFirst ? pair.firstD : pair.secondD;
        const int_t sampleCount = static_cast<int_t>(v.size()) / qp.np;
        for (int_t sample = 0; sample < sampleCount; ++sample)
        {
            int_t firstStage = 0, secondStage = 0;
            real_t alpha = 0.0;
            sampleStages(
                sample,
                pair.samplesPerInterval,
                qp.N,
                firstStage,
                secondStage,
                alpha
            );
            const int_t stages[2] = {firstStage, secondStage};
            const real_t weights[2] = {1.0 - alpha, alpha};
            std::vector<real_t> affine(qp.np, 0.0);
            for (int_t endpoint = 0; endpoint < 2; ++endpoint)
            {
                if (weights[endpoint] <= 0.0) continue;
                const real_t* d = &collisionD[stages[endpoint] * qp.np];
                for (int_t coordinate = 0; coordinate < qp.np; ++coordinate)
                    affine[coordinate] += weights[endpoint] * d[coordinate];
            }
            for (int_t endpoint = 0; endpoint < 2; ++endpoint)
            {
                if (weights[endpoint] <= 0.0) continue;
                const int_t offset = stateIndex(qp, stages[endpoint]);
                const real_t* C = &collisionC[
                    stages[endpoint] * qp.np * qp.nx
                ];
                for (int_t i = 0; i < qp.nx; ++i)
                {
                    real_t gradient = 0.0;
                    for (int_t coordinate = 0; coordinate < qp.np; ++coordinate)
                        gradient += weights[endpoint] * C[coordinate * qp.nx + i]
                            * (affine[coordinate] - v[sample * qp.np + coordinate]
                                + dual[sample * qp.np + coordinate]);
                    qp.g[offset + i] += rho * gradient;
                    if (!includeHessian) continue;
                    for (int_t otherEndpoint = 0; otherEndpoint < 2; ++otherEndpoint)

                    {
                        if (weights[otherEndpoint] <= 0.0) continue;
                        const int_t otherOffset = stateIndex(qp, stages[otherEndpoint]);
                        const real_t* otherC = &collisionC[
                            stages[otherEndpoint] * qp.np * qp.nx
                        ];
                        for (int_t j = 0; j < qp.nx; ++j)
                        {
                            real_t hessian = 0.0;
                            for (int_t coordinate = 0; coordinate < qp.np; ++coordinate)
                                hessian += weights[endpoint] * weights[otherEndpoint]
                                    * C[coordinate * qp.nx + i]
                                    * otherC[coordinate * qp.nx + j];
                            qp.H[(offset + i) * qp.nV + otherOffset + j]
                                += rho * hessian;
                        }
                    }
                }
            }
        }
    }
}

bool riccatiWarmStart(
    const NonlinearAgentProblem& p,
    const AgentQp& qp,
    std::vector<real_t>& warm)
{
    const int_t augmentedNx = qp.nx + qp.nu;
    StageVaryingLqrProblem lqr;
    StageVaryingLqrSolution solution;
    lqr.horizon = qp.N;
    lqr.nx = augmentedNx;
    lqr.nu = qp.nu;
    lqr.x0.assign(augmentedNx, 0.0);
    std::copy(p.initialState.begin(), p.initialState.end(), lqr.x0.begin());
    for (int_t control = 0; control < qp.nu; ++control)
        lqr.x0[qp.nx + control] =
            p.controlReference.empty() ? 0.0 : p.controlReference[control];
    lqr.A.assign(qp.N * augmentedNx * augmentedNx, 0.0);
    lqr.B.assign(qp.N * augmentedNx * qp.nu, 0.0);
    lqr.c.assign(qp.N * augmentedNx, 0.0);
    lqr.Q.assign(qp.N * augmentedNx * augmentedNx, 0.0);
    lqr.R.assign(qp.N * qp.nu * qp.nu, 0.0);
    lqr.stateControl.assign(qp.N * augmentedNx * qp.nu, 0.0);
    lqr.q.assign(qp.N * augmentedNx, 0.0);
    lqr.r.assign(qp.N * qp.nu, 0.0);
    lqr.Qterminal.assign(augmentedNx * augmentedNx, 0.0);
    lqr.qterminal.assign(augmentedNx, 0.0);
    for (int_t k = 0; k < qp.N; ++k)
    {
        const real_t* Ak = &qp.dynamicsA[k * qp.nx * qp.nx];
        const real_t* Bk = &qp.dynamicsB[k * qp.nx * qp.nu];
        for (int_t row = 0; row < qp.nx; ++row)
        {
            for (int_t column = 0; column < qp.nx; ++column)
                lqr.A[k * augmentedNx * augmentedNx
                    + row * augmentedNx + column] = Ak[row * qp.nx + column];
            for (int_t control = 0; control < qp.nu; ++control)
                lqr.B[k * augmentedNx * qp.nu + row * qp.nu + control] =
                    Bk[row * qp.nu + control];
            lqr.c[k * augmentedNx + row] = qp.dynamicsC[k * qp.nx + row];
        }
        for (int_t control = 0; control < qp.nu; ++control)
            lqr.B[k * augmentedNx * qp.nu
                + (qp.nx + control) * qp.nu + control] = 1.0;
        for (int_t i = 0; i < qp.nx; ++i)
        {
            lqr.Q[k * augmentedNx * augmentedNx + i * augmentedNx + i]
                = p.stateWeights[i];
            lqr.q[k * augmentedNx + i] =
                -p.stateWeights[i] * p.stateReference[k * qp.nx + i];
        }
        for (int_t i = 0; i < qp.nu; ++i)
        {
            const real_t differenceWeight = k == 0
                || p.controlDifferenceWeights.empty()
                ? 0.0 : p.controlDifferenceWeights[i];
            lqr.Q[k * augmentedNx * augmentedNx
                + (qp.nx + i) * augmentedNx + qp.nx + i] += differenceWeight;
            lqr.R[k * qp.nu * qp.nu + i * qp.nu + i] =
                p.controlWeights[i] + differenceWeight + 1.0e-9;
            lqr.stateControl[k * augmentedNx * qp.nu
                + (qp.nx + i) * qp.nu + i] = -differenceWeight;
            lqr.r[k * qp.nu + i] =
                -p.controlWeights[i] * p.controlReference[k * qp.nu + i];
        }
    }
    for (int_t i = 0; i < qp.nx; ++i)
    {
        lqr.Qterminal[i * augmentedNx + i] = p.terminalWeights[i];
        lqr.qterminal[i] =
            -p.terminalWeights[i] * p.stateReference[qp.N * qp.nx + i];
    }
    warm.assign(qp.nV, 0.0);
    if (!solveStageVaryingLqr(lqr, solution)) return false;
    for (int_t k = 0; k <= qp.N; ++k)
    {
        std::copy(solution.states.begin() + k * augmentedNx,
            solution.states.begin() + k * augmentedNx + qp.nx,
            warm.begin() + stateIndex(qp, k));
        if (k < qp.N)
            std::copy(solution.controls.begin() + k * qp.nu,
                solution.controls.begin() + (k + 1) * qp.nu,
                warm.begin() + controlIndex(qp, k));
    }
    return true;
}

void initializeRestorationSlacks(
    const AgentQp& qp,
    std::vector<real_t>& warm)
{
    if (qp.slackVariableCount <= 0) return;
    for (int_t variable = qp.trajectoryVariableCount;
         variable < qp.nV; ++variable)
        warm[variable] = 0.0;
    for (int_t row = 0; row < qp.nC; ++row)
    {
        real_t value = 0.0;
        for (int_t variable = 0;
             variable < qp.trajectoryVariableCount; ++variable)
            value += qp.A[row * qp.nV + variable] * warm[variable];
        if (qp.lbA[row] > -0.5 * INFTY && value < qp.lbA[row])
            for (int_t variable = qp.trajectoryVariableCount;
                 variable < qp.nV; ++variable)
            {
                const real_t coefficient = qp.A[row * qp.nV + variable];
                if (coefficient > 0.0)
                    warm[variable] = std::max(
                        warm[variable], (qp.lbA[row] - value) / coefficient);
            }
        if (qp.ubA[row] < 0.5 * INFTY && value > qp.ubA[row])
            for (int_t variable = qp.trajectoryVariableCount;
                 variable < qp.nV; ++variable)
            {
                const real_t coefficient = qp.A[row * qp.nV + variable];
                if (coefficient < 0.0)
                    warm[variable] = std::max(
                        warm[variable], (value - qp.ubA[row]) / -coefficient);
            }
    }
}

bool solveLocalQp(const NonlinearAgentProblem& p, int_t outer, int_t admm,
    const NonlinearTurboOptions& options, bool hessianChanged, bool reliableQp,
    SQProblem*& solver, AgentQp& qp, NonlinearTurboStatistics& stats)
{
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    enum SolvePath { COLD_START, MATRIX_HOTSTART, VECTOR_HOTSTART };
    SolvePath solvePath = COLD_START;
    if (options.continuationMode == NCONT_COLD && solver != 0)
    {
        delete solver;
        solver = 0;
    }
    int_t nWSR = options.maxWorkingSetRecalculations;
    returnValue status; ++stats.qpSolves;
    bool usedRiccatiWarmStart = false;
    initializeRestorationSlacks(qp, qp.solution);
    if (!solver)
    {
        solver = new SQProblem(qp.nV, qp.nC); configureSolver(*solver, reliableQp);
        if (options.useRiccatiWarmStart)
        {
            std::vector<real_t> warm;
            const std::chrono::steady_clock::time_point riccatiStart =
                std::chrono::steady_clock::now();
            const bool warmStartAvailable = riccatiWarmStart(p, qp, warm);
            stats.riccatiTimeMilliseconds += static_cast<real_t>(
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - riccatiStart).count());
            if (warmStartAvailable)
            {
                initializeRestorationSlacks(qp, warm);
                usedRiccatiWarmStart = true;
                ++stats.riccatiInitializations;
                status = solver->init(&qp.H[0], &qp.g[0], &qp.A[0],
                    &qp.lb[0], &qp.ub[0], &qp.lbA[0], &qp.ubA[0],
                    nWSR, 0, &warm[0]);
            }
            else
            {
                ++stats.riccatiFailures;
                status = solver->init(&qp.H[0], &qp.g[0], &qp.A[0],
                    &qp.lb[0], &qp.ub[0], &qp.lbA[0], &qp.ubA[0],
                    nWSR, 0, &qp.solution[0]);
            }
        }
        else
            status = solver->init(&qp.H[0], &qp.g[0], &qp.A[0], &qp.lb[0], &qp.ub[0],
                &qp.lbA[0], &qp.ubA[0], nWSR);
        if (status != SUCCESSFUL_RETURN && usedRiccatiWarmStart)
        {
            ++stats.riccatiFailures;
            delete solver;
            solver = new SQProblem(qp.nV, qp.nC);
            configureSolver(*solver, reliableQp);
            nWSR = options.maxWorkingSetRecalculations;
            status = solver->init(&qp.H[0], &qp.g[0], &qp.A[0],
                &qp.lb[0], &qp.ub[0], &qp.lbA[0], &qp.ubA[0],
                nWSR, 0, &qp.solution[0]);
            ++stats.coldStarts;
        }
        ++stats.coldStarts;
    }
    else if (hessianChanged
        || (admm == 0 && options.continuationMode >= NCONT_QP))
    {
        solvePath = MATRIX_HOTSTART;
        status = solver->hotstart(&qp.H[0], &qp.g[0], &qp.A[0], &qp.lb[0], &qp.ub[0],
            &qp.lbA[0], &qp.ubA[0], nWSR); ++stats.matrixHotstarts;
    }
    else
    {
        solvePath = VECTOR_HOTSTART;
        status = solver->hotstart(&qp.g[0], &qp.lb[0], &qp.ub[0], &qp.lbA[0], &qp.ubA[0], nWSR);
        ++stats.vectorHotstarts;
    }
    stats.qpWorkingSetRecalculations += nWSR;
    stats.backendIterations += nWSR;
    if (status != SUCCESSFUL_RETURN && solvePath != COLD_START)
    {
        ++stats.hotstartFallbacks;
        delete solver; solver = new SQProblem(qp.nV, qp.nC);
        configureSolver(*solver, reliableQp);
        nWSR = options.maxWorkingSetRecalculations;
        status = solver->init(
            &qp.H[0], &qp.g[0], &qp.A[0], &qp.lb[0], &qp.ub[0],
            &qp.lbA[0], &qp.ubA[0], nWSR, 0, &qp.solution[0]
        );
        stats.qpWorkingSetRecalculations += nWSR; ++stats.coldStarts;
        stats.backendIterations += nWSR;
    }
    const bool success = status == SUCCESSFUL_RETURN
        && solver->getPrimalSolution(&qp.solution[0]) == SUCCESSFUL_RETURN;
    const std::chrono::duration<double, std::milli> elapsed =
        std::chrono::steady_clock::now() - start;
    stats.maximumLocalQpSolveTimeMilliseconds = std::max(
        stats.maximumLocalQpSolveTimeMilliseconds,
        static_cast<real_t>(elapsed.count())
    );
    stats.localQpSolveTimeMilliseconds +=
        static_cast<real_t>(elapsed.count());
    if (solvePath == COLD_START)
        stats.coldStartQpTimeMilliseconds += static_cast<real_t>(elapsed.count());
    else if (solvePath == MATRIX_HOTSTART)
        stats.matrixHotstartQpTimeMilliseconds += static_cast<real_t>(elapsed.count());
    else
        stats.vectorHotstartQpTimeMilliseconds += static_cast<real_t>(elapsed.count());
    stats.lastQpStatus = static_cast<int_t>(status);
    return success;
}

void accumulateQpStatistics(
    const NonlinearTurboStatistics& local,
    NonlinearTurboStatistics& total)
{
    total.qpSolves += local.qpSolves;
    total.qpWorkingSetRecalculations += local.qpWorkingSetRecalculations;
    total.backendIterations += local.backendIterations;
    total.lastQpStatus = local.lastQpStatus;
    total.coldStarts += local.coldStarts;
    total.riccatiInitializations += local.riccatiInitializations;
    total.riccatiFailures += local.riccatiFailures;
    total.hotstartFallbacks += local.hotstartFallbacks;
    total.matrixHotstarts += local.matrixHotstarts;
    total.vectorHotstarts += local.vectorHotstarts;
    total.maximumLocalQpSolveTimeMilliseconds = std::max(
        total.maximumLocalQpSolveTimeMilliseconds,
        local.maximumLocalQpSolveTimeMilliseconds
    );
    total.localQpSolveTimeMilliseconds +=
        local.localQpSolveTimeMilliseconds;
    total.riccatiTimeMilliseconds += local.riccatiTimeMilliseconds;
    total.coldStartQpTimeMilliseconds += local.coldStartQpTimeMilliseconds;
    total.matrixHotstartQpTimeMilliseconds +=
        local.matrixHotstartQpTimeMilliseconds;
    total.vectorHotstartQpTimeMilliseconds +=
        local.vectorHotstartQpTimeMilliseconds;
}

bool solveDistributed(const std::vector<NonlinearAgentProblem>& agents, int_t outer,
    const NonlinearTurboOptions& options, bool reliableQp,
    SolverPool& pool, std::vector<PairData>& pairs,
    std::vector<AgentQp>& qps, NonlinearTurboStatistics& stats)
{
    if (outer > 0 && options.continuationMode <= NCONT_INNER_ADMM) pool.reset();
    if (pool.solvers.empty())
    {
        pool.solvers.assign(agents.size(), static_cast<SQProblem*>(0));
        pool.variableCounts.assign(agents.size(), -1);
        pool.constraintCounts.assign(agents.size(), -1);
    }
    for (std::size_t agent = 0; agent < agents.size(); ++agent)
    {
        if (pool.solvers[agent] != 0
            && (pool.variableCounts[agent] != qps[agent].nV
                || pool.constraintCounts[agent] != qps[agent].nC))
        {
            delete pool.solvers[agent];
            pool.solvers[agent] = 0;
        }
        pool.variableCounts[agent] = qps[agent].nV;
        pool.constraintCounts[agent] = qps[agent].nC;
    }
    std::vector<std::vector<std::size_t> > incidentPairs(agents.size());
    for (std::size_t p = 0; p < pairs.size(); ++p)
    {
        incidentPairs[pairs[p].first].push_back(p);
        incidentPairs[pairs[p].second].push_back(p);
    }
    stats.maximumActivePairs = std::max(
        stats.maximumActivePairs, static_cast<int_t>(pairs.size()));
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        stats.maximumAgentDegree = std::max(
            stats.maximumAgentDegree,
            static_cast<int_t>(incidentPairs[a].size()));
        stats.maximumLocalQpVariables = std::max(
            stats.maximumLocalQpVariables, qps[a].nV);
        stats.maximumLocalQpConstraints = std::max(
            stats.maximumLocalQpConstraints, qps[a].nC);
        stats.maximumCorridorRowsPerAgent = std::max(
            stats.maximumCorridorRowsPerAgent, qps[a].corridorRows);
    }

    real_t rho = options.rho;
    stats.minimumAdmmRho = std::min(stats.minimumAdmmRho, rho);
    stats.maximumAdmmRho = std::max(stats.maximumAdmmRho, rho);
    const std::chrono::steady_clock::time_point fixedAssemblyStart =
        std::chrono::steady_clock::now();
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        qps[a].H = qps[a].Hbase;
        qps[a].g = qps[a].gbase;
        addAdmmTerms(static_cast<int_t>(a), pairs, incidentPairs[a],
            rho, true, qps[a]);
    }
    stats.admmAssemblyTimeMilliseconds += static_cast<real_t>(
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - fixedAssemblyStart).count());

    std::vector<int> solved(agents.size(), 0);
    std::vector<NonlinearTurboStatistics> localStatistics(agents.size());
#if defined(_OPENMP)
    const int_t availableThreads = static_cast<int_t>(omp_get_max_threads());
    const int_t agentCount = static_cast<int_t>(agents.size());
    const int_t automaticThreads = std::min<int_t>(
        agentCount, std::max<int_t>(4, (agentCount + 1) / 2));
    const int_t requestedThreads = options.parallelAgentThreads > 0
        ? options.parallelAgentThreads : automaticThreads;
    const int_t agentThreads = std::max<int_t>(
        1, std::min<int_t>(requestedThreads, availableThreads));
#else
    const int_t agentThreads = 1;
#endif
    stats.parallelAgentThreads = std::max(stats.parallelAgentThreads, agentThreads);
    real_t toleranceMultiplier = 1.0;
    if (outer < options.inexactAdmmScpIterations)
        toleranceMultiplier += (options.inexactAdmmToleranceMultiplier - 1.0)
            * static_cast<real_t>(options.inexactAdmmScpIterations - outer)
            / static_cast<real_t>(options.inexactAdmmScpIterations);
    bool admmConverged = false;
    bool hessianChanged = false;
    int_t pendingRhoDirection = 0;
    int_t pendingRhoConfirmations = 0;
    for (int_t iteration = 0; iteration < options.maxAdmmIterations; ++iteration)
    {
        std::fill(solved.begin(), solved.end(), 0);
        for (std::size_t a = 0; a < localStatistics.size(); ++a)
            localStatistics[a] = NonlinearTurboStatistics();
        const std::chrono::steady_clock::time_point assemblyStart =
            std::chrono::steady_clock::now();
        for (std::size_t a = 0; a < agents.size(); ++a)
        {
            qps[a].g = qps[a].gbase;
            addAdmmTerms(static_cast<int_t>(a), pairs, incidentPairs[a],
                rho, false, qps[a]);
        }
        stats.admmAssemblyTimeMilliseconds += static_cast<real_t>(
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - assemblyStart).count());
        const std::chrono::steady_clock::time_point qpBatchStart =
            std::chrono::steady_clock::now();
#if defined(_OPENMP)
        const bool parallelBatch = options.parallelAgentSolves
            && agents.size() > 1 && agentThreads > 1;
        if (parallelBatch) ++stats.parallelQpBatches;
#pragma omp parallel for if(parallelBatch) num_threads(agentThreads)
#endif
        for (int_t a = 0; a < static_cast<int_t>(agents.size()); ++a)
        {
            solved[a] = solveLocalQp(
                agents[a],
                outer,
                iteration,
                options,
                hessianChanged,
                reliableQp,
                pool.solvers[a],
                qps[a],
                localStatistics[a]
            ) ? 1 : 0;
        }
        stats.localQpBatchTimeMilliseconds += static_cast<real_t>(
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - qpBatchStart).count());
        for (std::size_t a = 0; a < agents.size(); ++a)
        {
            accumulateQpStatistics(localStatistics[a], stats);
            if (!solved[a])
            {
                stats.failedAgent = static_cast<int_t>(a);
                return false;
            }
        }
        hessianChanged = false;
        const std::chrono::steady_clock::time_point consensusStart =
            std::chrono::steady_clock::now();
        for (std::size_t p = 0; p < pairs.size(); ++p)
        {
            const int_t np = qps[pairs[p].first].np;
            const int_t sampleCount = static_cast<int_t>(pairs[p].firstPosition.size()) / np;
            for (int_t sample = 0; sample < sampleCount; ++sample)
            {
                positionFromDecisionSample(
                    qps[pairs[p].first], pairs[p].firstC, pairs[p].firstD,
                    sample, pairs[p].samplesPerInterval,
                    qps[pairs[p].first].solution, &pairs[p].firstPosition[sample * np]
                );
                positionFromDecisionSample(
                    qps[pairs[p].second], pairs[p].secondC, pairs[p].secondD,
                    sample, pairs[p].samplesPerInterval,
                    qps[pairs[p].second].solution, &pairs[p].secondPosition[sample * np]
                );
            }
        }
        real_t primalSquared = 0.0, dualSquared = 0.0;
        real_t positionSquared = 0.0, auxiliarySquared = 0.0;
        real_t scaledDualSquared = 0.0;
        int_t residualDimension = 0;
        for (std::size_t p = 0; p < pairs.size(); ++p)
        {
            PairData& pair = pairs[p]; const int_t np = qps[pair.first].np;
            pair.previousFirstAuxiliary = pair.firstAuxiliary;
            pair.previousSecondAuxiliary = pair.secondAuxiliary;
            const int_t sampleCount = static_cast<int_t>(pair.firstPosition.size()) / np;
            for (int_t sample = 0; sample < sampleCount; ++sample)
            {
                real_t signedDistance = 0.0;
                for (int_t i = 0; i < np; ++i)
                {
                    const int_t x = sample * np + i;
                    const real_t relaxedFirst = options.admmRelaxation
                        * pair.firstPosition[x] + (1.0 - options.admmRelaxation)
                            * pair.previousFirstAuxiliary[x];
                    const real_t relaxedSecond = options.admmRelaxation
                        * pair.secondPosition[x] + (1.0 - options.admmRelaxation)
                            * pair.previousSecondAuxiliary[x];
                    signedDistance += pair.normal[x]
                        * ((relaxedFirst + pair.firstDual[x])
                            - (relaxedSecond + pair.secondDual[x]));
                }
                const real_t correction = 0.5 * std::max(
                    0.0, pair.safetyDistance - signedDistance);
                for (int_t i = 0; i < np; ++i)
                {
                    const int_t x = sample * np + i;
                    const real_t relaxedFirst = options.admmRelaxation
                        * pair.firstPosition[x] + (1.0 - options.admmRelaxation)
                            * pair.previousFirstAuxiliary[x];
                    const real_t relaxedSecond = options.admmRelaxation
                        * pair.secondPosition[x] + (1.0 - options.admmRelaxation)
                            * pair.previousSecondAuxiliary[x];
                    pair.firstAuxiliary[x] = relaxedFirst
                        + pair.firstDual[x] + correction * pair.normal[x];
                    pair.secondAuxiliary[x] = relaxedSecond
                        + pair.secondDual[x] - correction * pair.normal[x];
                    const real_t e1 = pair.firstPosition[x] - pair.firstAuxiliary[x];
                    const real_t e2 = pair.secondPosition[x] - pair.secondAuxiliary[x];
                    pair.firstDual[x] += relaxedFirst - pair.firstAuxiliary[x];
                    pair.secondDual[x] += relaxedSecond - pair.secondAuxiliary[x];
                    const real_t c1 = pair.firstAuxiliary[x]
                        - pair.previousFirstAuxiliary[x];
                    const real_t c2 = pair.secondAuxiliary[x]
                        - pair.previousSecondAuxiliary[x];
                    primalSquared += e1 * e1 + e2 * e2;
                    dualSquared += rho * rho
                        * (c1 * c1 + c2 * c2);
                    const real_t positionMidpoint = 0.5
                        * (pair.firstPosition[x] + pair.secondPosition[x]);
                    const real_t auxiliaryMidpoint = 0.5
                        * (pair.firstAuxiliary[x] + pair.secondAuxiliary[x]);
                    const real_t centeredFirstPosition =
                        pair.firstPosition[x] - positionMidpoint;
                    const real_t centeredSecondPosition =
                        pair.secondPosition[x] - positionMidpoint;
                    const real_t centeredFirstAuxiliary =
                        pair.firstAuxiliary[x] - auxiliaryMidpoint;
                    const real_t centeredSecondAuxiliary =
                        pair.secondAuxiliary[x] - auxiliaryMidpoint;
                    positionSquared += centeredFirstPosition * centeredFirstPosition
                        + centeredSecondPosition * centeredSecondPosition;
                    auxiliarySquared += centeredFirstAuxiliary * centeredFirstAuxiliary
                        + centeredSecondAuxiliary * centeredSecondAuxiliary;
                    scaledDualSquared += rho * rho
                        * (pair.firstDual[x] * pair.firstDual[x]
                            + pair.secondDual[x] * pair.secondDual[x]);
                    residualDimension += 2;
                }
            }
        }
        const real_t primal = std::sqrt(primalSquared);
        const real_t dualResidual = std::sqrt(dualSquared);
        const real_t dimensionScale = std::sqrt(
            static_cast<real_t>(residualDimension));
        const real_t primalThreshold = toleranceMultiplier * (dimensionScale
            * options.admmPrimalTolerance + options.admmRelativeTolerance
                * std::max(std::sqrt(positionSquared), std::sqrt(auxiliarySquared)));
        const real_t dualThreshold = toleranceMultiplier * (dimensionScale
            * options.admmDualTolerance + options.admmRelativeTolerance
                * std::sqrt(scaledDualSquared));
        stats.primalResidual = primal;
        stats.dualResidual = dualResidual;
        stats.primalStoppingThreshold = primalThreshold;
        stats.dualStoppingThreshold = dualThreshold;
        ++stats.admmIterations;
        stats.consensusTimeMilliseconds += static_cast<real_t>(
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - consensusStart).count());
        if (iteration + 1 >= options.minimumAdmmIterations
            && primal <= primalThreshold && dualResidual <= dualThreshold)
        {
            admmConverged = true;
            break;
        }
        if (options.adaptiveRho
            && (iteration + 1) % options.adaptiveRhoInterval == 0
            && iteration + 1 < options.maxAdmmIterations)
        {
            const real_t normalizedPrimal = primal
                / std::max(primalThreshold, static_cast<real_t>(1.0e-16));
            const real_t normalizedDual = dualResidual
                / std::max(dualThreshold, static_cast<real_t>(1.0e-16));
            int_t rhoDirection = 0;
            if (normalizedPrimal
                    > options.adaptiveRhoImbalance * normalizedDual
                && primal > primalThreshold)
                rhoDirection = 1;
            else if (normalizedDual
                    > options.adaptiveRhoImbalance * normalizedPrimal
                && dualResidual > dualThreshold)
                rhoDirection = -1;
            if (rhoDirection == 0)
            {
                pendingRhoDirection = 0;
                pendingRhoConfirmations = 0;
            }
            else if (rhoDirection == pendingRhoDirection)
                ++pendingRhoConfirmations;
            else
            {
                pendingRhoDirection = rhoDirection;
                pendingRhoConfirmations = 1;
            }
            real_t updatedRho = rho;
            if (pendingRhoConfirmations >= 2 && rhoDirection > 0)
                updatedRho = std::min(options.maximumRho,
                    rho * options.adaptiveRhoScale);
            else if (pendingRhoConfirmations >= 2 && rhoDirection < 0)
                updatedRho = std::max(options.minimumRho,
                    rho / options.adaptiveRhoScale);
            if (std::fabs(updatedRho - rho) > 1.0e-12 * rho)
            {
                const real_t dualScale = rho / updatedRho;
                for (std::size_t p = 0; p < pairs.size(); ++p)
                {
                    for (std::size_t i = 0; i < pairs[p].firstDual.size(); ++i)
                        pairs[p].firstDual[i] *= dualScale;
                    for (std::size_t i = 0; i < pairs[p].secondDual.size(); ++i)
                        pairs[p].secondDual[i] *= dualScale;
                }
                rho = updatedRho;
                pendingRhoDirection = 0;
                pendingRhoConfirmations = 0;
                stats.minimumAdmmRho = std::min(stats.minimumAdmmRho, rho);
                stats.maximumAdmmRho = std::max(stats.maximumAdmmRho, rho);
                ++stats.rhoUpdates;
                const std::chrono::steady_clock::time_point rebuildStart =
                    std::chrono::steady_clock::now();
                for (std::size_t a = 0; a < agents.size(); ++a)
                {
                    qps[a].H = qps[a].Hbase;
                    qps[a].g = qps[a].gbase;
                    addAdmmTerms(static_cast<int_t>(a), pairs, incidentPairs[a],
                        rho, true, qps[a]);
                }
                stats.admmAssemblyTimeMilliseconds += static_cast<real_t>(
                    std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - rebuildStart).count());
                hessianChanged = true;
            }
        }
    }
    stats.finalAdmmRho = rho;
    if (std::fabs(rho - options.rho) > 1.0e-12 * options.rho)
    {
        const real_t continuationScale = rho / options.rho;
        for (std::size_t p = 0; p < pairs.size(); ++p)
        {
            for (std::size_t i = 0; i < pairs[p].firstDual.size(); ++i)
                pairs[p].firstDual[i] *= continuationScale;
            for (std::size_t i = 0; i < pairs[p].secondDual.size(); ++i)
                pairs[p].secondDual[i] *= continuationScale;
        }
    }
    if (admmConverged) ++stats.admmConvergedSubproblems;
    else ++stats.admmIterationLimitSubproblems;

    return true;
}

#ifdef QPOASES_WITH_OSQP
#ifdef QPOASES_OSQP_V1
typedef OSQPFloat OsqpFloat;
typedef OSQPInt OsqpInt;
typedef OSQPCscMatrix OsqpCscMatrix;
#else
typedef c_float OsqpFloat;
typedef c_int OsqpInt;
typedef csc OsqpCscMatrix;
#endif
struct OsqpCscStorage
{
    std::vector<OsqpFloat> values;
    std::vector<OsqpInt> rowIndices;
    std::vector<OsqpInt> columnPointers;
    OsqpCscMatrix matrix;
};

void makeOsqpHessian(
    const std::vector<real_t>& dense,
    int_t dimension,
    OsqpCscStorage& sparse
)
{
    sparse.columnPointers.resize(dimension + 1);
    for (int_t column = 0; column < dimension; ++column)
    {
        sparse.columnPointers[column] = static_cast<OsqpInt>(sparse.values.size());
        for (int_t row = 0; row <= column; ++row)
        {
            const real_t value = dense[row * dimension + column];
            if (std::fabs(value) <= 1.0e-14) continue;
            sparse.rowIndices.push_back(static_cast<OsqpInt>(row));
            sparse.values.push_back(static_cast<OsqpFloat>(value));
        }
    }
    sparse.columnPointers[dimension] = static_cast<OsqpInt>(sparse.values.size());
    sparse.matrix.nzmax = static_cast<OsqpInt>(sparse.values.size());
    sparse.matrix.m = static_cast<OsqpInt>(dimension);
    sparse.matrix.n = static_cast<OsqpInt>(dimension);
    sparse.matrix.p = &sparse.columnPointers[0];
    sparse.matrix.i = sparse.rowIndices.empty() ? 0 : &sparse.rowIndices[0];
    sparse.matrix.x = sparse.values.empty() ? 0 : &sparse.values[0];
    sparse.matrix.nz = -1;
#ifdef QPOASES_OSQP_V1
    sparse.matrix.owned = 0;
#endif
}

void makeOsqpConstraints(
    const std::vector<real_t>& dense,
    int_t rows,
    int_t columns,
    OsqpCscStorage& sparse
)
{
    sparse.columnPointers.resize(columns + 1);
    for (int_t column = 0; column < columns; ++column)
    {
        sparse.columnPointers[column] = static_cast<OsqpInt>(sparse.values.size());
        sparse.rowIndices.push_back(static_cast<OsqpInt>(column));
        sparse.values.push_back(1.0);
        for (int_t row = 0; row < rows; ++row)
        {
            const real_t value = dense[row * columns + column];
            if (std::fabs(value) <= 1.0e-14) continue;
            sparse.rowIndices.push_back(static_cast<OsqpInt>(columns + row));
            sparse.values.push_back(static_cast<OsqpFloat>(value));
        }
    }
    sparse.columnPointers[columns] = static_cast<OsqpInt>(sparse.values.size());
    sparse.matrix.nzmax = static_cast<OsqpInt>(sparse.values.size());
    sparse.matrix.m = static_cast<OsqpInt>(columns + rows);
    sparse.matrix.n = static_cast<OsqpInt>(columns);
    sparse.matrix.p = &sparse.columnPointers[0];
    sparse.matrix.i = sparse.rowIndices.empty() ? 0 : &sparse.rowIndices[0];
    sparse.matrix.x = sparse.values.empty() ? 0 : &sparse.values[0];
    sparse.matrix.nz = -1;
#ifdef QPOASES_OSQP_V1
    sparse.matrix.owned = 0;
#endif
}

bool solveCentralizedOsqp(
    const std::vector<real_t>& H, const std::vector<real_t>& g,
    const std::vector<real_t>& A, const std::vector<real_t>& lb,
    const std::vector<real_t>& ub, const std::vector<real_t>& lbA,
    const std::vector<real_t>& ubA, CentralContext& context,
    std::vector<real_t>& decision, NonlinearTurboStatistics& stats)
{
    const int_t nV = static_cast<int_t>(g.size());
    const int_t nC = static_cast<int_t>(lbA.size());
    OsqpCscStorage P, constraints;
    makeOsqpHessian(H, nV, P);
    makeOsqpConstraints(A, nC, nV, constraints);
    std::vector<OsqpFloat> q(g.begin(), g.end());
    std::vector<OsqpFloat> lower(nV + nC), upper(nV + nC);
    for (int_t i = 0; i < nV; ++i)
    {
        lower[i] = static_cast<OsqpFloat>(lb[i]);
        upper[i] = static_cast<OsqpFloat>(ub[i]);
    }
    for (int_t i = 0; i < nC; ++i)
    {
        lower[nV + i] = static_cast<OsqpFloat>(lbA[i]);
        upper[nV + i] = static_cast<OsqpFloat>(ubA[i]);
    }
    ++stats.qpSolves; ++stats.coldStarts;
#ifdef QPOASES_OSQP_V1
    OSQPSettings settings; osqp_set_default_settings(&settings);
    settings.verbose = 0; settings.polishing = 1; settings.max_iter = 10000;
    settings.eps_abs = 1.0e-5; settings.eps_rel = 1.0e-5;
    OSQPSolver* solver = 0;
    if (osqp_setup(
            &solver, &P.matrix, &q[0], &constraints.matrix,
            &lower[0], &upper[0], static_cast<OSQPInt>(nV + nC),
            static_cast<OSQPInt>(nV), &settings) != 0 || solver == 0) return false;
    if (context.osqpWarmStart.size() == static_cast<std::size_t>(nV))
    {
        std::vector<OsqpFloat> warm(
            context.osqpWarmStart.begin(), context.osqpWarmStart.end());
        osqp_warm_start(solver, &warm[0], 0); ++stats.vectorHotstarts;
    }
    osqp_solve(solver);
    stats.backendIterations += static_cast<int_t>(solver->info->iter);
    const OSQPInt status = solver->info->status_val;
    stats.lastQpStatus = static_cast<int_t>(status);
    const bool solved = status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE;
    if (solved)
    {
        decision.resize(nV);
        for (int_t i = 0; i < nV; ++i) decision[i] = solver->solution->x[i];
        context.osqpWarmStart = decision;
    }
    osqp_cleanup(solver);
#else
    OSQPData data;
    data.n = static_cast<c_int>(nV); data.m = static_cast<c_int>(nV + nC);
    data.P = &P.matrix; data.A = &constraints.matrix;
    data.q = &q[0]; data.l = &lower[0]; data.u = &upper[0];
    OSQPSettings settings; osqp_set_default_settings(&settings);
    settings.verbose = 0; settings.polish = 1; settings.max_iter = 10000;
    settings.eps_abs = 1.0e-5; settings.eps_rel = 1.0e-5;
    OSQPWorkspace* workspace = 0;
    if (osqp_setup(&workspace, &data, &settings) != 0 || workspace == 0) return false;
    if (context.osqpWarmStart.size() == static_cast<std::size_t>(nV))
    {
        std::vector<OsqpFloat> warm(context.osqpWarmStart.begin(), context.osqpWarmStart.end());
        osqp_warm_start_x(workspace, &warm[0]); ++stats.vectorHotstarts;
    }
    osqp_solve(workspace);
    stats.backendIterations += static_cast<int_t>(workspace->info->iter);
    const c_int status = workspace->info->status_val;
    stats.lastQpStatus = static_cast<int_t>(status);
    const bool solved = status == OSQP_SOLVED || status == OSQP_SOLVED_INACCURATE;
    if (solved)
    {
        decision.resize(nV);
        for (int_t i = 0; i < nV; ++i) decision[i] = workspace->solution->x[i];
        context.osqpWarmStart = decision;
    }
    osqp_cleanup(workspace);
#endif
    return solved;
}
#endif

bool solveCentralized(const std::vector<PairData>& pairs, int_t outer,
    const NonlinearTurboOptions& options, bool reliableQp,
    CentralContext& context, std::vector<AgentQp>& qps, NonlinearTurboStatistics& stats)
{
    std::vector<int_t> offsets(qps.size(), 0);
    int_t nV = 0, dynamicsRows = 0;
    for (std::size_t a = 0; a < qps.size(); ++a)
    {
        offsets[a] = nV; nV += qps[a].nV; dynamicsRows += qps[a].nC;
    }
    const int_t collisionRowsPerPair = qps[0].N
        * options.collisionSamplesPerInterval;
    const int_t nC = dynamicsRows + static_cast<int_t>(pairs.size()) * collisionRowsPerPair;
    stats.centralizedQpVariables = std::max(stats.centralizedQpVariables, nV);
    stats.centralizedQpConstraints = std::max(stats.centralizedQpConstraints, nC);
    std::vector<real_t> H(nV * nV, 0.0), g(nV, 0.0), A(nC * nV, 0.0);
    std::vector<real_t> lb(nV, -INFTY), ub(nV, INFTY), lbA(nC, -INFTY), ubA(nC, INFTY);
    int_t rowOffset = 0;
    for (std::size_t a = 0; a < qps.size(); ++a)
    {
        const AgentQp& qp = qps[a]; const int_t offset = offsets[a];
        for (int_t i = 0; i < qp.nV; ++i)
        {
            g[offset + i] = qp.gbase[i]; lb[offset + i] = qp.lb[i]; ub[offset + i] = qp.ub[i];
            for (int_t j = 0; j < qp.nV; ++j)
                H[(offset + i) * nV + offset + j] = qp.Hbase[i * qp.nV + j];
        }
        for (int_t i = 0; i < qp.nC; ++i)
        {
            lbA[rowOffset + i] = qp.lbA[i]; ubA[rowOffset + i] = qp.ubA[i];
            for (int_t j = 0; j < qp.nV; ++j)
                A[(rowOffset + i) * nV + offset + j] = qp.A[i * qp.nV + j];
        }
        rowOffset += qp.nC;
    }
    for (std::size_t pairIndex = 0; pairIndex < pairs.size(); ++pairIndex)
    {
        const PairData& pair = pairs[pairIndex];
        const AgentQp& first = qps[pair.first]; const AgentQp& second = qps[pair.second];
        for (int_t sample = 1; sample <= collisionRowsPerPair; ++sample)
        {
            const real_t* normal = &pair.normal[sample * first.np];
            int_t firstStage = 0, secondStage = 0;
            real_t alpha = 0.0;
            sampleStages(
                sample, pair.samplesPerInterval, first.N,
                firstStage, secondStage, alpha
            );
            const int_t stages[2] = {firstStage, secondStage};
            const real_t weights[2] = {1.0 - alpha, alpha};
            real_t affine = 0.0;
            for (int_t endpoint = 0; endpoint < 2; ++endpoint)
            {
                if (weights[endpoint] <= 0.0) continue;
                const int_t stage = stages[endpoint];
                const real_t* firstC =
                    &pair.firstC[stage * first.np * first.nx];
                const real_t* secondC =
                    &pair.secondC[stage * second.np * second.nx];
                const real_t* firstD =
                    &pair.firstD[stage * first.np];
                const real_t* secondD =
                    &pair.secondD[stage * second.np];
                const int_t firstState = offsets[pair.first] + stateIndex(first, stage);
                const int_t secondState = offsets[pair.second] + stateIndex(second, stage);
                for (int_t j = 0; j < first.nx; ++j)
                    for (int_t i = 0; i < first.np; ++i)
                        A[rowOffset * nV + firstState + j]
                            += weights[endpoint] * normal[i] * firstC[i * first.nx + j];
                for (int_t j = 0; j < second.nx; ++j)
                    for (int_t i = 0; i < second.np; ++i)
                        A[rowOffset * nV + secondState + j]
                            -= weights[endpoint] * normal[i] * secondC[i * second.nx + j];
                for (int_t i = 0; i < first.np; ++i)
                    affine += weights[endpoint] * normal[i] * (firstD[i] - secondD[i]);
            }
            lbA[rowOffset] = pair.safetyDistance - affine; ubA[rowOffset] = INFTY; ++rowOffset;
        }
    }

    if (context.solver != 0
        && (context.g.size() != static_cast<std::size_t>(nV)
            || context.lbA.size() != static_cast<std::size_t>(nC)))
    {
        delete context.solver;
        context.solver = 0;
    }

    if (options.coordinationMethod == NCM_CENTRALIZED_OSQP)
    {
#ifdef QPOASES_WITH_OSQP
        std::vector<real_t> decision;
        if (!solveCentralizedOsqp(
                H, g, A, lb, ub, lbA, ubA, context, decision, stats)) return false;
        for (std::size_t a = 0; a < qps.size(); ++a)
            std::copy(
                decision.begin() + offsets[a],
                decision.begin() + offsets[a] + qps[a].nV,
                qps[a].solution.begin()
            );
        context.H.swap(H); context.g.swap(g); context.A.swap(A); context.lb.swap(lb);
        context.ub.swap(ub); context.lbA.swap(lbA); context.ubA.swap(ubA);
        return true;
#else
        return false;
#endif
    }

    int_t nWSR = options.maxWorkingSetRecalculations; returnValue status; ++stats.qpSolves;
    if (!context.solver)
    {
        context.solver = new SQProblem(nV, nC);
        configureSolver(*context.solver, reliableQp);
        status = context.solver->init(&H[0], &g[0], &A[0], &lb[0], &ub[0], &lbA[0], &ubA[0],
                                      nWSR); ++stats.coldStarts;
    }
    else
    {
        status = context.solver->hotstart(&H[0], &g[0], &A[0], &lb[0], &ub[0], &lbA[0], &ubA[0], nWSR);
        ++stats.matrixHotstarts;
    }
    stats.qpWorkingSetRecalculations += nWSR;
    stats.backendIterations += nWSR;
    stats.lastQpStatus = static_cast<int_t>(status);
    if (status != SUCCESSFUL_RETURN && outer > 0)
    {
        delete context.solver; context.solver = new SQProblem(nV, nC);
        configureSolver(*context.solver, reliableQp);
        nWSR = options.maxWorkingSetRecalculations;
        status = context.solver->init(&H[0], &g[0], &A[0], &lb[0], &ub[0], &lbA[0], &ubA[0], nWSR);
        stats.qpWorkingSetRecalculations += nWSR; ++stats.coldStarts;
        stats.backendIterations += nWSR;
        stats.lastQpStatus = static_cast<int_t>(status);
    }
    if (status != SUCCESSFUL_RETURN) return false;
    std::vector<real_t> decision(nV, 0.0);
    if (context.solver->getPrimalSolution(&decision[0]) != SUCCESSFUL_RETURN) return false;
    for (std::size_t a = 0; a < qps.size(); ++a)
        std::copy(decision.begin() + offsets[a], decision.begin() + offsets[a] + qps[a].nV,
                  qps[a].solution.begin());
    context.H.swap(H); context.g.swap(g); context.A.swap(A); context.lb.swap(lb);
    context.ub.swap(ub); context.lbA.swap(lbA); context.ubA.swap(ubA);
    return true;
}

real_t trajectoryCost(const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<std::vector<real_t> >& states,
    const std::vector<std::vector<real_t> >& controls)
{
    real_t cost = 0.0;
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        const int_t nx = agents[a].model->stateDimension();
        const int_t nu = agents[a].model->controlDimension();
        for (int_t k = 0; k <= agents[a].horizon; ++k)
        {
            for (int_t i = 0; i < nx; ++i)
            {
                const real_t error = states[a][k * nx + i] - agents[a].stateReference[k * nx + i];
                const real_t weight = k == agents[a].horizon ? agents[a].terminalWeights[i] : agents[a].stateWeights[i];
                cost += 0.5 * weight * error * error;
            }
            if (k < agents[a].horizon) for (int_t i = 0; i < nu; ++i)
            {
                const real_t error = controls[a][k * nu + i] - agents[a].controlReference[k * nu + i];
                cost += 0.5 * agents[a].controlWeights[i] * error * error;
            }
        }
        for (int_t k = 0; k + 1 < agents[a].horizon; ++k)
            for (int_t i = 0; i < nu; ++i)
            {
                const real_t weight = agents[a].controlDifferenceWeights.empty()
                    ? 0.0 : agents[a].controlDifferenceWeights[i];
                const real_t difference = controls[a][(k + 1) * nu + i]
                    - controls[a][k * nu + i];
                cost += 0.5 * weight * difference * difference;
            }
    }
    return cost;
}

real_t minimumDistance(const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<std::vector<real_t> >& states)
{
    if (agents.size() < 2) return INFTY;
    real_t minimum = INFTY;
    const int_t np = worldPositionDimension(agents);
    for (std::size_t first = 0; first < agents.size(); ++first)
    for (std::size_t second = first + 1; second < agents.size(); ++second)
    {
        const int_t nxf = agents[first].model->stateDimension();
        const int_t nxs = agents[second].model->stateDimension();
        std::vector<real_t> pf(np, 0.0), ps(np, 0.0);
        for (int_t k = 0; k <= agents[first].horizon; ++k)
        {
            worldPosition(
                *agents[first].model,
                &states[first][k * nxf],
                np,
                &pf[0]
            );
            worldPosition(
                *agents[second].model,
                &states[second][k * nxs],
                np,
                &ps[0]
            );
            real_t squared = 0.0;
            for (int_t i = 0; i < np; ++i) squared += (pf[i] - ps[i]) * (pf[i] - ps[i]);
            minimum = std::min(minimum, std::sqrt(squared));
        }
    }
    return minimum;
}

real_t minimumPairwiseClearance(
    const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<std::vector<real_t> >& states,
    const NonlinearTurboOptions& options
)
{
    if (agents.size() < 2) return INFTY;
    real_t minimum = INFTY;
    const int_t np = worldPositionDimension(agents);
    for (std::size_t first = 0; first < agents.size(); ++first)
    for (std::size_t second = first + 1; second < agents.size(); ++second)
    for (int_t firstCircle = 0;
         firstCircle < collisionCircleCount(agents[first]);
         ++firstCircle)
    for (int_t secondCircle = 0;
         secondCircle < collisionCircleCount(agents[second]);
         ++secondCircle)
    {
        const CollisionCircle firstGeometry = collisionCircleForAgent(
            agents[first], firstCircle, options);
        const CollisionCircle secondGeometry = collisionCircleForAgent(
            agents[second], secondCircle, options);
        const real_t requiredDistance = firstGeometry.radius
            + secondGeometry.radius + endpointFeasiblePairBuffer(
                agents[first], firstCircle, agents[second], secondCircle, options);
        const int_t nxf = agents[first].model->stateDimension();
        const int_t nxs = agents[second].model->stateDimension();
        std::vector<real_t> firstBegin(np, 0.0), firstEnd(np, 0.0);
        std::vector<real_t> secondBegin(np, 0.0), secondEnd(np, 0.0);
        for (int_t k = 0; k < agents[first].horizon; ++k)
        {
            worldCollisionPosition(
                agents[first], firstCircle,
                &states[first][k * nxf], np, &firstBegin[0]);
            worldCollisionPosition(
                agents[first], firstCircle,
                &states[first][(k + 1) * nxf], np, &firstEnd[0]);
            worldCollisionPosition(
                agents[second], secondCircle,
                &states[second][k * nxs], np, &secondBegin[0]);
            worldCollisionPosition(
                agents[second], secondCircle,
                &states[second][(k + 1) * nxs], np, &secondEnd[0]);
            for (int_t sample = 0;
                 sample <= options.collisionSamplesPerInterval;
                 ++sample)
            {
                const real_t alpha = static_cast<real_t>(sample)
                    / options.collisionSamplesPerInterval;
                real_t squared = 0.0;
                for (int_t i = 0; i < np; ++i)
                {
                    const real_t firstPosition = firstBegin[i]
                        + alpha * (firstEnd[i] - firstBegin[i]);
                    const real_t secondPosition = secondBegin[i]
                        + alpha * (secondEnd[i] - secondBegin[i]);
                    const real_t difference = firstPosition - secondPosition;
                    squared += difference * difference;
                }
                minimum = std::min(
                    minimum,
                    std::sqrt(squared) - requiredDistance
                );
            }
        }
    }
    return minimum;
}
real_t minimumObstacleDistance(
    const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<std::vector<real_t> >& states,
    const std::vector<ConvexPolygonObstacle>& obstacles
)
{
    if (obstacles.empty()) return INFTY;
    real_t minimum = INFTY;
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        const int_t nx = agents[a].model->stateDimension();
        const int_t np = agents[a].model->positionDimension();
        std::vector<real_t> position(np, 0.0);
        for (int_t k = 0; k <= agents[a].horizon; ++k)
        {
            agents[a].model->position(
                &states[a][k * nx],
                &position[0]
            );
            for (std::size_t obstacleIndex = 0;
                 obstacleIndex < obstacles.size();
                 ++obstacleIndex)
                minimum = std::min(
                    minimum,
                    projectToPolygon(
                        position[0],
                        position[1],
                        obstacles[obstacleIndex]
                    ).signedDistance
                );
        }
    }
    return minimum;
}

real_t minimumObstacleClearance(
    const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<std::vector<real_t> >& states,
    const std::vector<ConvexPolygonObstacle>& obstacles,
    const NonlinearTurboOptions& options
)
{
    if (obstacles.empty()) return INFTY;
    real_t minimum = INFTY;
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        const int_t np = agents[a].model->positionDimension();
        for (int_t circle = 0;
             circle < collisionCircleCount(agents[a]);
             ++circle)
        {
            const CollisionCircle geometry = collisionCircleForAgent(
                agents[a], circle, options);
            const real_t requiredDistance = obstacleDistanceForCircle(
                agents[a], geometry, options);
            const int_t nx = agents[a].model->stateDimension();
            std::vector<real_t> begin(np, 0.0), end(np, 0.0);
            for (int_t k = 0; k < agents[a].horizon; ++k)
            {
                worldCollisionPosition(
                    agents[a], circle, &states[a][k * nx], np, &begin[0]);
                worldCollisionPosition(
                    agents[a], circle, &states[a][(k + 1) * nx], np, &end[0]);
                for (int_t sample = 0;
                     sample <= options.collisionSamplesPerInterval;
                     ++sample)
                {
                    const real_t alpha = static_cast<real_t>(sample)
                        / options.collisionSamplesPerInterval;
                    const real_t x = begin[0] + alpha * (end[0] - begin[0]);
                    const real_t y = begin[1] + alpha * (end[1] - begin[1]);
                    for (std::size_t obstacleIndex = 0;
                         obstacleIndex < obstacles.size();
                         ++obstacleIndex)
                        minimum = std::min(
                            minimum,
                            projectToPolygon(
                                x, y, obstacles[obstacleIndex]
                            ).signedDistance - requiredDistance
                        );
                }
            }
        }
    }
    return minimum;
}

bool validateEndpointGeometry(
    const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<ConvexPolygonObstacle>& obstacles,
    const NonlinearTurboOptions& options,
    std::string& error
)
{
    const int_t worldDimension = worldPositionDimension(agents);
    for (int_t endpoint = 0; endpoint < 2; ++endpoint)
    {
        for (std::size_t a = 0; a < agents.size(); ++a)
        {
            const NonlinearAgentProblem& agent = agents[a];
            const int_t nx = agent.model->stateDimension();
            const real_t* state = endpoint == 0
                ? &agent.initialState[0]
                : &agent.stateReference[agent.horizon * nx];
            const int_t np = agent.model->positionDimension();
            for (int_t circle = 0;
                 circle < collisionCircleCount(agent);
                 ++circle)
            {
                const CollisionCircle geometry = collisionCircleForAgent(
                    agent, circle, options);
                const real_t requiredDistance = obstacleDistanceForCircle(
                    agent, geometry, options);
                std::vector<real_t> position(np, 0.0);
                worldCollisionPosition(
                    agent, circle, state, np, &position[0]);
                for (std::size_t obstacle = 0;
                     obstacle < obstacles.size();
                     ++obstacle)
                {
                    const real_t margin = projectToPolygon(
                        position[0],
                        position[1],
                        obstacles[obstacle]
                    ).signedDistance - requiredDistance;
                    if (margin < -options.collisionTolerance)
                    {
                        std::ostringstream message;
                        message << "agent " << a << " "
                            << (endpoint == 0 ? "initial state" : "goal reference")
                            << " violates obstacle " << obstacle
                            << " clearance at stage "
                            << (endpoint == 0 ? 0 : agent.horizon);
                        error = message.str();
                        return false;
                    }
                }
            }
        }

        for (std::size_t first = 0; first < agents.size(); ++first)
        for (std::size_t second = first + 1; second < agents.size(); ++second)
        for (int_t firstCircle = 0;
             firstCircle < collisionCircleCount(agents[first]);
             ++firstCircle)
        for (int_t secondCircle = 0;
             secondCircle < collisionCircleCount(agents[second]);
             ++secondCircle)
        {
            const NonlinearAgentProblem& firstAgent = agents[first];
            const NonlinearAgentProblem& secondAgent = agents[second];
            const int_t firstNx = firstAgent.model->stateDimension();
            const int_t secondNx = secondAgent.model->stateDimension();
            const real_t* firstState = endpoint == 0
                ? &firstAgent.initialState[0]
                : &firstAgent.stateReference[firstAgent.horizon * firstNx];
            const real_t* secondState = endpoint == 0
                ? &secondAgent.initialState[0]
                : &secondAgent.stateReference[secondAgent.horizon * secondNx];
            std::vector<real_t> firstPosition(worldDimension, 0.0);
            std::vector<real_t> secondPosition(worldDimension, 0.0);
            worldCollisionPosition(
                firstAgent, firstCircle, firstState,
                worldDimension, &firstPosition[0]);
            worldCollisionPosition(
                secondAgent, secondCircle, secondState,
                worldDimension, &secondPosition[0]);
            real_t squaredDistance = 0.0;
            for (int_t i = 0; i < worldDimension; ++i)
                squaredDistance +=
                    (firstPosition[i] - secondPosition[i])
                    * (firstPosition[i] - secondPosition[i]);
            const CollisionCircle firstGeometry = collisionCircleForAgent(
                firstAgent, firstCircle, options);
            const CollisionCircle secondGeometry = collisionCircleForAgent(
                secondAgent, secondCircle, options);
            const real_t requiredDistance = firstGeometry.radius
                + secondGeometry.radius + endpointFeasiblePairBuffer(
                    firstAgent, firstCircle, secondAgent, secondCircle, options);
            const real_t margin = std::sqrt(squaredDistance)
                - requiredDistance;
            if (margin < -options.collisionTolerance)
            {
                std::ostringstream message;
                message << "agents " << first << " and " << second << " "
                    << (endpoint == 0 ? "initial states" : "goal references")
                    << " violate pairwise clearance at stage "
                    << (endpoint == 0 ? 0 : firstAgent.horizon);
                error = message.str();
                return false;
            }
        }
    }
    return true;
}

std::string geometryDiagnostic(
    const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<std::vector<real_t> >& states,
    const std::vector<ConvexPolygonObstacle>& obstacles,
    const NonlinearTurboOptions& options
)
{
    bool hasBodyFootprint = false;
    for (std::size_t agent = 0; agent < agents.size(); ++agent)
        hasBodyFootprint = hasBodyFootprint
            || !agents[agent].collisionCircles.empty();
    if (hasBodyFootprint)
    {
        const real_t pairMargin =
            minimumPairwiseClearance(agents, states, options);
        const real_t obstacleMargin =
            minimumObstacleClearance(agents, states, obstacles, options);
        if (std::min(pairMargin, obstacleMargin)
            >= -options.collisionTolerance)
            return "nominal geometry is feasible; inspect dynamics and bounds";
        if (pairMargin <= obstacleMargin)
            return "nominal body footprints violate pairwise clearance";
        return "nominal body footprint violates obstacle clearance";
    }

    real_t worstMargin = INFTY;
    bool worstIsPair = false;
    std::size_t worstFirst = 0, worstSecond = 0;
    int_t worstStage = 0;
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        const int_t nx = agents[a].model->stateDimension();
        const int_t np = agents[a].model->positionDimension();
        const real_t requiredDistance = obstacleDistanceForAgent(
            agents[a],
            options
        );
        std::vector<real_t> position(np, 0.0);
        for (int_t k = 0; k <= agents[a].horizon; ++k)
        {
            agents[a].model->position(&states[a][k * nx], &position[0]);
            real_t firstViolation = INFTY, secondViolation = INFTY;
            std::size_t firstObstacle = 0, secondObstacle = 0;
            for (std::size_t obstacle = 0;
                 obstacle < obstacles.size();
                 ++obstacle)
            {
                const real_t margin = projectToPolygon(
                    position[0], position[1], obstacles[obstacle]
                ).signedDistance - requiredDistance;
                if (margin < firstViolation)
                {
                    secondViolation = firstViolation;
                    secondObstacle = firstObstacle;
                    firstViolation = margin;
                    firstObstacle = obstacle;
                }
                else if (margin < secondViolation)
                {
                    secondViolation = margin;
                    secondObstacle = obstacle;
                }
                if (margin < worstMargin)
                {
                    worstMargin = margin;
                    worstIsPair = false;
                    worstFirst = a;
                    worstSecond = obstacle;
                    worstStage = k;
                }
            }
            if (secondViolation < -options.collisionTolerance)
            {
                std::ostringstream message;
                message << "agent " << a
                    << " overlaps clearance regions of obstacles "
                    << firstObstacle << " and " << secondObstacle
                    << " at stage " << k
                    << "; passage may be too narrow";
                return message.str();
            }
        }
    }

    const int_t worldDimension = worldPositionDimension(agents);
    for (std::size_t first = 0; first < agents.size(); ++first)
    for (std::size_t second = first + 1; second < agents.size(); ++second)
    {
        const int_t firstNx = agents[first].model->stateDimension();
        const int_t secondNx = agents[second].model->stateDimension();
        std::vector<real_t> firstPosition(worldDimension, 0.0);
        std::vector<real_t> secondPosition(worldDimension, 0.0);
        for (int_t k = 0; k <= agents[first].horizon; ++k)
        {
            worldPosition(*agents[first].model, &states[first][k * firstNx],
                worldDimension, &firstPosition[0]);
            worldPosition(*agents[second].model, &states[second][k * secondNx],
                worldDimension, &secondPosition[0]);
            real_t squaredDistance = 0.0;
            for (int_t i = 0; i < worldDimension; ++i)
                squaredDistance += (firstPosition[i] - secondPosition[i])
                    * (firstPosition[i] - secondPosition[i]);
            const real_t margin = std::sqrt(squaredDistance)
                - pairSafetyDistance(agents[first], agents[second], options);
            if (margin < worstMargin)
            {
                worstMargin = margin;
                worstIsPair = true;
                worstFirst = first;
                worstSecond = second;
                worstStage = k;
            }
        }
    }

    std::ostringstream message;
    if (worstMargin >= -options.collisionTolerance)
        return "nominal geometry is feasible; inspect dynamics and bounds";
    if (worstIsPair)
        message << "agents " << worstFirst << " and " << worstSecond
            << " violate pairwise clearance at stage " << worstStage;
    else
        message << "agent " << worstFirst << " violates obstacle "
            << worstSecond << " clearance at stage " << worstStage;
    return message.str();
}
real_t terminalPositionError(
    const NonlinearAgentProblem& agent,
    const std::vector<real_t>& states)
{
    const int_t nx = agent.model->stateDimension();
    const int_t np = agent.model->positionDimension();
    std::vector<real_t> terminal(np, 0.0), reference(np, 0.0);
    agent.model->position(
        &states[agent.horizon * nx], &terminal[0]);
    agent.model->position(
        &agent.stateReference[agent.horizon * nx], &reference[0]);
    real_t squared = 0.0;
    for (int_t i = 0; i < np; ++i)
        squared += (terminal[i] - reference[i]) * (terminal[i] - reference[i]);
    return std::sqrt(squared);
}

real_t maximumTerminalPositionError(
    const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<std::vector<real_t> >& states)
{
    real_t maximum = 0.0;
    for (std::size_t a = 0; a < agents.size(); ++a)
        maximum = std::max(maximum, terminalPositionError(agents[a], states[a]));
    return maximum;
}

real_t maximumTerminalViolation(
    const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<std::vector<real_t> >& states,
    const NonlinearTurboOptions& options)
{
    real_t maximum = 0.0;
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        const real_t tolerance = terminalToleranceForAgent(
            agents[a], options);
        maximum = std::max(maximum,
            terminalPositionError(agents[a], states[a])
                - tolerance);
        if (!agents[a].enforceTerminalState) continue;
        const int_t nx = agents[a].model->stateDimension();
        const int_t terminal = agents[a].horizon * nx;
        for (int_t state = 0; state < nx; ++state)
        {
            if (!agents[a].terminalStateConstraintMask.empty()
                && !agents[a].terminalStateConstraintMask[state])
                continue;
            maximum = std::max(
                maximum,
                std::fabs(
                    states[a][terminal + state]
                    - agents[a].stateReference[terminal + state]
                ) - tolerance
            );
        }
    }
    return std::max(0.0, maximum);
}

real_t dynamicsDefect(const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<std::vector<real_t> >& states,
    const std::vector<std::vector<real_t> >& controls);

real_t merit(const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<std::vector<real_t> >& states,
    const std::vector<std::vector<real_t> >& controls,
    const std::vector<ConvexPolygonObstacle>& obstacles,
    const NonlinearTurboOptions& options)
{
    const real_t pairViolation = std::max(
        0.0, -minimumPairwiseClearance(agents, states, options));
    const real_t obstacleViolation = std::max(
        0.0, -minimumObstacleClearance(agents, states, obstacles, options)
    );
    const real_t terminalViolation = maximumTerminalViolation(
        agents, states, options);
    const real_t dynamicsViolation = std::max(
        0.0, dynamicsDefect(agents, states, controls)
            - options.dynamicsTolerance);
    return trajectoryCost(agents, states, controls)
        + options.meritPenalty
            * (pairViolation * pairViolation
                + obstacleViolation * obstacleViolation
                + terminalViolation * terminalViolation
                + dynamicsViolation * dynamicsViolation);
}

real_t dynamicsDefect(const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<std::vector<real_t> >& states,
    const std::vector<std::vector<real_t> >& controls)
{
    real_t maximum = 0.0;
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        const int_t nx = agents[a].model->stateDimension();
        const int_t nu = agents[a].model->controlDimension();
        std::vector<real_t> next(nx, 0.0);
        for (int_t k = 0; k < agents[a].horizon; ++k)
        {
            agents[a].model->dynamics(&states[a][k * nx], &controls[a][k * nu], &next[0]);
            for (int_t i = 0; i < nx; ++i)
                maximum = std::max(maximum, std::fabs(next[i] - states[a][(k + 1) * nx + i]));
        }
    }
    return maximum;
}

} // anonymous namespace

CollisionCircle::CollisionCircle()
    : longitudinalOffset(0.0), lateralOffset(0.0), radius(0.0) {}

CollisionCircle::CollisionCircle(
    real_t longitudinal,
    real_t lateral,
    real_t radiusValue)
    : longitudinalOffset(longitudinal), lateralOffset(lateral), radius(radiusValue) {}

NonlinearAgentProblem::NonlinearAgentProblem()
    : model(0), horizon(0), collisionRadius(-1.0),
      obstacleSafetyDistance(-1.0), terminalPositionTolerance(-1.0),
      enforceTerminalState(false) {}

NonlinearTurboOptions::NonlinearTurboOptions()
    : coordinationMethod(NCM_DISTRIBUTED_ADMM), continuationMode(NCONT_FULL),
      useRiccatiWarmStart(true), parallelAgentSolves(true), parallelAgentThreads(0),
      collisionSamplesPerInterval(2),
      maxScpIterations(8), maxAdmmIterations(40), minimumAdmmIterations(1),
      maxWorkingSetRecalculations(300), maxLineSearchSteps(6), rho(30.0),
      adaptiveRho(false), adaptiveRhoInterval(5), adaptiveRhoImbalance(10.0),
      adaptiveRhoScale(2.0), minimumRho(1.0), maximumRho(1000.0),
      inexactAdmmScpIterations(0), inexactAdmmToleranceMultiplier(1.0),
      polishingAdmmIterations(200),
      safetyDistance(1.0), pairSafetyBuffer(0.0),
      obstacleSafetyDistance(0.5), controlTrustRegion(1.0),
      maxRestorationAttempts(2), restorationTrustRegionShrink(0.5),
      minimumControlTrustRegion(0.1), restorationSlackPenalty(1.0e4),
      restorationSlackTolerance(1.0e-6),
      admmPrimalTolerance(1.0e-3), admmDualTolerance(1.0e-3),
      admmRelativeTolerance(1.0e-3), admmRelaxation(1.0),
      scpStepTolerance(1.0e-3), dynamicsTolerance(1.0e-7),
      collisionTolerance(1.0e-2),
      terminalPositionTolerance(2.0), meritPenalty(1.0e5),
      pairActivationDistance(-1.0), obstacleActivationDistance(-1.0),
      continuationMinimumNormalDot(0.5) {}

NonlinearTurboStatistics::NonlinearTurboStatistics()
    : scpIterations(0), admmIterations(0), qpSolves(0),
      qpWorkingSetRecalculations(0), backendIterations(0), lastQpStatus(0),
      failedAgent(-1), coldStarts(0), riccatiInitializations(0),
      riccatiFailures(0), hotstartFallbacks(0),
      matrixHotstarts(0), vectorHotstarts(0), transportedPairStages(0),
      resetPairStages(0), parallelQpBatches(0), parallelAgentThreads(1),
      rhoUpdates(0), restorationAttempts(0), successfulRestorations(0),
      lineSearchRecoveryAttempts(0), polishingScpIterations(0),
      admmConvergedSubproblems(0),
      admmIterationLimitSubproblems(0), maximumActivePairs(0),
      initialActivePairs(0), finalActivePairs(0), maximumPotentialPairs(0),
      maximumAgentDegree(0), maximumActiveObstaclesPerAgent(0),
      maximumPotentialObstaclesPerAgent(0), maximumCorridorRowsPerAgent(0),
      maximumLocalQpVariables(0),
      maximumLocalQpConstraints(0), centralizedQpVariables(0),
      centralizedQpConstraints(0), primalResidual(0.0), dualResidual(0.0),
      minimumAdmmRho(INFTY), maximumAdmmRho(0.0), finalAdmmRho(0.0),
      maximumRestorationSlack(0.0), finalRestorationSlack(0.0),
      minimumDistance(INFTY), primalStoppingThreshold(0.0),
      dualStoppingThreshold(0.0), minimumPairwiseClearance(INFTY),
      minimumObstacleDistance(INFTY), minimumObstacleClearance(INFTY),
      maximumDynamicsDefect(0.0), maximumTerminalPositionError(0.0),
      objective(0.0), solveTimeMilliseconds(0.0),
      maximumLocalQpSolveTimeMilliseconds(0.0),
      riccatiTimeMilliseconds(0.0), coldStartQpTimeMilliseconds(0.0),
      matrixHotstartQpTimeMilliseconds(0.0),
      vectorHotstartQpTimeMilliseconds(0.0),
      qpBuildTimeMilliseconds(0.0), pairBuildTimeMilliseconds(0.0),
      admmAssemblyTimeMilliseconds(0.0), localQpBatchTimeMilliseconds(0.0),
      localQpSolveTimeMilliseconds(0.0), consensusTimeMilliseconds(0.0),
      globalizationTimeMilliseconds(0.0) {}

NonlinearScpIterationStatistics::NonlinearScpIterationStatistics()
    : iteration(0), admmIterations(0), rhoUpdates(0), restorationAttempts(0),
      qpStatus(0), failedAgent(-1), qpSolved(false), admmConverged(false),
      restorationUsed(false), lineSearchRecovery(false), polishing(false),
      stepAccepted(false),
      objective(0.0), merit(0.0),
      primalResidual(0.0), dualResidual(0.0), rho(0.0),
      controlTrustRegion(0.0), maximumRestorationSlack(0.0),
      stepLength(0.0), maximumControlStep(0.0), maximumQpControlStep(0.0),
      minimumPairwiseClearance(INFTY), minimumObstacleClearance(INFTY),
      maximumDynamicsDefect(0.0), maximumTerminalPositionError(0.0) {}

NonlinearTurboResult::NonlinearTurboResult() : success(false), converged(false) {}

NonlinearTurboResult NonlinearTurboADMM::solve(
    const std::vector<NonlinearAgentProblem>& agents,
    const NonlinearTurboOptions& options) const
{
    return solve(
        agents,
        std::vector<ConvexPolygonObstacle>(),
        options
    );
}

NonlinearTurboResult NonlinearTurboADMM::solve(
    const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<ConvexPolygonObstacle>& obstacles,
    const NonlinearTurboOptions& options) const
{
    NonlinearTurboResult result;
    const std::chrono::steady_clock::time_point start = std::chrono::steady_clock::now();
    std::string validationError;
    if (!validateProblems(agents, validationError)) { result.status = validationError; return result; }
    if (!validateObstacles(agents, obstacles, validationError))
    {
        result.status = validationError;
        return result;
    }
    if (options.maxScpIterations <= 0 || options.maxAdmmIterations <= 0
        || options.minimumAdmmIterations <= 0
        || options.minimumAdmmIterations > options.maxAdmmIterations
        || !std::isfinite(options.admmPrimalTolerance)
        || options.admmPrimalTolerance < 0.0
        || !std::isfinite(options.admmDualTolerance)
        || options.admmDualTolerance < 0.0
        || !std::isfinite(options.admmRelaxation)
        || options.admmRelaxation <= 0.0
        || options.admmRelaxation >= 2.0
        || !std::isfinite(options.admmRelativeTolerance)
        || options.admmRelativeTolerance < 0.0
        || !std::isfinite(options.rho) || options.rho <= 0.0
        || options.adaptiveRhoInterval <= 0
        || !std::isfinite(options.adaptiveRhoImbalance)
        || options.adaptiveRhoImbalance <= 1.0
        || !std::isfinite(options.adaptiveRhoScale)
        || options.adaptiveRhoScale <= 1.0
        || !std::isfinite(options.minimumRho) || options.minimumRho <= 0.0
        || !std::isfinite(options.maximumRho) || options.maximumRho < options.minimumRho
        || (options.adaptiveRho
            && (options.rho < options.minimumRho || options.rho > options.maximumRho))
        || options.inexactAdmmScpIterations < 0
        || !std::isfinite(options.inexactAdmmToleranceMultiplier)
        || options.inexactAdmmToleranceMultiplier < 1.0
        || options.polishingAdmmIterations < options.maxAdmmIterations
        || !std::isfinite(options.safetyDistance)
        || !std::isfinite(options.pairSafetyBuffer)
        || !std::isfinite(options.pairActivationDistance)
        || options.pairActivationDistance < -1.0
        || !std::isfinite(options.obstacleActivationDistance)
        || options.obstacleActivationDistance < -1.0
        || options.safetyDistance < 0.0
        || options.pairSafetyBuffer < 0.0
        || !std::isfinite(options.obstacleSafetyDistance)
        || options.obstacleSafetyDistance < 0.0
        || !std::isfinite(options.collisionTolerance)
        || options.collisionTolerance < 0.0
        || !std::isfinite(options.dynamicsTolerance)
        || options.dynamicsTolerance < 0.0
        || !std::isfinite(options.terminalPositionTolerance)
        || options.terminalPositionTolerance < 0.0
        || options.collisionSamplesPerInterval <= 0
        || options.maxRestorationAttempts < 0
        || !std::isfinite(options.restorationTrustRegionShrink)
        || options.restorationTrustRegionShrink <= 0.0
        || options.restorationTrustRegionShrink >= 1.0
        || !std::isfinite(options.minimumControlTrustRegion)
        || options.minimumControlTrustRegion <= 0.0
        || !std::isfinite(options.restorationSlackPenalty)
        || options.restorationSlackPenalty <= 0.0
        || !std::isfinite(options.restorationSlackTolerance)
        || options.restorationSlackTolerance < 0.0
        || options.parallelAgentThreads < 0
        || options.coordinationMethod < NCM_DISTRIBUTED_ADMM
        || options.coordinationMethod > NCM_CENTRALIZED_OSQP
        || options.continuationMode < NCONT_COLD
        || options.continuationMode > NCONT_FULL
        || !std::isfinite(options.continuationMinimumNormalDot)
        || options.continuationMinimumNormalDot < -1.0
        || options.continuationMinimumNormalDot > 1.0)
    {
        result.status = "invalid solver options"; return result;
    }
    if (!validateEndpointGeometry(agents, obstacles, options, validationError))
    {
        result.status = validationError;
        return result;
    }

    std::vector<std::vector<real_t> > nominalControls(agents.size());
    std::vector<std::vector<real_t> > nominalStates(agents.size());
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        nominalControls[a] = agents[a].initialControls.empty()
            ? agents[a].controlReference : agents[a].initialControls;
        clampControls(agents[a], nominalControls[a]);
        if (agents[a].initialStates.empty())
            rollout(agents[a], nominalControls[a], nominalStates[a]);
        else
        {
            nominalStates[a] = agents[a].initialStates;
            std::copy(
                agents[a].initialState.begin(),
                agents[a].initialState.end(),
                nominalStates[a].begin());
        }
    }
    const std::vector<std::vector<real_t> > homotopyStates =
        nominalStates;

    std::vector<int> fullSpaceGlobalization(agents.size(), 0);
    for (std::size_t a = 0; a < agents.size(); ++a)
        fullSpaceGlobalization[a] =
            agents[a].initialStates.empty() ? 0 : 1;

    std::vector<AgentQp> activeQps;
    std::vector<PairData> previousPairs;
    SolverPool distributedSolvers;
    SolverPool reliableDistributedSolvers;
    SolverPool elasticDistributedSolvers;
    CentralContext centralContext;
    bool qpFailed = false;
    bool polishing = false;
    bool recoveringRejectedStep = false;
    bool adaptiveRhoSafeguard = false;
    int_t rejectedStepRecoveryAttempts = 0;
    int_t slowProgressIterations = 0;
    const int_t positionDimension = worldPositionDimension(agents);
    real_t currentTrustRegion = options.controlTrustRegion;
    for (int_t outer = 0; outer < options.maxScpIterations; ++outer)
    {
        NonlinearScpIterationStatistics trace;
        trace.iteration = outer + 1;
        trace.polishing = polishing;
        trace.lineSearchRecovery = recoveringRejectedStep;
        if (polishing) ++result.statistics.polishingScpIterations;
        trace.controlTrustRegion = options.controlTrustRegion;
        trace.objective = trajectoryCost(agents, nominalStates, nominalControls);
        trace.merit = merit(agents, nominalStates, nominalControls, obstacles, options);
        const int_t admmIterationsBefore = result.statistics.admmIterations;
        const int_t rhoUpdatesBefore = result.statistics.rhoUpdates;
        const int_t admmConvergedBefore = result.statistics.admmConvergedSubproblems;
        NonlinearTurboOptions iterationOptions = options;
        // Preserve the fast inexact path early in SCP.  Tighten consensus only
        // when pairwise clearance is the sole remaining feasibility defect.
        const int_t pairRepairIteration = std::max<int_t>(
            3, options.maxScpIterations / 2);
        const bool nominalDynamicsFeasible = dynamicsDefect(
            agents, nominalStates, nominalControls) <= options.dynamicsTolerance;
        const bool nominalTerminalFeasible = maximumTerminalViolation(
            agents, nominalStates, options) <= 0.0;
        const bool nominalObstacleFeasible = minimumObstacleClearance(
            agents, nominalStates, obstacles, options)
                >= -options.collisionTolerance;
        const real_t nominalPairwiseClearance = minimumPairwiseClearance(
            agents, nominalStates, options);
        const bool nominalPairwiseFeasible = nominalPairwiseClearance
            >= -options.collisionTolerance;
        const bool nominalFeasible = nominalDynamicsFeasible
            && nominalTerminalFeasible && nominalObstacleFeasible
            && nominalPairwiseFeasible;
        const bool exactRejectedStepRecovery = recoveringRejectedStep;
        const bool finalPairRepair =
            options.coordinationMethod == NCM_DISTRIBUTED_ADMM
            && outer + 1 >= pairRepairIteration
            && nominalDynamicsFeasible && nominalTerminalFeasible
            && nominalObstacleFeasible
            && nominalPairwiseClearance < -options.collisionTolerance;
        if (adaptiveRhoSafeguard)
            iterationOptions.adaptiveRho = false;
        if (polishing)
        {
            iterationOptions.adaptiveRho = false;
            iterationOptions.rho = options.rho;
            iterationOptions.inexactAdmmScpIterations = 0;
            iterationOptions.admmDualTolerance = options.admmPrimalTolerance;
            iterationOptions.admmRelativeTolerance = 0.0;
            iterationOptions.maxAdmmIterations = options.polishingAdmmIterations;
        }
        if (exactRejectedStepRecovery || finalPairRepair)
        {
            iterationOptions.adaptiveRho = false;
            iterationOptions.rho = options.rho;
            iterationOptions.inexactAdmmScpIterations = 0;
            const real_t recoveryTolerance = std::max(
                1.0e-12,
                std::min(options.admmPrimalTolerance,
                    0.1 * options.collisionTolerance));
            iterationOptions.admmPrimalTolerance = recoveryTolerance;
            iterationOptions.admmDualTolerance = recoveryTolerance;
            iterationOptions.admmRelativeTolerance = 0.0;
            iterationOptions.maxAdmmIterations = std::max(
                options.maxAdmmIterations, options.polishingAdmmIterations);
        }
        std::vector<AgentQp> qps;
        std::vector<PairData> pairs;
        bool solved = false;
        bool restorationUsed = false;
        real_t attemptTrustRegion = currentTrustRegion;
        for (int_t attempt = 0;
             attempt <= options.maxRestorationAttempts; ++attempt)
        {
            const bool restorationAttempt = attempt > 0;
            const bool elasticConstraints = attempt > 1;
            if (restorationAttempt)
            {
                restorationUsed = true;
                ++trace.restorationAttempts;
                ++result.statistics.restorationAttempts;
                if (options.coordinationMethod != NCM_DISTRIBUTED_ADMM)
                    centralContext.reset();
            }

            const std::chrono::steady_clock::time_point qpBuildStart =
                std::chrono::steady_clock::now();
            qps.assign(agents.size(), AgentQp());
            for (std::size_t a = 0; a < agents.size(); ++a)
                buildAgentQp(
                    agents[a],
                    nominalStates[a],
                    homotopyStates[a],
                    nominalControls[a],
                    attemptTrustRegion,
                    elasticConstraints,
                    options.restorationSlackPenalty,
                    obstacles,
                    obstacleDistanceForAgent(agents[a], options),
                    options.obstacleActivationDistance,
                    terminalToleranceForAgent(agents[a], options),
                    options.collisionSamplesPerInterval,
                    positionDimension,
                    qps[a]
                );
            int_t potentialObstacles = 0;
            for (std::size_t a = 0; a < agents.size(); ++a)
                potentialObstacles = std::max(
                    potentialObstacles,
                    static_cast<int_t>(obstacles.size())
                        * collisionCircleCount(agents[a]));
            result.statistics.maximumPotentialObstaclesPerAgent = std::max(
                result.statistics.maximumPotentialObstaclesPerAgent,
                potentialObstacles);
            for (std::size_t a = 0; a < qps.size(); ++a)
                result.statistics.maximumActiveObstaclesPerAgent = std::max(
                    result.statistics.maximumActiveObstaclesPerAgent,
                    qps[a].activeObstacleCount);
            result.statistics.qpBuildTimeMilliseconds += static_cast<real_t>(
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - qpBuildStart).count());

            const std::chrono::steady_clock::time_point pairBuildStart =
                std::chrono::steady_clock::now();
            const std::vector<PairData>* continuationPairs =
                options.coordinationMethod == NCM_DISTRIBUTED_ADMM
                && options.continuationMode == NCONT_FULL
                && !previousPairs.empty() ? &previousPairs : 0;
            pairs = buildPairs(
                agents,
                nominalStates,
                positionDimension,
                options,
                continuationPairs,
                result.statistics
            );
            if (outer == 0 && attempt == 0)
                result.statistics.initialActivePairs = static_cast<int_t>(pairs.size());
            result.statistics.finalActivePairs = static_cast<int_t>(pairs.size());
            result.statistics.pairBuildTimeMilliseconds += static_cast<real_t>(
                std::chrono::duration<double, std::milli>(
                    std::chrono::steady_clock::now() - pairBuildStart).count());

            result.statistics.lastQpStatus = 0;
            result.statistics.failedAgent = -1;
            NonlinearTurboOptions solveOptions = iterationOptions;
            if (restorationAttempt)
            {
                solveOptions.inexactAdmmScpIterations = 0;
                solveOptions.maxAdmmIterations = std::max(
                    options.maxAdmmIterations, options.polishingAdmmIterations);
            }
            SolverPool* attemptSolvers = &distributedSolvers;
            if (elasticConstraints)
                attemptSolvers = &elasticDistributedSolvers;
            else if (restorationAttempt)
                attemptSolvers = &reliableDistributedSolvers;
            solved = options.coordinationMethod == NCM_DISTRIBUTED_ADMM
                ? solveDistributed(
                    agents, outer, solveOptions,
                    restorationAttempt || recoveringRejectedStep,
                    *attemptSolvers, pairs, qps, result.statistics)
                : solveCentralized(
                    pairs, outer, solveOptions,
                    restorationAttempt || recoveringRejectedStep,
                    centralContext, qps, result.statistics);
            if (solved)
            {
                if (restorationAttempt) ++result.statistics.successfulRestorations;
                break;
            }
        }

        trace.qpSolved = solved;
        trace.qpStatus = result.statistics.lastQpStatus;
        trace.failedAgent = result.statistics.failedAgent;
        trace.restorationUsed = restorationUsed;
        trace.controlTrustRegion = attemptTrustRegion;
        trace.admmIterations = result.statistics.admmIterations - admmIterationsBefore;
        trace.rhoUpdates = result.statistics.rhoUpdates - rhoUpdatesBefore;
        trace.admmConverged = options.coordinationMethod != NCM_DISTRIBUTED_ADMM
            || result.statistics.admmConvergedSubproblems > admmConvergedBefore;
        trace.primalResidual = result.statistics.primalResidual;
        trace.dualResidual = result.statistics.dualResidual;
        trace.rho = result.statistics.finalAdmmRho;
        if (solved && restorationUsed)
            for (std::size_t a = 0; a < qps.size(); ++a)
                trace.maximumRestorationSlack = std::max(
                    trace.maximumRestorationSlack,
                    maximumRestorationSlack(qps[a])
                );
        result.statistics.maximumRestorationSlack = std::max(
            result.statistics.maximumRestorationSlack,
            trace.maximumRestorationSlack
        );
        result.statistics.finalRestorationSlack = trace.maximumRestorationSlack;
        if (!solved)
        {
            result.scpTrace.push_back(trace);
            qpFailed = true;
            result.status = "a convex SCP subproblem failed after restoration: "
                + geometryDiagnostic(agents, nominalStates, obstacles, options);
            break;
        }
        if (options.coordinationMethod == NCM_DISTRIBUTED_ADMM
            && options.continuationMode == NCONT_FULL) previousPairs = pairs;
        const std::chrono::steady_clock::time_point globalizationStart =
            std::chrono::steady_clock::now();

        std::vector<std::vector<real_t> > qpStates(agents.size());
        std::vector<std::vector<real_t> > qpControls(agents.size());
        for (std::size_t a = 0; a < agents.size(); ++a)
        {
            qpStates[a].assign((qps[a].N + 1) * qps[a].nx, 0.0);
            for (int_t k = 0; k <= qps[a].N; ++k)
                std::copy(qps[a].solution.begin() + stateIndex(qps[a], k),
                    qps[a].solution.begin() + stateIndex(qps[a], k) + qps[a].nx,
                    qpStates[a].begin() + k * qps[a].nx);
            qpControls[a].assign(qps[a].N * qps[a].nu, 0.0);
            for (int_t k = 0; k < qps[a].N; ++k)
                std::copy(qps[a].solution.begin() + controlIndex(qps[a], k),
                    qps[a].solution.begin() + controlIndex(qps[a], k) + qps[a].nu,
                    qpControls[a].begin() + k * qps[a].nu);
        }

        real_t bestMerit = merit(
            agents,
            nominalStates,
            nominalControls,
            obstacles,
            options
        );
        const real_t initialMerit = bestMerit;
        real_t bestAlpha = 0.0;
        std::vector<std::vector<real_t> > bestStates = nominalStates;
        std::vector<std::vector<real_t> > bestControls = nominalControls;
        real_t alpha = 1.0;
        for (int_t line = 0; line < options.maxLineSearchSteps; ++line)
        {
            std::vector<std::vector<real_t> > candidateControls(agents.size());
            std::vector<std::vector<real_t> > candidateStates(agents.size());
            for (std::size_t a = 0; a < agents.size(); ++a)
            {
                if (fullSpaceGlobalization[a])
                {
                    candidateStates[a].resize(nominalStates[a].size());
                    for (std::size_t i = 0; i < candidateStates[a].size(); ++i)
                        candidateStates[a][i] = nominalStates[a][i]
                            + alpha * (qpStates[a][i] - nominalStates[a][i]);
                }
                candidateControls[a].resize(nominalControls[a].size());
                for (std::size_t i = 0; i < candidateControls[a].size(); ++i)
                    candidateControls[a][i] = nominalControls[a][i]
                        + alpha * (qpControls[a][i] - nominalControls[a][i]);
                clampControls(agents[a], candidateControls[a]);
                if (!fullSpaceGlobalization[a])
                    rollout(agents[a], candidateControls[a], candidateStates[a]);
            }
            const real_t candidateMerit = merit(
                agents,
                candidateStates,
                candidateControls,
                obstacles,
                options
            );
            if (candidateMerit < bestMerit - 1.0e-10)
            {
                bestMerit = candidateMerit; bestAlpha = alpha;
                bestStates.swap(candidateStates); bestControls.swap(candidateControls);
                break;
            }
            alpha *= 0.5;
        }
        // A full-state warm start can keep making tiny merit improvements while
        // retaining a nonlinear dynamics defect.  Let the exact rollout compete
        // with every accepted full-space step, not only with a rejected step.
        // A filter step may also enter the exact-dynamics manifold with a
        // recoverable terminal error when that rollout already satisfies the
        // physical collision constraints.
        {
            bool hasFullStateWarmStart = false;
            std::vector<std::vector<real_t> > projectedStates = bestStates;
            for (std::size_t a = 0; a < agents.size(); ++a)
                if (fullSpaceGlobalization[a])
                {
                    hasFullStateWarmStart = true;
                    rollout(agents[a], bestControls[a], projectedStates[a]);
                }
            if (hasFullStateWarmStart)
            {
                const real_t currentDynamicsDefect = dynamicsDefect(
                    agents, bestStates, bestControls);
                const real_t projectedDynamicsDefect = dynamicsDefect(
                    agents, projectedStates, bestControls);
                const real_t projectionMerit = merit(
                    agents, projectedStates, bestControls, obstacles, options);
                int_t maximumHorizon = 0;
                for (std::size_t a = 0; a < agents.size(); ++a)
                    maximumHorizon = std::max(maximumHorizon, agents[a].horizon);
                const real_t accumulatedDefectBound = std::sqrt(
                    static_cast<real_t>(worldPositionDimension(agents)))
                    * maximumHorizon * currentDynamicsDefect;
                const bool safeDynamicsRestoration =
                    currentDynamicsDefect > options.dynamicsTolerance
                    && projectedDynamicsDefect <= options.dynamicsTolerance
                    && minimumPairwiseClearance(
                        agents, projectedStates, options)
                        >= -options.collisionTolerance
                    && minimumObstacleClearance(
                        agents, projectedStates, obstacles, options)
                        >= -options.collisionTolerance
                    && maximumTerminalViolation(
                        agents, projectedStates, options)
                        <= maximumTerminalViolation(
                            agents, bestStates, options)
                            + accumulatedDefectBound;
                if (projectionMerit < bestMerit - 1.0e-10
                    || safeDynamicsRestoration)
                {
                    bestMerit = projectionMerit;
                    bestAlpha = 1.0;
                    bestStates.swap(projectedStates);
                    for (std::size_t a = 0; a < agents.size(); ++a)
                        fullSpaceGlobalization[a] = 0;
                }
            }
        }
        real_t maximumStep = 0.0;
        real_t maximumQpStep = 0.0;
        for (std::size_t a = 0; a < agents.size(); ++a)
            for (std::size_t i = 0; i < nominalControls[a].size(); ++i)
                maximumQpStep = std::max(maximumQpStep,
                    std::fabs(qpControls[a][i] - nominalControls[a][i]));
        if (bestAlpha <= 0.0)
        {
            real_t contractedTrustRegion =
                attemptTrustRegion * options.restorationTrustRegionShrink;
            real_t trustRegionFloor = options.minimumControlTrustRegion;
            // Jump to verification scale only after polishing has begun or
            // the rejected QP direction is already within one order of the
            // requested SCP step tolerance.
            if (nominalFeasible
                && (polishing || exactRejectedStepRecovery
                    || maximumQpStep <= 10.0 * options.scpStepTolerance))
            {
                trustRegionFloor = 0.5 * options.scpStepTolerance;
                contractedTrustRegion = trustRegionFloor;
            }
            currentTrustRegion = std::max(
                trustRegionFloor, contractedTrustRegion);
        }
        // A heavily damped terminal/dynamics restoration step signals a poor
        // local model. Keep the established trust policy for collision repair,
        // where shrinking here can lock the iterate into a colliding homotopy.
        else if (bestAlpha < 0.25
            && minimumPairwiseClearance(agents, bestStates, options)
                >= -options.collisionTolerance
            && minimumObstacleClearance(
                agents, bestStates, obstacles, options)
                >= -options.collisionTolerance
            && (maximumTerminalViolation(agents, bestStates, options) > 0.0
                || dynamicsDefect(agents, bestStates, bestControls)
                    > options.dynamicsTolerance))
        {
            currentTrustRegion = std::max(
                options.minimumControlTrustRegion,
                attemptTrustRegion * options.restorationTrustRegionShrink);
        }
        else if (bestAlpha >= 1.0 - 1.0e-12)
        {
            currentTrustRegion = std::min(
                options.controlTrustRegion,
                attemptTrustRegion / options.restorationTrustRegionShrink
            );
        }
        for (std::size_t a = 0; a < agents.size(); ++a)
            for (std::size_t i = 0; i < nominalControls[a].size(); ++i)
                maximumStep = std::max(maximumStep, std::fabs(bestControls[a][i] - nominalControls[a][i]));
        const real_t relativeMeritDecrease = (initialMerit - bestMerit)
            / std::max(1.0, std::fabs(initialMerit));
        if (bestAlpha > 0.0)
        {
            if (relativeMeritDecrease <= options.scpStepTolerance)
                ++slowProgressIterations;
            else
                slowProgressIterations = 0;
        }
        const bool coupledInteractionGraph =
            result.statistics.maximumAgentDegree > 1;

        result.statistics.globalizationTimeMilliseconds += static_cast<real_t>(
            std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - globalizationStart).count());
        nominalStates.swap(bestStates); nominalControls.swap(bestControls); activeQps.swap(qps);
        result.statistics.scpIterations = outer + 1;
        const real_t pairwiseClearance = minimumPairwiseClearance(agents, nominalStates, options);
        const real_t obstacleClearance = minimumObstacleClearance(
            agents,
            nominalStates,
            obstacles,
            options
        );
        const bool terminalFeasible = maximumTerminalViolation(
            agents, nominalStates, options) <= 0.0;
        trace.stepAccepted = bestAlpha > 0.0;
        trace.stepLength = bestAlpha;
        trace.maximumControlStep = maximumStep;
        trace.maximumQpControlStep = maximumQpStep;
        trace.objective = trajectoryCost(agents, nominalStates, nominalControls);
        trace.merit = bestMerit;
        trace.minimumPairwiseClearance = pairwiseClearance;
        trace.minimumObstacleClearance = obstacleClearance;
        trace.maximumDynamicsDefect = dynamicsDefect(
            agents, nominalStates, nominalControls);
        trace.maximumTerminalPositionError = maximumTerminalPositionError(
            agents, nominalStates);
        const bool rejectedStepStationary = bestAlpha <= 0.0
            && recoveringRejectedStep
            && (polishing || exactRejectedStepRecovery) && trace.admmConverged
            && maximumQpStep <= options.scpStepTolerance;
        const bool sustainedStrictStagnation = bestAlpha > 0.0
            && polishing && trace.admmConverged
            && slowProgressIterations >= 3
            && (coupledInteractionGraph
                || maximumQpStep <= 10.0 * options.scpStepTolerance);
        const bool nonlinearConverged =
            ((bestAlpha > 0.0
                && maximumStep <= options.scpStepTolerance
                && maximumQpStep <= options.scpStepTolerance)
                || rejectedStepStationary || sustainedStrictStagnation)
            && pairwiseClearance >= -options.collisionTolerance
            && obstacleClearance >= -options.collisionTolerance
            && terminalFeasible
            && dynamicsDefect(agents, nominalStates, nominalControls)
                <= options.dynamicsTolerance
            && trace.maximumRestorationSlack <= options.restorationSlackTolerance;
        if (bestAlpha <= 0.0 && !rejectedStepStationary)
        {
            if (rejectedStepRecoveryAttempts < options.maxRestorationAttempts)
            {
                ++rejectedStepRecoveryAttempts;
                ++result.statistics.lineSearchRecoveryAttempts;
                recoveringRejectedStep = true;
                if (!(nominalFeasible
                        && maximumQpStep
                            <= 10.0 * options.scpStepTolerance))
                    polishing = false;
                adaptiveRhoSafeguard = true;
                result.scpTrace.push_back(trace);
                continue;
            }
            result.scpTrace.push_back(trace);
            result.status = "line-search recovery exhausted with a nonstationary QP step: "
                + geometryDiagnostic(agents, nominalStates, obstacles, options);
            break;
        }
        if (bestAlpha > 0.0)
        {
            recoveringRejectedStep = false;
            rejectedStepRecoveryAttempts = 0;
        }
        result.scpTrace.push_back(trace);
        if (nonlinearConverged)
        {
            if (options.coordinationMethod == NCM_DISTRIBUTED_ADMM && !polishing)
            {
                polishing = true;
                continue;
            }
            if (trace.admmConverged)
            {
                result.converged = true;
                break;
            }
        }
        else if (options.coordinationMethod == NCM_DISTRIBUTED_ADMM
            && !polishing
            && bestAlpha > 0.0
            && (coupledInteractionGraph
                || maximumQpStep <= 10.0 * options.scpStepTolerance)
            && relativeMeritDecrease <= options.scpStepTolerance)
        {
            // Merit stagnation requests exact consensus for a coupled graph,
            // or after a sparse-graph direction is already locally small.
            polishing = true;
            continue;
        }
        else if (polishing && bestAlpha > 0.0)
            polishing = false;
    }

    result.trajectories.resize(agents.size());
    for (std::size_t a = 0; a < agents.size(); ++a)
    {
        result.trajectories[a].states = nominalStates[a];
        result.trajectories[a].controls = nominalControls[a];
    }
    result.statistics.minimumDistance = minimumDistance(agents, nominalStates);
    result.statistics.maximumDynamicsDefect = dynamicsDefect(agents, nominalStates, nominalControls);
    result.statistics.maximumTerminalPositionError = maximumTerminalPositionError(
        agents, nominalStates);
    result.statistics.minimumPairwiseClearance = minimumPairwiseClearance(
        agents,
        nominalStates,
        options
    );
    result.statistics.minimumObstacleDistance = minimumObstacleDistance(
        agents,
        nominalStates,
        obstacles
    );
    result.statistics.minimumObstacleClearance = minimumObstacleClearance(
        agents,
        nominalStates,
        obstacles,
        options
    );
    result.statistics.objective = trajectoryCost(agents, nominalStates, nominalControls);
    result.success = result.statistics.minimumPairwiseClearance >= -options.collisionTolerance
        && result.statistics.minimumObstacleClearance >= -options.collisionTolerance
        && maximumTerminalViolation(agents, nominalStates, options) <= 0.0
        && result.statistics.maximumDynamicsDefect <= options.dynamicsTolerance;
    if (qpFailed && result.success)
        result.status =
            "returning a feasible incumbent after QP restoration exhausted";
    if (result.status.empty())
        result.status = result.converged ? "converged" : (result.success
            ? "maximum SCP iterations reached with a feasible trajectory"
            : "trajectory remains collision-infeasible: "
                + geometryDiagnostic(agents, nominalStates, obstacles, options));
    const std::chrono::duration<double, std::milli> elapsed = std::chrono::steady_clock::now() - start;
    result.statistics.solveTimeMilliseconds = static_cast<real_t>(elapsed.count());
    return result;
}

END_NAMESPACE_QPOASES
