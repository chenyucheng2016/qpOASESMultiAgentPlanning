#include <qpOASES/NonlinearTrajectoryValidator.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <memory>
#include <random>
#include <sstream>
#include <string>
#include <vector>

USING_NAMESPACE_QPOASES

namespace
{

const real_t pi = 3.14159265358979323846;

struct BenchmarkScenario
{
    std::string caseId;
    std::string difficulty;
    std::string family;
    std::string density;
    std::string composition;
    int seed;
    int obstacleCount;
    std::vector<std::unique_ptr<NonlinearModel> > models;
    std::vector<NonlinearAgentProblem> agents;
    std::vector<ConvexPolygonObstacle> obstacles;
};

struct ManualCase
{
    std::string id;
    std::string difficulty;
    int agentCount;
    int obstacleCount;
    std::string composition;
    std::string family;
};

struct Method
{
    std::string name;
    NonlinearCoordinationMethod coordination;
    NonlinearContinuationMode continuation;
    bool riccati;
};

struct Arguments
{
    std::string suite;
    std::string output;
    std::string method;
    std::string track;
    std::string caseId;
    int scenarioIndex;
    int threads;
    bool fixedRho;
    bool exactAdmm;
    bool dryRun;
    bool requireSuccess;
    bool requireConvergence;
    Arguments() : suite("smoke"), output("nonlinear_benchmark.csv"), method("all"),
        track("all"), caseId("all"), scenarioIndex(-1), threads(0),
        fixedRho(false), exactAdmm(false), dryRun(false), requireSuccess(false),
        requireConvergence(false) {}
};

std::string csvText(const std::string& text)
{
    std::string escaped;
    for (std::size_t i = 0; i < text.size(); ++i)
    {
        if (text[i] == '"') escaped += "\"\"";
        else if (text[i] == '\n' || text[i] == '\r') escaped += ' ';
        else escaped += text[i];
    }
    return "\"" + escaped + "\"";
}

bool parseArguments(int argc, char** argv, Arguments& arguments)
{
    for (int i = 1; i < argc; ++i)
    {
        const std::string option = argv[i];
        if (option == "--dry-run")
        {
            arguments.dryRun = true;
            continue;
        }
        if (option == "--require-success")
        {
            arguments.requireSuccess = true;
            continue;
        }
        if (option == "--require-convergence")
        {
            arguments.requireConvergence = true;
            continue;
        }
        if (option == "--fixed-rho")
        {
            arguments.fixedRho = true;
            continue;
        }
        if (option == "--exact-admm")
        {
            arguments.exactAdmm = true;
            continue;
        }
        if ((option == "--scenario-index" || option == "--threads") && i + 1 < argc)
        {
            char* end = 0;
            const long value = std::strtol(argv[++i], &end, 10);
            if (end == argv[i] || *end != '\0'
                || (option == "--scenario-index" ? value < 0 : value <= 0))
                return false;
            if (option == "--scenario-index")
                arguments.scenarioIndex = static_cast<int>(value);
            else arguments.threads = static_cast<int>(value);
            continue;
        }
        if ((option == "--suite" || option == "--output" || option == "--method" || option == "--track" || option == "--case")
            && i + 1 < argc)
        {
            const std::string value = argv[++i];
            if (option == "--suite") arguments.suite = value;
            else if (option == "--output") arguments.output = value;
            else if (option == "--method") arguments.method = value;
            else if (option == "--track") arguments.track = value;
            else arguments.caseId = value;
        }
        else return false;
    }
    const bool validSuite = arguments.suite == "smoke"
        || arguments.suite == "ci"
        || arguments.suite == "development"
        || arguments.suite == "final"
        || arguments.suite == "manual";
    const bool validTrack = arguments.track == "all" || arguments.track == "scaling"
        || arguments.track == "models" || arguments.track == "families";
    return validSuite && validTrack;
}

std::vector<int> agentCounts(const std::string& suite)
{
    if (suite == "smoke") return std::vector<int>{2, 4};
    if (suite == "ci") return std::vector<int>{2};
    return std::vector<int>{2, 4, 8, 14, 20};
}

std::vector<int> obstacleCounts(const std::string& suite)
{
    if (suite == "smoke") return std::vector<int>{0, 4};
    if (suite == "ci") return std::vector<int>{0};
    return std::vector<int>{0, 4, 8, 16, 32};
}

std::vector<int> seeds(const std::string& suite)
{
    int first = 0, count = 3;
    if (suite == "ci") count = 1;
    if (suite == "development") { first = 1000; count = 10; }
    if (suite == "final") { first = 10000; count = 30; }
    std::vector<int> values;
    for (int seed = first; seed < first + count; ++seed) values.push_back(seed);
    return values;
}

std::vector<std::string> compositions(const std::string& suite)
{
    if (suite == "smoke") return std::vector<std::string>{"unicycle", "balanced"};
    if (suite == "ci") return std::vector<std::string>{"unicycle"};
    return std::vector<std::string>{"unicycle", "bicycle", "quadcopter", "balanced"};
}

std::vector<std::string> scenarioFamilies(
    const std::string& suite,
    const std::string& density,
    int obstacleCount)
{
    if (density == "constant_density")
        return obstacleCount == 0
            ? std::vector<std::string>{"local_exchange"}
            : std::vector<std::string>{"local_exchange_obstacles"};
    if (obstacleCount == 0) return std::vector<std::string>{"antipodal"};
    if (suite == "smoke") return std::vector<std::string>{"random_convex_field"};
    return std::vector<std::string>{
        "random_convex_field", "bottleneck", "warehouse_aisles",
        "multiple_homotopy_maze"
    };
}
std::vector<std::string> densityRegimes(const std::string& suite)
{
    if (suite == "ci" || suite == "smoke")
        return std::vector<std::string>{"fixed"};
    return std::vector<std::string>{"fixed", "constant_density"};
}

bool includedInTrack(
    const Arguments& arguments,
    int agentCount,
    int obstacleCount,
    const std::string& density,
    const std::string& composition,
    const std::string& family)
{
    if (arguments.suite == "ci" || arguments.suite == "smoke") return true;
    const bool scaling = composition == "balanced"
        && (family == "antipodal" || family == "random_convex_field"
            || family == "local_exchange" || family == "local_exchange_obstacles");
    const bool models = density == "fixed"
        && (agentCount == 4 || agentCount == 8)
        && (obstacleCount == 0 || obstacleCount == 8)
        && (family == "antipodal" || family == "random_convex_field");
    const bool families = density == "fixed"
        && composition == "balanced"
        && agentCount == 8
        && (obstacleCount == 8 || obstacleCount == 16);
    if (arguments.track == "scaling") return scaling;
    if (arguments.track == "models") return models;
    if (arguments.track == "families") return families;
    return scaling || models || families;
}

std::vector<ManualCase> manualCases()
{
    std::vector<ManualCase> cases;
    cases.push_back(ManualCase{"easy_open", "easy", 2, 0, "unicycle", "antipodal"});
    cases.push_back(ManualCase{"easy_single_blocker", "easy", 2, 1, "unicycle", "single_blocker"});
    cases.push_back(ManualCase{"medium_doorway", "medium", 4, 2, "unicycle", "doorway"});
    cases.push_back(ManualCase{"medium_heterogeneous_open", "medium", 4, 0, "balanced", "antipodal"});
    cases.push_back(ManualCase{"hard_heterogeneous_doorway", "hard", 4, 2, "balanced", "doorway"});
    cases.push_back(ManualCase{"hard_warehouse", "hard", 8, 8, "balanced", "warehouse_aisles"});
    cases.push_back(ManualCase{"very_hard_maze", "very_hard", 8, 16, "balanced", "multiple_homotopy_maze"});
    return cases;
}

void fillUnicycle(
    NonlinearAgentProblem& problem,
    const UnicycleModel& model,
    int_t horizon,
    real_t startX,
    real_t startY,
    real_t goalX,
    real_t goalY,
    int)
{
    const real_t referenceHeading = std::atan2(goalY - startY, goalX - startX);
    const real_t distance = std::sqrt(
        (goalX - startX) * (goalX - startX)
        + (goalY - startY) * (goalY - startY)
    );
    const real_t duration = horizon * model.timeStep();
    const real_t routeRadius = std::sqrt(startX * startX + startY * startY);
    const real_t referenceSpeed = std::min(2.0, distance / duration);
    const real_t arcSpeed = std::min(2.5, pi * routeRadius / duration);
    const real_t tangentHeading = std::atan2(-startX, startY);
    const real_t yawRate = -pi / duration;
    problem.model = &model;
    problem.horizon = horizon;
    problem.collisionRadius = 0.35;
    problem.obstacleSafetyDistance = 0.30;
    const real_t initial[] = {startX, startY, tangentHeading, arcSpeed};
    problem.initialState.assign(initial, initial + 4);
    problem.stateReference.assign((horizon + 1) * 4, 0.0);
    problem.controlReference.assign(horizon * 2, 0.0);
    problem.initialControls.assign(horizon * 2, 0.0);
    for (int_t k = 0; k <= horizon; ++k)
    {
        const real_t alpha = static_cast<real_t>(k) / horizon;
        problem.stateReference[k * 4] = (1.0 - alpha) * startX + alpha * goalX;
        problem.stateReference[k * 4 + 1] = (1.0 - alpha) * startY + alpha * goalY;
        problem.stateReference[k * 4 + 2] = referenceHeading;
        problem.stateReference[k * 4 + 3] = referenceSpeed;
        if (k < horizon) problem.initialControls[k * 2 + 1] = yawRate;
    }
    const real_t q[] = {2.0, 2.0, 0.15, 0.1};
    const real_t qTerminal[] = {20.0, 20.0, 1.0, 0.5};
    const real_t r[] = {0.1, 0.25};
    const real_t lowerState[] = {-40.0, -40.0, -20.0, 0.0};
    const real_t upperState[] = {40.0, 40.0, 20.0, 3.0};
    const real_t lowerControl[] = {-2.0, -1.2};
    const real_t upperControl[] = {2.0, 1.2};
    problem.stateWeights.assign(q, q + 4);
    problem.terminalWeights.assign(qTerminal, qTerminal + 4);
    problem.controlWeights.assign(r, r + 2);
    problem.stateLowerBounds.assign(lowerState, lowerState + 4);
    problem.stateUpperBounds.assign(upperState, upperState + 4);
    problem.controlLowerBounds.assign(lowerControl, lowerControl + 2);
    problem.controlUpperBounds.assign(upperControl, upperControl + 2);
}

void fillBicycle(
    NonlinearAgentProblem& problem,
    const BicycleModel& model,
    int_t horizon,
    real_t startX,
    real_t startY,
    real_t goalX,
    real_t goalY,
    int)
{
    const real_t referenceHeading = std::atan2(goalY - startY, goalX - startX);
    const real_t distance = std::sqrt(
        (goalX - startX) * (goalX - startX)
        + (goalY - startY) * (goalY - startY)
    );
    const real_t duration = horizon * model.timeStep();
    const real_t routeRadius = std::sqrt(startX * startX + startY * startY);
    const real_t referenceSpeed = std::min(2.0, distance / duration);
    const real_t arcSpeed = std::min(2.5, pi * routeRadius / duration);
    const real_t tangentHeading = std::atan2(-startX, startY);
    const real_t yawRate = -pi / duration;
    const real_t steering = std::atan(2.0 * yawRate / arcSpeed);
    problem.model = &model;
    problem.horizon = horizon;
    problem.collisionRadius = 0.55;
    problem.obstacleSafetyDistance = 0.40;
    const real_t initial[] = {startX, startY, tangentHeading, arcSpeed, steering};
    problem.initialState.assign(initial, initial + 5);
    problem.stateReference.assign((horizon + 1) * 5, 0.0);
    problem.controlReference.assign(horizon * 2, 0.0);
    problem.initialControls.assign(horizon * 2, 0.0);
    for (int_t k = 0; k <= horizon; ++k)
    {
        const real_t alpha = static_cast<real_t>(k) / horizon;
        problem.stateReference[k * 5] = (1.0 - alpha) * startX + alpha * goalX;
        problem.stateReference[k * 5 + 1] = (1.0 - alpha) * startY + alpha * goalY;
        problem.stateReference[k * 5 + 2] = referenceHeading;
        problem.stateReference[k * 5 + 3] = referenceSpeed;
    }
    const real_t q[] = {2.0, 2.0, 0.15, 0.1, 0.1};
    const real_t qTerminal[] = {20.0, 20.0, 1.0, 0.5, 0.2};
    const real_t r[] = {0.1, 0.25};
    const real_t lowerState[] = {-40.0, -40.0, -20.0, 0.0, -0.65};
    const real_t upperState[] = {40.0, 40.0, 20.0, 3.0, 0.65};
    const real_t lowerControl[] = {-2.0, -1.2};
    const real_t upperControl[] = {2.0, 1.2};
    problem.stateWeights.assign(q, q + 5);
    problem.terminalWeights.assign(qTerminal, qTerminal + 5);
    problem.controlWeights.assign(r, r + 2);
    problem.stateLowerBounds.assign(lowerState, lowerState + 5);
    problem.stateUpperBounds.assign(upperState, upperState + 5);
    problem.controlLowerBounds.assign(lowerControl, lowerControl + 2);
    problem.controlUpperBounds.assign(upperControl, upperControl + 2);
}

void fillQuadcopter(
    NonlinearAgentProblem& problem,
    const QuadcopterModel& model,
    int_t horizon,
    real_t startX,
    real_t startY,
    real_t goalX,
    real_t goalY,
    int index)
{
    const real_t duration = horizon * model.timeStep();
    const real_t referenceVx = (goalX - startX) / duration;
    const real_t referenceVy = (goalY - startY) / duration;
    const real_t routeRadius = std::sqrt(startX * startX + startY * startY);
    const real_t angularRate = pi / duration;
    const real_t initialVx = angularRate * startY;
    const real_t initialVy = -angularRate * startX;
    const real_t radialAcceleration = angularRate * angularRate * routeRadius;
    const real_t pitch = -std::atan(
        radialAcceleration / model.gravityAcceleration()
    );
    const real_t orbitThrust = model.vehicleMass() * std::sqrt(
        model.gravityAcceleration() * model.gravityAcceleration()
        + radialAcceleration * radialAcceleration
    );
    const real_t altitude = 0.55 + 0.15 * (index % 2);
    problem.model = &model;
    problem.horizon = horizon;
    problem.collisionRadius = 0.30;
    problem.obstacleSafetyDistance = 0.30;
    const real_t initial[] = {
        startX, startY, altitude, initialVx, initialVy, 0.0,
        0.0, pitch, std::atan2(startY, startX)
    };
    problem.initialState.assign(initial, initial + 9);
    problem.stateReference.assign((horizon + 1) * 9, 0.0);
    problem.controlReference.assign(horizon * 4, 0.0);
    problem.initialControls.assign(horizon * 4, 0.0);
    for (int_t k = 0; k <= horizon; ++k)
    {
        const real_t alpha = static_cast<real_t>(k) / horizon;
        problem.stateReference[k * 9] = (1.0 - alpha) * startX + alpha * goalX;
        problem.stateReference[k * 9 + 1] = (1.0 - alpha) * startY + alpha * goalY;
        problem.stateReference[k * 9 + 2] = altitude;
        problem.stateReference[k * 9 + 3] = referenceVx;
        problem.stateReference[k * 9 + 4] = referenceVy;
        if (k < horizon)
        {
            problem.controlReference[k * 4] = model.hoverThrust();
            problem.initialControls[k * 4] = orbitThrust;
            problem.initialControls[k * 4 + 3] = -angularRate;
        }
    }
    const real_t q[] = {2.0, 2.0, 3.0, 0.2, 0.2, 0.2, 0.2, 0.2, 0.1};
    const real_t qTerminal[] = {15.0, 15.0, 20.0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.2};
    const real_t r[] = {0.03, 0.2, 0.2, 0.1};
    const real_t lowerState[] = {-40.0, -40.0, 0.2, -4.0, -4.0, -4.0, -0.7, -0.7, -20.0};
    const real_t upperState[] = {40.0, 40.0, 5.0, 4.0, 4.0, 4.0, 0.7, 0.7, 20.0};
    const real_t lowerControl[] = {0.0, -1.2, -1.2, -1.2};
    const real_t upperControl[] = {20.0, 1.2, 1.2, 1.2};
    problem.stateWeights.assign(q, q + 9);
    problem.terminalWeights.assign(qTerminal, qTerminal + 9);
    problem.controlWeights.assign(r, r + 4);
    problem.stateLowerBounds.assign(lowerState, lowerState + 9);
    problem.stateUpperBounds.assign(upperState, upperState + 9);
    problem.controlLowerBounds.assign(lowerControl, lowerControl + 4);
    problem.controlUpperBounds.assign(upperControl, upperControl + 4);
}

std::string modelType(const std::string& composition, int index)
{
    if (composition != "balanced") return composition;
    if (index % 3 == 0) return "unicycle";
    if (index % 3 == 1) return "bicycle";
    return "quadcopter";
}

void addBox(
    std::vector<ConvexPolygonObstacle>& obstacles,
    real_t centerX, real_t centerY, real_t halfX, real_t halfY)
{
    ConvexPolygonObstacle obstacle;
    const real_t vertices[] = {
        centerX - halfX, centerY - halfY,
        centerX + halfX, centerY - halfY,
        centerX + halfX, centerY + halfY,
        centerX - halfX, centerY + halfY
    };
    obstacle.vertices.assign(vertices, vertices + 8);
    obstacles.push_back(obstacle);
}

BenchmarkScenario makeScenario(
    int agentCount,
    int obstacleCount,
    int seed,
    const std::string& density,
    const std::string& composition,
    const std::string& family,
    bool randomizeFormation)
{
    BenchmarkScenario scenario;
    scenario.family = family;
    scenario.density = density;
    scenario.composition = composition;
    scenario.caseId = "monte_carlo";
    scenario.difficulty = "monte_carlo";
    scenario.seed = seed;
    scenario.obstacleCount = obstacleCount;
    const real_t dt = 0.25;
    const bool localExchange = family == "local_exchange"
        || family == "local_exchange_obstacles";
    const int pairCount = std::max(1, agentCount / 2);
    const int localColumns = static_cast<int>(std::ceil(
        std::sqrt(static_cast<real_t>(pairCount))));
    const int localRows = (pairCount + localColumns - 1) / localColumns;
    const real_t localSpacing = 8.0;
    const real_t workspaceRadius = localExchange
        ? std::max(6.0, 0.5 * localSpacing * std::max(localColumns, localRows) + 2.5)
        : (density == "fixed"
            ? 6.0 : std::max(6.0,
                1.5 * std::sqrt(static_cast<real_t>(agentCount + obstacleCount))));
    const real_t routeRadius = 0.78 * workspaceRadius;
    const int_t horizon = localExchange ? 28 : std::max<int_t>(
        28,
        static_cast<int_t>(std::ceil(
            pi * routeRadius / (2.5 * dt)
        ))
    );
    std::mt19937 formationGenerator(
        static_cast<unsigned>(seed + 3571 * agentCount + 65537 * obstacleCount)
    );
    std::uniform_real_distribution<real_t> phaseDistribution(0.0, 2.0 * pi);
    std::uniform_real_distribution<real_t> jitterDistribution(
        -0.2 * pi / agentCount, 0.2 * pi / agentCount
    );
    const real_t phase = randomizeFormation
        ? phaseDistribution(formationGenerator) : 0.0;

    for (int agent = 0; agent < agentCount; ++agent)
    {
        real_t startX = 0.0, startY = 0.0, goalX = 0.0, goalY = 0.0;
        if (localExchange)
        {
            const int pair = agent / 2;
            const int column = pair % localColumns;
            const int row = pair / localColumns;
            const real_t centerX = (column - 0.5 * (localColumns - 1))
                * localSpacing;
            const real_t centerY = (row - 0.5 * (localRows - 1))
                * localSpacing;
            const real_t localJitter = randomizeFormation
                ? 0.1 * std::sin(static_cast<real_t>(seed + 17 * pair)) : 0.0;
            const real_t direction = agent % 2 == 0 ? 1.0 : -1.0;
            startX = centerX - direction * 1.5;
            goalX = centerX + direction * 1.5;
            startY = goalY = centerY + localJitter + direction * 0.15;
        }
        else
        {
            const real_t angle = 2.0 * pi * agent / agentCount + phase
                + (randomizeFormation ? jitterDistribution(formationGenerator) : 0.0);
            startX = routeRadius * std::cos(angle);
            startY = routeRadius * std::sin(angle);
            goalX = -startX;
            goalY = -startY;
        }
        const std::string type = modelType(composition, agent);
        NonlinearAgentProblem problem;
        if (type == "unicycle")
        {
            scenario.models.push_back(std::unique_ptr<NonlinearModel>(new UnicycleModel(dt)));
            fillUnicycle(problem, static_cast<const UnicycleModel&>(*scenario.models.back()),
                horizon, startX, startY, goalX, goalY, agent);
        }
        else if (type == "bicycle")
        {
            scenario.models.push_back(std::unique_ptr<NonlinearModel>(new BicycleModel(dt, 2.0)));
            fillBicycle(problem, static_cast<const BicycleModel&>(*scenario.models.back()),
                horizon, startX, startY, goalX, goalY, agent);
        }
        else
        {
            scenario.models.push_back(std::unique_ptr<NonlinearModel>(new QuadcopterModel(dt, 1.0, 9.81)));
            fillQuadcopter(problem, static_cast<const QuadcopterModel&>(*scenario.models.back()),
                horizon, startX, startY, goalX, goalY, agent);
        }
        if (localExchange)
        {
            const int_t nx = problem.model->stateDimension();
            problem.initialState.assign(
                problem.stateReference.begin(),
                problem.stateReference.begin() + nx);
            problem.initialControls = problem.controlReference;
        }
        scenario.agents.push_back(problem);
    }

    if (family == "local_exchange_obstacles")
    {
        for (int obstacle = 0; obstacle < obstacleCount; ++obstacle)
        {
            const int pair = obstacle % pairCount;
            const int layer = obstacle / pairCount;
            const int column = pair % localColumns;
            const int row = pair / localColumns;
            const real_t centerX = (column - 0.5 * (localColumns - 1))
                * localSpacing;
            const real_t centerY = (row - 0.5 * (localRows - 1))
                * localSpacing;
            if (layer < 2)
            {
                const real_t side = layer == 0 ? 1.0 : -1.0;
                addBox(scenario.obstacles, centerX, centerY + side * 1.30,
                    0.25, 0.25);
            }
            else
            {
                const real_t angle = 0.5 * pi
                    + 2.0 * pi * (layer - 2) / std::max(1, obstacleCount / pairCount);
                const real_t radius = 3.0 + 0.25 * ((layer - 2) / 8);
                addBox(scenario.obstacles,
                    centerX + radius * std::cos(angle),
                    centerY + radius * std::sin(angle),
                    0.20, 0.20);
            }
        }
        return scenario;
    }

    if (family == "single_blocker")
    {
        addBox(scenario.obstacles, 0.0, 0.0,
            0.10 * workspaceRadius, 0.09 * workspaceRadius);
        return scenario;
    }
    if (family == "doorway")
    {
        addBox(scenario.obstacles, 0.0, 0.40 * workspaceRadius,
            0.05 * workspaceRadius, 0.20 * workspaceRadius);
        addBox(scenario.obstacles, 0.0, -0.40 * workspaceRadius,
            0.05 * workspaceRadius, 0.20 * workspaceRadius);
        return scenario;
    }
    if (family == "bottleneck")
    {
        for (int obstacle = 0; obstacle < obstacleCount; ++obstacle)
        {
            const real_t fraction = (obstacle + 0.5) / obstacleCount;
            const real_t centerX = -0.42 * workspaceRadius
                + 0.84 * workspaceRadius * fraction;
            const real_t centerY = obstacle % 2 == 0
                ? 0.24 * workspaceRadius : -0.24 * workspaceRadius;
            addBox(scenario.obstacles, centerX, centerY,
                0.055 * workspaceRadius, 0.20 * workspaceRadius);
        }
        return scenario;
    }
    if (family == "warehouse_aisles")
    {
        const int columns = static_cast<int>(std::ceil(std::sqrt(
            static_cast<real_t>(obstacleCount))));
        const int rows = (obstacleCount + columns - 1) / columns;
        for (int obstacle = 0; obstacle < obstacleCount; ++obstacle)
        {
            const int column = obstacle % columns;
            const int row = obstacle / columns;
            const real_t centerX = columns == 1 ? 0.0
                : -0.40 * workspaceRadius + 0.80 * workspaceRadius * column / (columns - 1);
            const real_t centerY = rows == 1 ? 0.0
                : -0.38 * workspaceRadius + 0.76 * workspaceRadius * row / (rows - 1);
            addBox(scenario.obstacles, centerX, centerY,
                0.035 * workspaceRadius, 0.11 * workspaceRadius);
        }
        return scenario;
    }
    if (family == "multiple_homotopy_maze")
    {
        for (int obstacle = 0; obstacle < obstacleCount; ++obstacle)
        {
            const real_t fraction = (obstacle + 0.5) / obstacleCount;
            const real_t centerX = obstacle % 2 == 0
                ? -0.17 * workspaceRadius : 0.17 * workspaceRadius;
            const real_t centerY = -0.44 * workspaceRadius
                + 0.88 * workspaceRadius * fraction;
            addBox(scenario.obstacles, centerX, centerY,
                0.22 * workspaceRadius, 0.025 * workspaceRadius);
        }
        return scenario;
    }

    std::mt19937 generator(static_cast<unsigned>(seed + 7919 * obstacleCount + 104729 * agentCount));
    std::uniform_real_distribution<real_t> coordinate(-0.48 * workspaceRadius, 0.48 * workspaceRadius);
    int attempts = 0;
    while (static_cast<int>(scenario.obstacles.size()) < obstacleCount && attempts < 20000)
    {
        ++attempts;
        const real_t cx = coordinate(generator);
        const real_t cy = coordinate(generator);
        const real_t half = 0.20 + 0.05 * (scenario.obstacles.size() % 3);
        if (std::sqrt(cx * cx + cy * cy)
                + std::sqrt(2.0) * half + 0.65 >= routeRadius)
            continue;
        bool separated = true;
        for (std::size_t obstacle = 0; obstacle < scenario.obstacles.size(); ++obstacle)
        {
            const real_t oldX = 0.5 * (scenario.obstacles[obstacle].vertices[0]
                + scenario.obstacles[obstacle].vertices[4]);
            const real_t oldY = 0.5 * (scenario.obstacles[obstacle].vertices[1]
                + scenario.obstacles[obstacle].vertices[5]);
            const real_t oldHalf = 0.5 * std::fabs(
                scenario.obstacles[obstacle].vertices[2]
                - scenario.obstacles[obstacle].vertices[0]
            );
            const real_t requiredGap = half + oldHalf + 0.15;
            if (std::fabs(cx - oldX) < requiredGap
                && std::fabs(cy - oldY) < requiredGap) separated = false;
        }
        if (!separated) continue;
        addBox(scenario.obstacles, cx, cy, half, half);
    }
    return scenario;
}

std::vector<Method> methods(const std::string& requested)
{
    std::vector<Method> all;
    all.push_back(Method{"cold", NCM_DISTRIBUTED_ADMM, NCONT_COLD, true});
    all.push_back(Method{"inner", NCM_DISTRIBUTED_ADMM, NCONT_INNER_ADMM, true});
    all.push_back(Method{"qp_continuation", NCM_DISTRIBUTED_ADMM, NCONT_QP, true});
    all.push_back(Method{"full", NCM_DISTRIBUTED_ADMM, NCONT_FULL, true});
    all.push_back(Method{"centralized_qpoases", NCM_CENTRALIZED_SCP, NCONT_FULL, true});
#ifdef QPOASES_WITH_OSQP
    all.push_back(Method{"centralized_osqp", NCM_CENTRALIZED_OSQP, NCONT_FULL, true});
#endif
    if (requested == "all") return all;
    std::vector<Method> selected;
    for (std::size_t i = 0; i < all.size(); ++i)
        if (all[i].name == requested) selected.push_back(all[i]);
    return selected;
}

std::string failureCategory(
    const NonlinearTurboResult& result,
    const NonlinearValidationResult& validation)
{
    if (result.status.find("invalid") != std::string::npos) return "invalid_scenario";
    if (result.status.find("subproblem failed") != std::string::npos) return "qp_failure";
    if (!validation.success)
    {
        if (validation.status.find("pairwise") != std::string::npos) return "safety_pair";
        if (validation.status.find("obstacle") != std::string::npos) return "safety_obstacle";
        if (validation.status.find("dynamics") != std::string::npos) return "dynamics";
        if (validation.status.find("terminal") != std::string::npos) return "terminal";
        return "validation";
    }
    if (!result.converged) return "scp_limit";
    return "none";
}

NonlinearTurboOptions solverOptions(
    const Method& method, int threads, bool adaptiveRho, bool inexactAdmm)
{
    NonlinearTurboOptions options;
    options.coordinationMethod = method.coordination;
    options.continuationMode = method.continuation;
    options.useRiccatiWarmStart = method.riccati;
    options.parallelAgentSolves = true;
    options.parallelAgentThreads = threads;
    options.collisionSamplesPerInterval = 20;
    options.maxScpIterations = 30;
    options.maxAdmmIterations = 50;
    options.maxWorkingSetRecalculations = 400;
    options.rho = 35.0;
    options.adaptiveRho = adaptiveRho;
    options.adaptiveRhoInterval = 5;
    options.adaptiveRhoImbalance = 10.0;
    options.adaptiveRhoScale = 2.0;
    options.minimumRho = 5.0;
    options.maximumRho = 140.0;
    options.inexactAdmmScpIterations = inexactAdmm ? 2 : 0;
    options.inexactAdmmToleranceMultiplier = inexactAdmm ? 5.0 : 1.0;
    options.polishingAdmmIterations = 200;
    options.controlTrustRegion = 1.0;
    options.admmPrimalTolerance = 1.0e-3;
    options.admmDualTolerance = 1.0e-2;
    options.admmRelativeTolerance = 1.0e-3;
    options.admmRelaxation = 1.6;
    options.terminalPositionTolerance = 2.5;
    options.meritPenalty = 1.0e7;
    return options;
}

void writeHeader(std::ofstream& output)
{
    output << "suite,case_id,difficulty,family,density,composition,seed,n,m,feasibility_witness,method,"
        "solver_success,validator_success,protocol_success,converged,status,failure_category,"
        "solve_time_ms,max_local_qp_time_ms,objective,minimum_pairwise_clearance,"
        "minimum_obstacle_clearance,maximum_dynamics_defect,maximum_terminal_error,"
        "scp_iterations,admm_iterations,qp_solves,qp_working_set_recalculations,backend_iterations,"
        "last_qp_status,failed_agent,primal_residual,dual_residual,"
        "collision_samples_per_interval,max_scp_iterations,max_admm_iterations,"
        "polishing_admm_iterations,merit_penalty,rho,adaptive_rho,adaptive_rho_active,"
        "rho_updates,minimum_admm_rho,maximum_admm_rho,final_admm_rho,"
        "restoration_attempts,successful_restorations,polishing_scp_iterations,"
        "maximum_restoration_slack,final_restoration_slack,max_restoration_attempts,"
        "restoration_trust_region_shrink,minimum_control_trust_region,"
        "restoration_slack_penalty,restoration_slack_tolerance,"
        "inexact_admm_scp_iterations,inexact_admm_tolerance_multiplier,"
        "cold_starts,matrix_hotstarts,vector_hotstarts,transported_pair_stages,"
        "reset_pair_stages,parallel_qp_batches,parallel_agent_threads,admm_relative_tolerance,admm_relaxation,"
        "pair_activation_distance,obstacle_activation_distance,"
        "primal_stopping_threshold,dual_stopping_threshold,"
        "admm_converged_subproblems,admm_iteration_limit_subproblems,"
        "maximum_active_pairs,initial_active_pairs,final_active_pairs,maximum_potential_pairs,maximum_agent_degree,"
        "maximum_active_obstacles_per_agent,maximum_potential_obstacles_per_agent,maximum_local_qp_variables,"
        "maximum_local_qp_constraints,centralized_qp_variables,centralized_qp_constraints,"
        "qp_build_time_ms,pair_build_time_ms,admm_assembly_time_ms,"
        "local_qp_batch_time_ms,local_qp_solve_time_ms,consensus_time_ms,"
        "globalization_time_ms\n";
}

void writeScpHeader(std::ofstream& output)
{
    output << "case_id,method,iteration,qp_solved,qp_status,failed_agent,"
        "restoration_attempts,maximum_restoration_slack,"
        "admm_converged,restoration_used,polishing,step_accepted,"
        "admm_iterations,rho_updates,"
        "objective,merit,primal_residual,dual_residual,rho,"
        "control_trust_region,step_length,maximum_control_step,minimum_pairwise_clearance,"
        "minimum_obstacle_clearance,maximum_terminal_error\n";
}

bool runCase(
    const std::string& suite,
    const BenchmarkScenario& scenario,
    const Method& method,
    int threads,
    bool adaptiveRho,
    bool inexactAdmm,
    bool requireConvergence,
    std::ofstream& output,
    std::ofstream& traceOutput)
{
    NonlinearTurboOptions options = solverOptions(
        method, threads, adaptiveRho, inexactAdmm);
    if (scenario.density == "constant_density")
    {
        options.pairActivationDistance = 2.0;
        options.obstacleActivationDistance = 2.0;
    }
    const NonlinearTurboResult result = NonlinearTurboADMM().solve(
        scenario.agents, scenario.obstacles, options
    );
    for (std::size_t iteration = 0; iteration < result.scpTrace.size(); ++iteration)
    {
        const NonlinearScpIterationStatistics& trace = result.scpTrace[iteration];
        traceOutput << scenario.caseId << ',' << method.name << ','
            << trace.iteration << ',' << (trace.qpSolved ? 1 : 0) << ','
            << trace.qpStatus << ',' << trace.failedAgent << ','
            << trace.restorationAttempts << ',' << trace.maximumRestorationSlack << ','
            << (trace.admmConverged ? 1 : 0) << ','
            << (trace.restorationUsed ? 1 : 0) << ','
            << (trace.polishing ? 1 : 0) << ','
            << (trace.stepAccepted ? 1 : 0) << ',' << trace.admmIterations << ','
            << trace.rhoUpdates << ',' << std::setprecision(12) << trace.objective << ','
            << trace.merit << ',' << trace.primalResidual << ',' << trace.dualResidual << ','
            << trace.rho << ',' << trace.controlTrustRegion << ',' << trace.stepLength << ','
            << trace.maximumControlStep << ',' << trace.minimumPairwiseClearance << ','
            << trace.minimumObstacleClearance << ',' << trace.maximumTerminalPositionError << '\n';
    }
    traceOutput.flush();
    NonlinearValidationOptions validationOptions;
    validationOptions.interpolationSubsteps = 20;
    validationOptions.terminalPositionTolerance = 2.5;
    const NonlinearValidationResult validation = validateNonlinearTrajectories(
        scenario.agents,
        scenario.obstacles,
        result.trajectories,
        options,
        validationOptions
    );
    output << suite << ',' << scenario.caseId << ',' << scenario.difficulty << ','
        << scenario.family << ',' << scenario.density << ',' << scenario.composition << ','
        << scenario.seed << ','
        << scenario.agents.size() << ',' << scenario.obstacleCount << ','
        << (scenario.family.find("local_exchange") == 0
            ? "local_lanes" : "outer_ring") << ',' << method.name << ','
        << (result.success ? 1 : 0) << ',' << (validation.success ? 1 : 0) << ','
        << (result.success && validation.success ? 1 : 0) << ','
        << (result.converged ? 1 : 0) << ',' << csvText(result.status) << ','
        << failureCategory(result, validation) << ',' << std::setprecision(12)
        << result.statistics.solveTimeMilliseconds << ','
        << result.statistics.maximumLocalQpSolveTimeMilliseconds << ','
        << result.statistics.objective << ','
        << validation.minimumPairwiseClearance << ','
        << validation.minimumObstacleClearance << ','
        << validation.maximumDynamicsDefect << ','
        << validation.maximumTerminalPositionError << ','
        << result.statistics.scpIterations << ',' << result.statistics.admmIterations << ','
        << result.statistics.qpSolves << ','
        << result.statistics.qpWorkingSetRecalculations << ','
        << result.statistics.backendIterations << ','
        << result.statistics.lastQpStatus << ',' << result.statistics.failedAgent << ','
        << result.statistics.primalResidual << ',' << result.statistics.dualResidual << ','
        << options.collisionSamplesPerInterval << ',' << options.maxScpIterations << ','
        << options.maxAdmmIterations << ',' << options.polishingAdmmIterations << ','
        << options.meritPenalty << ','
        << options.rho << ',' << (options.adaptiveRho ? 1 : 0) << ','
        << (options.adaptiveRho ? 1 : 0) << ','
        << result.statistics.rhoUpdates << ','
        << result.statistics.minimumAdmmRho << ','
        << result.statistics.maximumAdmmRho << ','
        << result.statistics.finalAdmmRho << ','
        << result.statistics.restorationAttempts << ','
        << result.statistics.successfulRestorations << ','
        << result.statistics.polishingScpIterations << ','
        << result.statistics.maximumRestorationSlack << ','
        << result.statistics.finalRestorationSlack << ','
        << options.maxRestorationAttempts << ','
        << options.restorationTrustRegionShrink << ','
        << options.minimumControlTrustRegion << ','
        << options.restorationSlackPenalty << ','
        << options.restorationSlackTolerance << ','
        << options.inexactAdmmScpIterations << ','
        << options.inexactAdmmToleranceMultiplier << ','
        << result.statistics.coldStarts << ',' << result.statistics.matrixHotstarts << ','
        << result.statistics.vectorHotstarts << ','
        << result.statistics.transportedPairStages << ','
        << result.statistics.resetPairStages << ','
        << result.statistics.parallelQpBatches << ','
        << result.statistics.parallelAgentThreads << ','
        << options.admmRelativeTolerance << ','
        << options.admmRelaxation << ','
        << options.pairActivationDistance << ','
        << options.obstacleActivationDistance << ','
        << result.statistics.primalStoppingThreshold << ','
        << result.statistics.dualStoppingThreshold << ','
        << result.statistics.admmConvergedSubproblems << ','
        << result.statistics.admmIterationLimitSubproblems << ','
        << result.statistics.maximumActivePairs << ','
        << result.statistics.initialActivePairs << ','
        << result.statistics.finalActivePairs << ','
        << result.statistics.maximumPotentialPairs << ','
        << result.statistics.maximumAgentDegree << ','
        << result.statistics.maximumActiveObstaclesPerAgent << ','
        << result.statistics.maximumPotentialObstaclesPerAgent << ','
        << result.statistics.maximumLocalQpVariables << ','
        << result.statistics.maximumLocalQpConstraints << ','
        << result.statistics.centralizedQpVariables << ','
        << result.statistics.centralizedQpConstraints << ','
        << result.statistics.qpBuildTimeMilliseconds << ','
        << result.statistics.pairBuildTimeMilliseconds << ','
        << result.statistics.admmAssemblyTimeMilliseconds << ','
        << result.statistics.localQpBatchTimeMilliseconds << ','
        << result.statistics.localQpSolveTimeMilliseconds << ','
        << result.statistics.consensusTimeMilliseconds << ','
        << result.statistics.globalizationTimeMilliseconds << '\n';
    output.flush();
    std::printf("%s n=%u m=%d seed=%d %s: solver=%d validator=%d %.2f ms\n",
        scenario.composition.c_str(), static_cast<unsigned>(scenario.agents.size()),
        scenario.obstacleCount, scenario.seed, method.name.c_str(),
        result.success ? 1 : 0, validation.success ? 1 : 0,
        result.statistics.solveTimeMilliseconds);
    return result.success && validation.success
        && (!requireConvergence || result.converged);
}

}

int main(int argc, char** argv)
{
    Arguments arguments;
    if (!parseArguments(argc, argv, arguments))
    {
        std::fprintf(stderr,
            "usage: %s [--suite manual|ci|smoke|development|final] [--case id|all] "
            "[--scenario-index nonnegative_integer] [--track all|scaling|models|families] [--output file.csv] "
            "[--method all|cold|inner|qp_continuation|full|centralized_qpoases|centralized_osqp] "
            "[--threads positive_integer] [--fixed-rho] [--exact-admm] "
            "[--dry-run] [--require-success] [--require-convergence]\n",
            argv[0]);
        return 2;
    }
    const std::vector<Method> selectedMethods = methods(arguments.method);
    if (selectedMethods.empty())
    {
        std::fprintf(stderr, "unknown method: %s\n", arguments.method.c_str());
        return 2;
    }
    std::ofstream output(arguments.output.c_str());
    const std::string tracePath = arguments.output + ".scp.csv";
    std::ofstream traceOutput(tracePath.c_str());
    if (!output || !traceOutput)
    {
        std::fprintf(stderr, "cannot open benchmark output: %s\n", arguments.output.c_str());
        return 2;
    }
    writeHeader(output);
    writeScpHeader(traceOutput);
    const std::vector<int> ns = agentCounts(arguments.suite);
    const std::vector<int> ms = obstacleCounts(arguments.suite);
    const std::vector<int> suiteSeeds = seeds(arguments.suite);
    const std::vector<std::string> suiteCompositions = compositions(arguments.suite);
    if (arguments.suite == "manual")
    {
        const std::vector<ManualCase> cases = manualCases();
        bool allSuccessful = true;
        std::size_t generated = 0;
        for (std::size_t index = 0; index < cases.size(); ++index)
        {
            if (arguments.scenarioIndex >= 0 && static_cast<int>(index) != arguments.scenarioIndex)
                continue;
            if (arguments.caseId != "all" && arguments.caseId != cases[index].id)
                continue;
            BenchmarkScenario scenario = makeScenario(
                cases[index].agentCount, cases[index].obstacleCount, 0, "fixed",
                cases[index].composition, cases[index].family, false
            );
            scenario.caseId = cases[index].id;
            scenario.difficulty = cases[index].difficulty;
            if (static_cast<int>(scenario.obstacles.size()) != cases[index].obstacleCount)
            {
                std::fprintf(stderr, "manual case generated the wrong obstacle count\n");
                return 1;
            }
            ++generated;
            if (arguments.dryRun) continue;
            for (std::size_t method = 0; method < selectedMethods.size(); ++method)
                allSuccessful = runCase(
                    arguments.suite, scenario, selectedMethods[method], arguments.threads,
                    !arguments.fixedRho, !arguments.exactAdmm,
                    arguments.requireConvergence, output, traceOutput
                ) && allSuccessful;
        }
        if (generated == 0)
        {
            std::fprintf(stderr, "manual case selection matched no scenario\n");
            return 2;
        }
        if (arguments.dryRun)
            std::printf("generated %u manual scenarios\n", static_cast<unsigned>(generated));
        return arguments.requireSuccess && !allSuccessful ? 1 : 0;
    }

    const std::vector<std::string> densities = densityRegimes(arguments.suite);
    std::size_t scenarioCount = 0;
    std::size_t candidateIndex = 0;
    bool allSuccessful = true;
    for (std::size_t density = 0; density < densities.size(); ++density)
    for (std::size_t composition = 0; composition < suiteCompositions.size(); ++composition)
    for (std::size_t ni = 0; ni < ns.size(); ++ni)
    for (std::size_t mi = 0; mi < ms.size(); ++mi)
    {
        const std::vector<std::string> families = scenarioFamilies(
            arguments.suite, densities[density], ms[mi]);
        for (std::size_t family = 0; family < families.size(); ++family)
        {
            if (!includedInTrack(arguments, ns[ni], ms[mi], densities[density],
                    suiteCompositions[composition], families[family])) continue;
            for (std::size_t si = 0; si < suiteSeeds.size(); ++si)
            {
                const std::size_t currentIndex = candidateIndex++;
                if (arguments.scenarioIndex >= 0
                    && static_cast<int>(currentIndex) != arguments.scenarioIndex)
                    continue;
                BenchmarkScenario scenario = makeScenario(
                    ns[ni], ms[mi], suiteSeeds[si], densities[density],
                    suiteCompositions[composition], families[family], true
                );
                if (static_cast<int>(scenario.obstacles.size()) != ms[mi])
                {
                    std::fprintf(stderr, "failed to generate requested obstacle count\n");
                    return 1;
                }
                ++scenarioCount;
                if (arguments.dryRun) continue;
                for (std::size_t method = 0; method < selectedMethods.size(); ++method)
                    allSuccessful = runCase(
                        arguments.suite, scenario, selectedMethods[method], arguments.threads,
                        !arguments.fixedRho, !arguments.exactAdmm,
                        arguments.requireConvergence, output, traceOutput
                    ) && allSuccessful;
            }
        }
    }
    if (arguments.scenarioIndex >= 0 && scenarioCount == 0)
    {
        std::fprintf(stderr, "scenario index is outside the selected suite and track\n");
        return 2;
    }
    if (arguments.dryRun)
        std::printf("generated %u scenarios\n", static_cast<unsigned>(scenarioCount));
    return arguments.requireSuccess && !allSuccessful ? 1 : 0;
}
