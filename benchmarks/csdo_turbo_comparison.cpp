#include <qpOASES/Constants.hpp>
#include <qpOASES/NonlinearTrajectoryValidator.hpp>

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

USING_NAMESPACE_QPOASES

namespace
{

const real_t pi = 3.14159265358979323846;
const real_t csdoDegreeToRadian = 3.14 / 180.0;

struct Arguments
{
    std::string input;
    std::string guess;
    std::string output;
    int_t threads;
    Arguments() : threads(0) {}
};

struct CsdoGeometry
{
    real_t mapWidth;
    real_t mapHeight;
    std::vector<real_t> obstacleX;
    std::vector<real_t> obstacleY;
    std::vector<real_t> obstacleRadius;
};

Arguments parseArguments(int argc, char* argv[])
{
    Arguments arguments;
    for (int index = 1; index < argc; ++index)
    {
        const std::string option(argv[index]);
        if ((option == "--input" || option == "-i") && index + 1 < argc)
            arguments.input = argv[++index];
        else if ((option == "--guess" || option == "-g") && index + 1 < argc)
            arguments.guess = argv[++index];
        else if ((option == "--output" || option == "-o") && index + 1 < argc)
            arguments.output = argv[++index];
        else if (option == "--threads" && index + 1 < argc)
            arguments.threads = static_cast<int_t>(std::atoi(argv[++index]));
        else
            throw std::runtime_error("unknown or incomplete argument: " + option);
    }
    if (arguments.input.empty() || arguments.guess.empty()
        || arguments.output.empty())
        throw std::runtime_error(
            "usage: csdo_turbo_comparison --input INSTANCE "
            "--guess CSDO_GUESSES --output RESULT [--threads N]");
    if (arguments.threads < 0)
        throw std::runtime_error("--threads must be nonnegative");
    return arguments;
}

ConvexPolygonObstacle rectangle(
    real_t minimumX,
    real_t minimumY,
    real_t maximumX,
    real_t maximumY)
{
    ConvexPolygonObstacle obstacle;
    obstacle.vertices.push_back(minimumX);
    obstacle.vertices.push_back(minimumY);
    obstacle.vertices.push_back(maximumX);
    obstacle.vertices.push_back(minimumY);
    obstacle.vertices.push_back(maximumX);
    obstacle.vertices.push_back(maximumY);
    obstacle.vertices.push_back(minimumX);
    obstacle.vertices.push_back(maximumY);
    return obstacle;
}

ConvexPolygonObstacle circumscribedCircle(
    real_t centerX,
    real_t centerY,
    real_t radius,
    int_t sides)
{
    ConvexPolygonObstacle obstacle;
    const real_t vertexRadius = radius / std::cos(pi / sides);
    for (int_t vertex = 0; vertex < sides; ++vertex)
    {
        const real_t angle = 2.0 * pi * vertex / sides + pi / sides;
        obstacle.vertices.push_back(centerX + vertexRadius * std::cos(angle));
        obstacle.vertices.push_back(centerY + vertexRadius * std::sin(angle));
    }
    return obstacle;
}

CsdoGeometry readGeometry(
    const YAML::Node& input,
    std::vector<ConvexPolygonObstacle>& polygons)
{
    CsdoGeometry geometry;
    geometry.mapWidth = input["map"]["dimensions"][0].as<real_t>();
    geometry.mapHeight = input["map"]["dimensions"][1].as<real_t>();
    const YAML::Node obstacleNodes = input["map"]["obstacles"];
    for (std::size_t obstacle = 0; obstacle < obstacleNodes.size(); ++obstacle)
    {
        const real_t x = obstacleNodes[obstacle][0].as<real_t>();
        const real_t y = obstacleNodes[obstacle][1].as<real_t>();
        const real_t radius = obstacleNodes[obstacle][2].as<real_t>();
        geometry.obstacleX.push_back(x);
        geometry.obstacleY.push_back(y);
        geometry.obstacleRadius.push_back(radius);
        polygons.push_back(circumscribedCircle(x, y, radius, 32));
    }

    const real_t extent = 2.0 * std::max(
        geometry.mapWidth, geometry.mapHeight) + 10.0;
    polygons.push_back(rectangle(
        -extent, -extent, 0.0, geometry.mapHeight + extent));
    polygons.push_back(rectangle(
        geometry.mapWidth, -extent,
        geometry.mapWidth + extent, geometry.mapHeight + extent));
    polygons.push_back(rectangle(
        0.0, -extent, geometry.mapWidth, 0.0));
    polygons.push_back(rectangle(
        0.0, geometry.mapHeight,
        geometry.mapWidth, geometry.mapHeight + extent));
    return geometry;
}

std::vector<std::string> agentNames(const YAML::Node& input)
{
    std::vector<std::string> names;
    const YAML::Node agents = input["agents"];
    for (std::size_t agent = 0; agent < agents.size(); ++agent)
    {
        if (agents[agent]["name"])
            names.push_back(agents[agent]["name"].as<std::string>());
        else
            names.push_back("agent" + std::to_string(agent));
    }
    return names;
}

std::vector<NonlinearAgentProblem> readAgents(
    const YAML::Node& input,
    const YAML::Node& guess,
    const FrontSteeringModel& model)
{
    const std::vector<std::string> names = agentNames(input);
    const YAML::Node schedule = guess["schedule"];
    if (!schedule || names.empty())
        throw std::runtime_error("CSDO guess has no schedule");

    const std::size_t stateCount = schedule[names[0]].size();
    if (stateCount < 2)
        throw std::runtime_error("CSDO guess horizon is empty");
    const int_t horizon = static_cast<int_t>(stateCount - 1);
    std::vector<NonlinearAgentProblem> agents(names.size());
    for (std::size_t agent = 0; agent < names.size(); ++agent)
    {
        const YAML::Node nodes = schedule[names[agent]];
        if (nodes.size() != stateCount)
            throw std::runtime_error("CSDO guesses require a common horizon");
        NonlinearAgentProblem& problem = agents[agent];
        problem.model = &model;
        problem.horizon = horizon;
        problem.stateReference.assign((horizon + 1) * 4, 0.0);
        problem.controlReference.assign(horizon * 2, 0.0);
        problem.initialControls.assign(horizon * 2, 0.0);
        for (int_t stage = 0; stage <= horizon; ++stage)
        {
            problem.stateReference[stage * 4] =
                nodes[stage]["x"].as<real_t>();
            problem.stateReference[stage * 4 + 1] =
                nodes[stage]["y"].as<real_t>();
            problem.stateReference[stage * 4 + 2] =
                nodes[stage]["yaw"].as<real_t>();
            problem.stateReference[stage * 4 + 3] =
                nodes[stage]["steer"].as<real_t>() * csdoDegreeToRadian;
            if (stage < horizon)
            {
                problem.initialControls[stage * 2] =
                    nodes[stage]["v"].as<real_t>();
                problem.initialControls[stage * 2 + 1] =
                    nodes[stage]["omega"].as<real_t>()
                    * csdoDegreeToRadian;
            }
        }
        problem.initialState.assign(
            problem.stateReference.begin(),
            problem.stateReference.begin() + 4
        );
        problem.controlReference = problem.initialControls;
        problem.stateWeights.assign(4, 0.0);
        problem.terminalWeights.assign(4, 0.0);
        problem.controlWeights.assign(2, 0.0);
        problem.controlWeights[1] = 1.0;
        problem.controlDifferenceWeights.assign(2, 0.0);
        problem.controlDifferenceWeights[0] = 1.0;
        problem.stateLowerBounds.assign(4, -INFTY);
        problem.stateUpperBounds.assign(4, INFTY);
        const real_t maximumSteering = std::atan(1.0 / 3.0);
        problem.stateLowerBounds[3] = -maximumSteering;
        problem.stateUpperBounds[3] = maximumSteering;
        problem.controlLowerBounds.push_back(-1.0);
        problem.controlLowerBounds.push_back(-0.07);
        problem.controlUpperBounds.push_back(1.0);
        problem.controlUpperBounds.push_back(0.07);
        problem.collisionCircles.push_back(
            CollisionCircle(1.25, 0.0, 1.25));
        problem.collisionCircles.push_back(
            CollisionCircle(-0.25, 0.0, 1.25));
        problem.obstacleSafetyDistance = 0.0;
        problem.terminalPositionTolerance = 1.0e-3;
        problem.enforceTerminalState = true;
        problem.terminalStateConstraintMask.push_back(true);
        problem.terminalStateConstraintMask.push_back(true);
        problem.terminalStateConstraintMask.push_back(true);
        problem.terminalStateConstraintMask.push_back(false);
    }
    return agents;
}

real_t csdoObjective(const std::vector<NonlinearTrajectory>& trajectories)
{
    real_t objective = 0.0;
    for (std::size_t agent = 0; agent < trajectories.size(); ++agent)
    {
        const std::vector<real_t>& controls = trajectories[agent].controls;
        const int_t horizon = static_cast<int_t>(controls.size() / 2);
        for (int_t stage = 0; stage < horizon; ++stage)
            objective += controls[stage * 2 + 1] * controls[stage * 2 + 1];
        for (int_t stage = 0; stage + 1 < horizon; ++stage)
        {
            const real_t difference = controls[(stage + 1) * 2]
                - controls[stage * 2];
            objective += difference * difference;
        }
    }
    return objective;
}

real_t exactCircularObstacleClearance(
    const std::vector<NonlinearAgentProblem>& agents,
    const std::vector<NonlinearTrajectory>& trajectories,
    const CsdoGeometry& geometry,
    int_t interpolationSubsteps)
{
    real_t minimum = INFTY;
    for (std::size_t agent = 0; agent < agents.size(); ++agent)
    {
        const int_t horizon = agents[agent].horizon;
        for (int_t stage = 0; stage < horizon; ++stage)
        for (int_t substep = 0; substep <= interpolationSubsteps; ++substep)
        {
            const real_t alpha = static_cast<real_t>(substep)
                / interpolationSubsteps;
            for (std::size_t circle = 0;
                 circle < agents[agent].collisionCircles.size();
                 ++circle)
            {
                real_t first[2], second[2];
                const CollisionCircle& footprint =
                    agents[agent].collisionCircles[circle];
                agents[agent].model->collisionPoint(
                    &trajectories[agent].states[stage * 4],
                    footprint.longitudinalOffset,
                    footprint.lateralOffset,
                    first
                );
                agents[agent].model->collisionPoint(
                    &trajectories[agent].states[(stage + 1) * 4],
                    footprint.longitudinalOffset,
                    footprint.lateralOffset,
                    second
                );
                const real_t x = first[0] + alpha * (second[0] - first[0]);
                const real_t y = first[1] + alpha * (second[1] - first[1]);
                minimum = std::min(minimum, x - footprint.radius);
                minimum = std::min(
                    minimum, geometry.mapWidth - x - footprint.radius);
                minimum = std::min(minimum, y - footprint.radius);
                minimum = std::min(
                    minimum, geometry.mapHeight - y - footprint.radius);
                for (std::size_t obstacle = 0;
                     obstacle < geometry.obstacleX.size();
                     ++obstacle)
                {
                    const real_t dx = x - geometry.obstacleX[obstacle];
                    const real_t dy = y - geometry.obstacleY[obstacle];
                    minimum = std::min(
                        minimum,
                        std::sqrt(dx * dx + dy * dy)
                            - footprint.radius
                            - geometry.obstacleRadius[obstacle]
                    );
                }
            }
        }
    }
    return minimum;
}

void writeResult(
    const std::string& filename,
    const std::vector<std::string>& names,
    const NonlinearTurboResult& result,
    const NonlinearValidationResult& validation,
    real_t objective,
    real_t exactObstacleClearance)
{
    YAML::Emitter output;
    output << YAML::BeginMap;
    output << YAML::Key << "statistics" << YAML::Value << YAML::BeginMap;
    output << YAML::Key << "success" << YAML::Value << result.success;
    output << YAML::Key << "converged" << YAML::Value << result.converged;
    output << YAML::Key << "status" << YAML::Value << result.status;
    output << YAML::Key << "validated" << YAML::Value << validation.success;
    output << YAML::Key << "validation_status" << YAML::Value
        << validation.status;
    output << YAML::Key << "solver_time" << YAML::Value
        << result.statistics.solveTimeMilliseconds / 1000.0;
    output << YAML::Key << "scp_iterations" << YAML::Value
        << result.statistics.scpIterations;
    output << YAML::Key << "admm_iterations" << YAML::Value
        << result.statistics.admmIterations;
    output << YAML::Key << "objective" << YAML::Value << objective;
    output << YAML::Key << "minimum_pairwise_clearance" << YAML::Value
        << validation.minimumPairwiseClearance;
    output << YAML::Key << "minimum_polygon_obstacle_clearance" << YAML::Value
        << validation.minimumObstacleClearance;
    output << YAML::Key << "minimum_exact_obstacle_clearance" << YAML::Value
        << exactObstacleClearance;
    output << YAML::Key << "maximum_dynamics_defect" << YAML::Value
        << validation.maximumDynamicsDefect;
    output << YAML::Key << "maximum_terminal_position_error" << YAML::Value
        << validation.maximumTerminalPositionError;
    output << YAML::Key << "parallel_threads" << YAML::Value
        << result.statistics.parallelAgentThreads;
    output << YAML::EndMap;
    output << YAML::Key << "schedule" << YAML::Value << YAML::BeginMap;
    for (std::size_t agent = 0;
         agent < result.trajectories.size();
         ++agent)
    {
        output << YAML::Key << names[agent] << YAML::Value << YAML::BeginSeq;
        const NonlinearTrajectory& trajectory = result.trajectories[agent];
        const int_t horizon =
            static_cast<int_t>(trajectory.controls.size() / 2);
        for (int_t stage = 0; stage <= horizon; ++stage)
        {
            output << YAML::BeginMap;
            output << YAML::Key << "x" << YAML::Value
                << trajectory.states[stage * 4];
            output << YAML::Key << "y" << YAML::Value
                << trajectory.states[stage * 4 + 1];
            output << YAML::Key << "yaw" << YAML::Value
                << trajectory.states[stage * 4 + 2];
            output << YAML::Key << "steer" << YAML::Value
                << trajectory.states[stage * 4 + 3] / csdoDegreeToRadian;
            output << YAML::Key << "t" << YAML::Value << stage;
            if (stage < horizon)
            {
                output << YAML::Key << "v" << YAML::Value
                    << trajectory.controls[stage * 2];
                output << YAML::Key << "omega" << YAML::Value
                    << trajectory.controls[stage * 2 + 1]
                        / csdoDegreeToRadian;
            }
            output << YAML::EndMap;
        }
        output << YAML::EndSeq;
    }
    output << YAML::EndMap << YAML::EndMap;
    std::ofstream stream(filename.c_str());
    if (!stream) throw std::runtime_error("cannot open output file");
    stream << output.c_str() << "\n";
}

}

int main(int argc, char* argv[])
{
    try
    {
        const Arguments arguments = parseArguments(argc, argv);
        const YAML::Node input = YAML::LoadFile(arguments.input);
        const YAML::Node guess = YAML::LoadFile(arguments.guess);
        std::vector<ConvexPolygonObstacle> obstacles;
        const CsdoGeometry geometry = readGeometry(input, obstacles);
        const real_t timeStep = 3.0 * 0.706 / 1.0 / 3.0 / 0.8;
        FrontSteeringModel model(timeStep, 1.0);
        const std::vector<NonlinearAgentProblem> agents =
            readAgents(input, guess, model);

        NonlinearTurboOptions options;
        options.coordinationMethod = NCM_DISTRIBUTED_ADMM;
        options.continuationMode = NCONT_FULL;
        options.useRiccatiWarmStart = false;
        options.parallelAgentSolves = true;
        options.parallelAgentThreads = arguments.threads;
        options.collisionSamplesPerInterval = 1;
        options.maxScpIterations = 20;
        options.maxAdmmIterations = 100;
        options.polishingAdmmIterations = 300;
        options.adaptiveRho = true;
        options.controlTrustRegion = 2.0;
        options.obstacleSafetyDistance = 0.0;
        options.pairActivationDistance = 2.0;
        options.obstacleActivationDistance = 2.0;
        options.collisionTolerance = 1.0e-3;
        options.terminalPositionTolerance = 1.0e-3;

        NonlinearTurboADMM solver;
        const NonlinearTurboResult result =
            solver.solve(agents, obstacles, options);
        NonlinearValidationOptions validationOptions;
        validationOptions.interpolationSubsteps = 10;
        validationOptions.dynamicsTolerance = 1.0e-7;
        validationOptions.terminalPositionTolerance = 1.0e-3;
        NonlinearValidationResult validation;
        real_t objective = INFTY;
        real_t exactObstacleClearance = INFTY;
        if (result.trajectories.size() == agents.size())
        {
            validation = validateNonlinearTrajectories(
                agents, obstacles, result.trajectories,
                options, validationOptions);
            objective = csdoObjective(result.trajectories);
            exactObstacleClearance = exactCircularObstacleClearance(
                agents, result.trajectories, geometry,
                validationOptions.interpolationSubsteps);
        }
        else
            validation.status = "solver returned no complete trajectory set";
        writeResult(
            arguments.output,
            agentNames(input),
            result,
            validation,
            objective,
            exactObstacleClearance
        );
        std::cout << "TurboADMM-NL: " << result.status
            << ", solver " << result.statistics.solveTimeMilliseconds / 1000.0
            << " s, validation " << validation.status << std::endl;
        return result.success && validation.success ? 0 : 2;
    }
    catch (const std::exception& error)
    {
        std::cerr << "csdo_turbo_comparison: " << error.what() << std::endl;
        return 1;
    }
}
