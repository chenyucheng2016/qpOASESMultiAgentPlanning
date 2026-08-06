#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "common/motion_planning.h"
#include "hybrid_a_star/Instance.h"
#include "pbs/PBS.h"
#include "sqp/corridor.h"
#include "sqp/inter_agent_cons.h"

using libMultiRobotPlanning::Corridor;
using libMultiRobotPlanning::OptimizeResult;
using libMultiRobotPlanning::QpParm;
using libMultiRobotPlanning::SolutionStatistics;

namespace
{
struct Arguments
{
    std::string input;
    std::string config;
    std::string guess;
    std::string corridor;
    std::string metadata;
    double timeLimit;
    int screen;
    int padStages;
    int delayFromAgent;
    int delayStages;
    int sequentialDelayStages;
    bool root;
    bool independent;
    Arguments() : timeLimit(7200.0), screen(0), padStages(0),
        delayFromAgent(-1), delayStages(0), sequentialDelayStages(0), root(false), independent(false) {}
};

Arguments parseArguments(int argc, char* argv[])
{
    Arguments arguments;
    for (int index = 1; index < argc; ++index)
    {
        const std::string option(argv[index]);
        if (option == "--input" && index + 1 < argc)
            arguments.input = argv[++index];
        else if (option == "--config" && index + 1 < argc)
            arguments.config = argv[++index];
        else if (option == "--guess" && index + 1 < argc)
            arguments.guess = argv[++index];
        else if (option == "--corridor" && index + 1 < argc)
            arguments.corridor = argv[++index];
        else if (option == "--metadata" && index + 1 < argc)
            arguments.metadata = argv[++index];
        else if (option == "--time-limit" && index + 1 < argc)
            arguments.timeLimit = std::stod(argv[++index]);
        else if (option == "--screen" && index + 1 < argc)
            arguments.screen = std::stoi(argv[++index]);
        else if (option == "--pad-stages" && index + 1 < argc)
            arguments.padStages = std::stoi(argv[++index]);
        else if (option == "--delay-from-agent" && index + 1 < argc)
            arguments.delayFromAgent = std::stoi(argv[++index]);
        else if (option == "--delay-stages" && index + 1 < argc)
            arguments.delayStages = std::stoi(argv[++index]);
        else if (option == "--sequential-delay-stages" && index + 1 < argc)
            arguments.sequentialDelayStages = std::stoi(argv[++index]);
        else if (option == "--independent")
            arguments.independent = true;
        else if (option == "--root")
            arguments.root = true;
        else
            throw std::runtime_error("unknown or incomplete argument: " + option);
    }
    if (arguments.input.empty() || arguments.config.empty()
        || arguments.guess.empty() || arguments.corridor.empty()
        || arguments.metadata.empty())
        throw std::runtime_error(
            "usage: csdo_root_warmstart_exporter --input INSTANCE "
            "--config CSDO_CONFIG --guess GUESS --corridor CORRIDOR "
            "--metadata METADATA [--time-limit SECONDS] [--screen LEVEL] "
            "[--pad-stages N] [--delay-from-agent I --delay-stages N] [--sequential-delay-stages N] [--independent|--root]"
        );
    if (!(arguments.timeLimit > 0.0) || arguments.screen < 0
        || arguments.padStages < 0 || arguments.delayStages < 0
        || arguments.sequentialDelayStages < 0
        || (arguments.delayStages > 0 && arguments.delayFromAgent < 0)
        || (arguments.delayStages > 0 && arguments.sequentialDelayStages > 0))
        throw std::runtime_error("invalid time limit or screen level");
    return arguments;
}

QpParm readParameters(const std::string& filename)
{
    const YAML::Node config = YAML::LoadFile(filename);
    QpParm parameters;
    parameters.max_omega = config["max_omega"].as<double>();
    parameters.max_v = config["max_v"].as<double>();
    parameters.max_iter = config["max_iter"].as<double>();
    parameters.delta_solution_threshold =
        config["delta_solution_threshold"].as<double>();
    parameters.max_violation = config["max_violation"].as<double>();
    parameters.osqp_max_iter = config["osqp_max_iter"].as<int>();
    parameters.r_trust = config["r_trust"].as<double>();
    parameters.num_interpolation = config["num_interpolation"].as<int>();
    parameters.fixed_corridor = config["fixed_corridor"].as<bool>();
    const double deceleration = config["decelerate_factor"].as<double>();
    parameters.dt = Constants::r * Constants::deltat / parameters.max_v
        / static_cast<double>(parameters.num_interpolation + 1)
        / deceleration;
    return parameters;
}

std::vector<Path> rootPaths(const PBS& pbs, int agentCount)
{
    if (pbs.dummy_start == 0)
        throw std::runtime_error("PBS did not construct a root node");
    std::vector<Path> paths(agentCount);
    std::vector<bool> present(agentCount, false);
    for (const auto& entry : pbs.dummy_start->paths)
    {
        if (entry.first < 0 || entry.first >= agentCount)
            throw std::runtime_error("PBS root contains an invalid agent index");
        paths[entry.first] = entry.second;
        present[entry.first] = true;
    }
    for (int agent = 0; agent < agentCount; ++agent)
        if (!present[agent] || paths[agent].states.empty())
            throw std::runtime_error("PBS root is missing a single-agent path");
    return paths;
}

std::vector<Path> independentPaths(
    const Arguments& arguments,
    int agentCount)
{
    const YAML::Node input = YAML::LoadFile(arguments.input);
    std::vector<Path> paths(agentCount);
    for (int agent = 0; agent < agentCount; ++agent)
    {
        YAML::Node isolated;
        isolated["map"] = input["map"];
        isolated["agents"].push_back(input["agents"][agent]);
        const std::string filename = arguments.metadata
            + ".agent" + std::to_string(agent) + ".yaml";
        {
            std::ofstream stream(filename.c_str());
            if (!stream)
                throw std::runtime_error(
                    "cannot write isolated-agent instance");
            stream << isolated;
        }
        std::srand(0);
        Instance instance(filename);
        PBS pbs(instance, 0);
        const bool success = pbs.solve(arguments.timeLimit);
        std::vector<Path> isolatedPaths;
        const bool havePath = success && pbs.getPaths(isolatedPaths)
            && isolatedPaths.size() == 1
            && !isolatedPaths[0].states.empty();
        std::remove(filename.c_str());
        if (!havePath)
            throw std::runtime_error(
                "agent " + std::to_string(agent)
                + " has no independently feasible Hybrid-A* path");
        paths[agent] = isolatedPaths[0];
    }
    return paths;
}

int conflictingPairs(const std::vector<Path>& paths)
{
    int count = 0;
    for (std::size_t first = 0; first < paths.size(); ++first)
    for (std::size_t second = first + 1; second < paths.size(); ++second)
    {
        const std::size_t horizon = std::max(
            paths[first].states.size(), paths[second].states.size());
        bool conflict = false;
        for (std::size_t stage = 0; stage < horizon && !conflict; ++stage)
        {
            const State& firstState = paths[first].states[
                std::min(stage, paths[first].states.size() - 1)].first;
            const State& secondState = paths[second].states[
                std::min(stage, paths[second].states.size() - 1)].first;
            conflict = firstState.agentCollision(secondState);
        }
        if (conflict) ++count;
    }
    return count;
}

int conflictingPairs(
    const std::vector<std::vector<OptimizeResult> >& guesses)
{
    int count = 0;
    for (std::size_t first = 0; first < guesses.size(); ++first)
    for (std::size_t second = first + 1; second < guesses.size(); ++second)
    {
        bool conflict = false;
        for (std::size_t stage = 0;
             stage < guesses[first].size() && !conflict; ++stage)
        {
            const State firstState(
                guesses[first][stage].x,
                guesses[first][stage].y,
                guesses[first][stage].yaw
            );
            const State secondState(
                guesses[second][stage].x,
                guesses[second][stage].y,
                guesses[second][stage].yaw
            );
            conflict = firstState.agentCollision(secondState);
        }
        if (conflict) ++count;
    }
    return count;
}

void dumpCorridors(
    const std::string& filename,
    const std::vector<std::vector<Corridor> >& corridors,
    const std::vector<std::vector<OptimizeResult> >& guesses)
{
    std::ofstream output(filename.c_str());
    if (!output) throw std::runtime_error("cannot open corridor output");
    output << std::fixed << std::setprecision(6);
    for (std::size_t agent = 0; agent < corridors.size(); ++agent)
    {
        output << "agent" << agent << ":\n";
        for (std::size_t stage = 0; stage < corridors[agent].size(); ++stage)
        {
            double xf, yf, xr, yr;
            State state(
                guesses[agent][stage].x,
                guesses[agent][stage].y,
                guesses[agent][stage].yaw
            );
            state.GetDiscCenter(xf, yf, xr, yr);
            const Corridor& corridor = corridors[agent][stage];
            output << "  - [" << xf << ", " << yf << ", "
                << corridor.xf_min << ", " << corridor.xf_max << ", "
                << corridor.yf_min << ", " << corridor.yf_max << "]\n";
            output << "  - [" << xr << ", " << yr << ", "
                << corridor.xr_min << ", " << corridor.xr_max << ", "
                << corridor.yr_min << ", " << corridor.yr_max << "]\n";
        }
    }
}

void dumpMetadata(
    const std::string& filename,
    bool pbsSuccess,
    const std::string& warmstartSource,
    bool corridorLegal,
    int rootPathConflicts,
    int warmstartConflicts,
    std::size_t agents,
    std::size_t horizon,
    int padStages,
    int delayFromAgent,
    int delayStages,
    int sequentialDelayStages,
    double searchTime,
    double corridorTime)
{
    YAML::Emitter output;
    output << YAML::BeginMap;
    output << YAML::Key << "pbs_success" << YAML::Value << pbsSuccess;
    output << YAML::Key << "warmstart_source" << YAML::Value
        << warmstartSource;
    output << YAML::Key << "root_conflicting_pairs" << YAML::Value
        << rootPathConflicts;
    output << YAML::Key << "warmstart_conflicting_pairs" << YAML::Value
        << warmstartConflicts;
    output << YAML::Key << "static_corridors_legal" << YAML::Value
        << corridorLegal;
    output << YAML::Key << "agents" << YAML::Value << agents;
    output << YAML::Key << "horizon" << YAML::Value << horizon;
    output << YAML::Key << "search_seed" << YAML::Value << 0;
    output << YAML::Key << "pad_stages" << YAML::Value << padStages;
    output << YAML::Key << "delay_from_agent" << YAML::Value
        << delayFromAgent;
    output << YAML::Key << "delay_stages" << YAML::Value
        << delayStages;
    output << YAML::Key << "sequential_delay_stages" << YAML::Value
        << sequentialDelayStages;
    output << YAML::Key << "search_time" << YAML::Value << searchTime;
    output << YAML::Key << "corridor_time" << YAML::Value << corridorTime;
    output << YAML::EndMap;
    std::ofstream stream(filename.c_str());
    if (!stream) throw std::runtime_error("cannot open metadata output");
    stream << output.c_str() << "\n";
}
}

int main(int argc, char* argv[])
{
    try
    {
        const Arguments arguments = parseArguments(argc, argv);
        readAgentConfig(arguments.config);
        const QpParm parameters = readParameters(arguments.config);
        Instance instance(arguments.input);

        const std::clock_t searchStart = std::clock();
        std::srand(0);
        PBS pbs(instance, arguments.screen);
        const bool pbsSuccess = pbs.solve(arguments.timeLimit);
        const double searchTime = static_cast<double>(
            std::clock() - searchStart) / CLOCKS_PER_SEC;

        std::vector<Path> paths;
        const bool usedRoot = arguments.root || !pbsSuccess;
        if (arguments.independent)
            paths = independentPaths(
                arguments, instance.getDefaultNumberOfAgents());
        else if (usedRoot)
        {
            if (pbs.dummy_start != 0)
                paths = rootPaths(pbs, instance.getDefaultNumberOfAgents());
            else
                paths = independentPaths(
                    arguments, instance.getDefaultNumberOfAgents());
        }
        else
        {
            if (!pbs.getPaths(paths))
                throw std::runtime_error("PBS reported success without paths");
        }
        const std::string warmstartSource = arguments.independent
            ? "independent_single_agent" : !usedRoot ? "pbs_goal"
            : (pbs.dummy_start != 0
                ? "pbs_root" : "independent_single_agent");
        const int pathConflicts = conflictingPairs(paths);

        std::vector<std::vector<OptimizeResult> > guesses;
        InterpolateInitalGuess(paths, guesses, instance.goal_states, parameters);
        if (guesses.empty() || guesses.front().empty())
            throw std::runtime_error("interpolation produced an empty warm start");
        if (arguments.sequentialDelayStages > 0)
        {
            const int totalDelay = static_cast<int>(guesses.size() - 1)
                * arguments.sequentialDelayStages;
            for (std::size_t agent = 0; agent < guesses.size(); ++agent)
            {
                const int delay = static_cast<int>(agent)
                    * arguments.sequentialDelayStages;
                OptimizeResult waiting = guesses[agent].front();
                waiting.v = waiting.a = waiting.d_steer = 0.0;
                std::vector<OptimizeResult> delayed(delay, waiting);
                delayed.insert(delayed.end(),
                    guesses[agent].begin(), guesses[agent].end());
                delayed.insert(delayed.end(), totalDelay - delay,
                    guesses[agent].back());
                guesses[agent].swap(delayed);
            }
        }
        else if (arguments.delayStages > 0)
        {
            for (std::size_t agent = 0; agent < guesses.size(); ++agent)
            {
                std::vector<OptimizeResult> delayed;
                if (static_cast<int>(agent) >= arguments.delayFromAgent)
                {
                    OptimizeResult waiting = guesses[agent].front();
                    waiting.v = waiting.a = waiting.d_steer = 0.0;
                    delayed.assign(arguments.delayStages, waiting);
                    delayed.insert(delayed.end(),
                        guesses[agent].begin(), guesses[agent].end());
                }
                else
                {
                    delayed = guesses[agent];
                    delayed.insert(delayed.end(), arguments.delayStages,
                        guesses[agent].back());
                }
                guesses[agent].swap(delayed);
            }
        }
        for (std::size_t agent = 0; agent < guesses.size(); ++agent)
        {
            const OptimizeResult terminal = guesses[agent].back();
            for (int stage = 0; stage < arguments.padStages; ++stage)
                guesses[agent].push_back(terminal);
        }
        const int warmstartConflicts = conflictingPairs(guesses);

        std::vector<std::vector<Corridor> > corridors;
        double corridorTime = 0.0;
        const bool corridorLegal = calcCorridors(
            guesses,
            instance.obstacles,
            instance.dimx,
            instance.dimy,
            corridors,
            guesses.size(),
            guesses.front().size(),
            corridorTime,
            arguments.screen
        );

        SolutionStatistics statistics;
        statistics.rt_search = searchTime;
        statistics.search_status = pbsSuccess ? 2 : 0;
        dumpSolutions(arguments.guess, guesses, statistics);
        dumpCorridors(arguments.corridor, corridors, guesses);
        dumpMetadata(
            arguments.metadata,
            pbsSuccess,
            warmstartSource,
            corridorLegal,
            pathConflicts,
            warmstartConflicts,
            guesses.size(),
            guesses.front().size() - 1,
            arguments.padStages,
            arguments.delayFromAgent,
            arguments.delayStages,
            arguments.sequentialDelayStages,
            searchTime,
            corridorTime
        );
        std::cout << "exported " << warmstartSource
            << " warm start with " << pathConflicts
            << " root-path and " << warmstartConflicts
            << " post-delay conflicting agent pairs" << std::endl;
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << "csdo_root_warmstart_exporter: " << error.what() << std::endl;
        return 2;
    }
}
