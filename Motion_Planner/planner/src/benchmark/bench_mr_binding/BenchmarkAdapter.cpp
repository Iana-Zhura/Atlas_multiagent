#include "BenchmarkAdapter.h"

#include <base/environments/GridMaze.h>
#include <base/PathStatistics.hpp>
#include <utils/ScenarioLoader.h>
#include <utils/PlannerUtils.hpp>
#include <utils/Log.h>
#include <metrics/ClearingMetric.h>
#include <metrics/MaxCurvatureMetric.h>
#include <metrics/PathLengthMetric.h>
#include <metrics/TrajectoryMetric.h>
#include <metrics/AOLMetric.h>
#include <metrics/NormalizedCurvatureMetric.h>
#include <fstream>


BenchmarkAdapter::BenchmarkAdapter(PlannerSettings::GlobalSettings &settings) {
    if (settings.benchmark.moving_ai.active) {
        loadMovingAIScenarios(settings);
    } else {
        loadOtherScenarios(settings);
    }
    mBenchmarkEnvironment = settings.environment;
    Log::instantiateRun();
}

void BenchmarkAdapter::loadMovingAIScenarios(PlannerSettings::GlobalSettings &settings) {
    ScenarioLoader scenarioLoader;
    scenarioLoader.load(settings.benchmark.moving_ai.scenario);
    const auto n = scenarioLoader.scenarios().size();
    std::size_t id = (global::settings.benchmark.moving_ai.start + n) % n;
    auto &scenario = scenarioLoader.scenarios()[id];
    settings.environment = GridMaze::createFromMovingAiScenario(scenario);
    settings.env.collision.initializeCollisionModel();
}

void BenchmarkAdapter::loadOtherScenarios(PlannerSettings::GlobalSettings &settings) {
    settings.env.createEnvironment();
    settings.env.grid.seed += 1;
}

BenchmarkAdapter::BenchmarkAdapter(const std::string &settingFile) {
    std::ifstream stream(settingFile);
    const nlohmann::json settings = nlohmann::json::parse(stream);
    global::settings.load(settings);
    BenchmarkAdapter benchmark(global::settings);
    if (global::settings.benchmark.moving_ai.active) {
        loadMovingAIScenarios(global::settings);
    } else {
        loadOtherScenarios(global::settings);
    }
    global::settings.steer.initializeSteering();
    mBenchmarkEnvironment = global::settings.environment;
    Log::instantiateRun();
    stopwatch.start();
}

std::tuple<double, double, double, double> BenchmarkAdapter::bounds() {
    auto bounds = mBenchmarkEnvironment->bounds();
    return {
            bounds.low.at(0),
            bounds.high.at(0),
            bounds.low.at(1),
            bounds.high.at(1)
    };
}

PathStatistics BenchmarkAdapter::evaluate(ompl::geometric::PathGeometric &path, const std::string &name) {
    PathStatistics stats(name);
    stats.planning_time = planningTime();
    stats.collision_time = global::settings.environment->elapsedCollisionTime();
    stats.steering_time = global::settings.ompl.steering_timer.elapsed();
    stats.planner = name;
    if (path.getStateCount() < 2) {
        stats.path_found = false;
        stats.exact_goal_path = false;
        return stats;
    }
    stats.path_found = true;
    auto solution = PlannerUtils::interpolated(path);
    stats.path_collides = !isValid(solution, stats.collisions);
    stats.exact_goal_path = Point(solution.getStates().back()).distance(
            global::settings.environment->goal()) <= global::settings.exact_goal_radius;

    stats.path_length = PathLengthMetric::evaluate(solution);
    stats.max_curvature = MaxCurvatureMetric::evaluate(solution);
    stats.normalized_curvature =
            NormalizedCurvatureMetric::evaluate(solution);
    stats.aol = AOLMetric::evaluate(solution);
    stats.smoothness = solution.smoothness();

    if (global::settings.evaluate_clearing &&
        global::settings.environment->distance(0., 0.) >= 0.) {
        const auto clearings = ClearingMetric::clearingDistances(solution);
        stats.mean_clearing_distance = statistics::mean(clearings);
        stats.median_clearing_distance = statistics::median(clearings);
        stats.min_clearing_distance = statistics::min(clearings);
        stats.max_clearing_distance = statistics::max(clearings);
    }
    const auto pointPath = Point::fromPath(solution);
    computeCusps(stats, pointPath);
    return stats;
}

void BenchmarkAdapter::computeCusps(PathStatistics &stats, const std::vector<Point>& path) {
    std::vector<Point> &cusps = stats.cusps.value();

    auto prev = path.begin();
    auto current = prev;
    auto next = prev;
    while (next != path.end()) {
        // advance until current point != prev point, i.e., skip duplicates
        if (prev->distance(*current) <= 0) {
            ++current;
            ++next;
        } else if (current->distance(*next) <= 0) {
            ++next;
        } else {
            const double yaw_prev = PlannerUtils::slope(*prev, *current);
            const double yaw_next = PlannerUtils::slope(*current, *next);

            // compute angle difference in [0, pi)
            // close to pi -> cusp; 0 -> straight line; inbetween -> curve
            const double yaw_change =
                    std::abs(PlannerUtils::normalizeAngle(yaw_next - yaw_prev));

            if (yaw_change > global::settings.cusp_angle_threshold) {
                cusps.emplace_back(*current);
            }
            prev = current;
            current = next;
            ++next;
        }
    }
}

void BenchmarkAdapter::evaluateAndSaveResult(const std::vector<Position> &resultPath, const std::string &name) {
    stopwatch.stop();
    auto omplResultPath = omplPathFromPositions(resultPath);
    nlohmann::json info;
    mBenchmarkEnvironment->to_json(info["environment"]);
    info["settings"] = nlohmann::json(global::settings)["settings"];
    auto &path_info = info["plans"][name];
    path_info["trajectory"] = Log::serializeTrajectory(omplResultPath);
    path_info["path"] = Log::serializeTrajectory(omplResultPath);
    path_info["stats"] = nlohmann::json(evaluate(omplResultPath, name))["stats"];
    Log::log(info);
    Log::save(global::settings.benchmark.log_file);
}

ompl::geometric::PathGeometric BenchmarkAdapter::omplPathFromPositions(const std::vector<Position> &resultPath) {
    ompl::geometric::PathGeometric result(global::settings.ompl.space_info);
    for (const auto position: resultPath) {
        result.append(base::StateFromXYT(position.x, position.y, position.angle));
    }
    return result;
}

Position BenchmarkAdapter::start() {
    auto startState = mBenchmarkEnvironment->startScopedState();
    auto x = startState->getX();
    auto y = startState->getY();
    auto angle = startState->getYaw();
    return {x, y, angle};
}

Position BenchmarkAdapter::goal() {
    auto startState = mBenchmarkEnvironment->goalScopedState();
    auto x = startState->getX();
    auto y = startState->getY();
    auto angle = startState->getYaw();
    return {x, y, angle};
}

std::vector<bool> BenchmarkAdapter::collides_positions(const std::vector<Position> &positions) {
    std::vector<bool> result(positions.size());
    for (int i = 0; i < positions.size(); i++) {
        const auto &position = positions[i];
        ompl::base::ScopedState<ob::SE2StateSpace> state(
                global::settings.ompl.state_space);
        state->setX(position.x);
        state->setY(position.y);
        state->setYaw(position.angle);
        result[i] = !mBenchmarkEnvironment->checkValidity(state.get());
    }
    return result;
}

bool BenchmarkAdapter::isValid(ompl::geometric::PathGeometric &path, std::vector<Point> &collisions) {
    collisions.clear();
    for (const auto *state: path.getStates())
        if (!mBenchmarkEnvironment->checkValidity(state))
            collisions.emplace_back(state);
    return collisions.empty();
}

double BenchmarkAdapter::planningTime() {
    return stopwatch.elapsed();
}

std::tuple<bool, double> BenchmarkAdapter::evaluatePath(const std::vector<Position> &resultPath) {
    auto omplResultPath = omplPathFromPositions(resultPath);
    auto solution = PlannerUtils::interpolated(omplResultPath);
    std::vector<Point> collisions;
    auto path_collides = !isValid(solution, collisions);
    auto path_length = PathLengthMetric::evaluate(solution);
    return {path_collides, path_length};
}

