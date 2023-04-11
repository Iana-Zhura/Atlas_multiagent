#! /usr/bin/env python3
import argparse
import json

import numpy as np
import torch
from matplotlib import pyplot as plt
from pytorch_lightning.utilities import AttributeDict

from neural_field_optimal_planner.benchmark_adapter import BenchmarkAdapter
from neural_field_optimal_planner.benchmark_adapter.benchmark_collision_checker import BenchmarkCollisionChecker
from neural_field_optimal_planner.planner_factory import PlannerFactory
from neural_field_optimal_planner.plotting_utils import prepare_figure, plot_planner_data, plot_collision_positions
from neural_field_optimal_planner.utils.config import Config
from neural_field_optimal_planner.utils.position2 import Position2

torch.random.manual_seed(100)
np.random.seed(400)

planner_parameters = AttributeDict(
    device="cpu",
    trajectory_length=100,
    trajectory_initializer=AttributeDict(
        name="AstarTrajectoryInitializer",
        resolution=0.5,
        init_angles_with_trajectory=False,
    ),
    collision_model=AttributeDict(
        mean=0,
        sigma=10,
        use_cos=True,
        bias=True,
        use_normal_init=True,
        angle_encoding=True,
        name="ONF"
    ),
    collision_optimizer=AttributeDict(
        lr=2e-2,
        betas=(0.9, 0.9)
    ),
    trajectory_optimizer=AttributeDict(
        lr=5e-2,
        betas=(0.9, 0.9)
    ),
    planner=AttributeDict(
        name="ConstrainedNERFOptPlanner",
        trajectory_random_offset=0.02,
        collision_weight=100,
        velocity_hessian_weight=0.5,
        random_field_points=10,
        init_collision_iteration=0,
        constraint_deltas_weight=100,
        multipliers_lr=0.1,
        init_collision_points=100,
        reparametrize_trajectory_freq=10,
        optimize_collision_model_freq=1,
        angle_weight=5,
        angle_offset=0.3,
        boundary_weight=1,
        direction_delta_weight=100,
        collision_multipliers_lr=1e-3,
        collision_beta=10
    ),
    max_iterations=1000,
    min_iterations=200,
    check_collision_frequence=50
)

parser = argparse.ArgumentParser()
parser.add_argument("settings")
parser.add_argument("--show", default=False)
args = parser.parse_args()

print("Start with config", args.settings)
benchmark = BenchmarkAdapter(args.settings)
print("Benchmark adapter initialized")
collision_checker = BenchmarkCollisionChecker(benchmark, benchmark.bounds())
print("Collision checker initialized")

config = Config.from_dict(planner_parameters)
with open(args.settings, "r") as fd:
    settings_config = json.load(fd)
if "nfomp" in settings_config["settings"].keys():
    config.update(settings_config["settings"]["nfomp"])
planner_parameters = config.as_attribute_dict()
print(planner_parameters)

planner = PlannerFactory.make_constrained_onf_planner(collision_checker, planner_parameters)

goal_point = benchmark.goal().as_vec()
start_point = benchmark.start().as_vec()
collisions = collision_checker.check_collision(Position2.from_array([benchmark.start(), benchmark.goal()]))
print("Start collision ", collisions[0])
if collisions[0]:
    exit(3)
print("Goal collision ", collisions[1])
if collisions[1]:
    exit(4)

trajectory_boundaries = benchmark.bounds()
planner.init(start_point, goal_point, trajectory_boundaries)
device = planner._device
collision_model = planner._collision_model
is_show = args.show
fig = None
if is_show:
    fig = plt.figure(dpi=200)

best_length = np.inf
best_path = None
for i in range(planner_parameters.max_iterations):
    planner.step()
    if is_show:
        trajectory = planner.get_path()
        fig.clear()
        prepare_figure(trajectory_boundaries)
        plot_planner_data(trajectory, collision_model, trajectory_boundaries, np.zeros((0, 2)), device=device)
        plt.pause(0.01)
    if (i > planner_parameters.min_iterations) and (i % planner_parameters.check_collision_frequence == 0):
        collision, length = benchmark.evaluate_path(planner.get_path())
        print("Current path length =", length, "collision =", collision)
        if not collision and length < best_length:
            best_length = length
            best_path = planner.get_path()
        elif not collision:
            break

path = planner.get_path()
collision, length = benchmark.evaluate_path(path)

if collision and best_path is not None:
    path = best_path

benchmark.evaluate_and_save_results(path, "constrained_onf_planner")
