#! /usr/bin/env python
import argparse
import os
import time

import numpy as np
import torch
from matplotlib import pyplot as plt
from pytorch_lightning.utilities import AttributeDict

from neural_field_optimal_planner.benchmark_adapter import BenchmarkAdapter
from neural_field_optimal_planner.benchmark_adapter.benchmark_collision_checker import BenchmarkCollisionChecker
from neural_field_optimal_planner.planner_factory import PlannerFactory
from neural_field_optimal_planner.plotting_utils import prepare_figure, plot_planner_data

torch.random.manual_seed(100)
np.random.seed(400)

with open("/home/mikhail/tmp.id", "r") as fd:
    config_id = int(fd.read())

with open("/home/mikhail/tmp.id", "w") as fd:
    fd.write(str(config_id + 1))

planner_parameters = AttributeDict(
    device="cpu",
    trajectory_length=100,
    trajectory_initializer=AttributeDict(
        name="AstarTrajectoryInitializer",
        resolution=0.5
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
        collision_beta=1
    )
)
if config_id == 1:
    planner_parameters.collision_model.sigma = 0.1
    planner_parameters.collision_model.sigma = 0.1

if config_id == 2:
    planner_parameters.collision_model.sigma = 100

if config_id == 3:
    planner_parameters.planner.collision_weight = 10

if config_id == 4:
    planner_parameters.planner.collision_weight = 1000

# if config_id == 3:
#     planner_parameters.angle_weight = 0.05
#     planner_parameters.constraint_deltas_weight = 0.5
#     planner_parameters.multipliers_lr = 1e-3
#
# if config_id == 4:
#     planner_parameters.constraint_deltas_weight = 0.5
#     planner_parameters.multipliers_lr = 1e-3

# For polygon dataset configs
# planner_parameters = AttributeDict(
#     device="cpu",
#     trajectory_length=20,
#     collision_model=AttributeDict(
#         mean=0,
#         sigma=3,
#         use_cos=True,
#         bias=True,
#         use_normal_init=True,
#         angle_encoding=True,
#         name="ONF"
#     ),
#     trajectory_initializer=AttributeDict(
#         name="TrajectoryInitializer",
#     ),
#     collision_optimizer=AttributeDict(
#         lr=1e-2,
#         betas=(0.9, 0.9)
#     ),
#     trajectory_optimizer=AttributeDict(
#         lr=5e-2,
#         betas=(0.9, 0.9)
#     ),
#     planner=AttributeDict(
#         name="ConstrainedNERFOptPlanner",
#         trajectory_random_offset=0.02,
#         collision_weight=1,
#         velocity_hessian_weight=0.5,
#         random_field_points=10,
#         init_collision_iteration=0,
#         constraint_deltas_weight=200,
#         multipliers_lr=0.1,
#         init_collision_points=100,
#         reparametrize_trajectory_freq=10,
#         optimize_collision_model_freq=1,
#         angle_weight=15,
#         angle_offset=0.3,
#         boundary_weight=1,
#         collision_multipliers_lr=1e-3,
#         collision_beta=4
#     )
# )

parser = argparse.ArgumentParser()
parser.add_argument("settings")
parser.add_argument("--show", default=False)
args = parser.parse_args()

print("Start with config", args.settings)
benchmark = BenchmarkAdapter(args.settings)
print("Benchmark adapter initialized")
collision_checker = BenchmarkCollisionChecker(benchmark, benchmark.bounds())
print("Collision checker initialized")
planner = PlannerFactory.make_constrained_onf_planner(collision_checker, planner_parameters)
goal_point = benchmark.goal().as_vec()
start_point = benchmark.start().as_vec()
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
for i in range(1000):
    planner.step()
    if is_show:
        trajectory = planner.get_path()
        fig.clear()
        prepare_figure(trajectory_boundaries)
        plot_planner_data(trajectory, collision_model, trajectory_boundaries, np.zeros((0, 2)), device=device)
        # plot_nerf_opt_planner(planner)
        # plot_collision_positions(planner.checked_positions, planner.truth_collision)
        plt.pause(0.01)
    if (i > 0) and (i % 20 == 0):
        collision, length = benchmark.evaluate_path(planner.get_path())
        print("Current path length =", length, "collision =", collision)
        if not collision and length < best_length:
            best_length = length
            best_path = planner.get_path()
        elif not collision:
            break

path = planner.get_path()
collision, length = benchmark.evaluate_path(path)

if (length > best_length) or (collision and best_path is not None):
    path = best_path

# result = np.array([start_point, goal_point])
if config_id == 0:
    name = "sigma = 10, col_w = 100"
if config_id == 1:
    name = "sigma = 0.1, col_w = 100"
if config_id == 2:
    name = "sigma = 100, col_w = 100"
if config_id == 3:
    name = "sigma = 10, col_w = 10"
if config_id == 4:
    name = "sigma = 10, col_w = 1000"

benchmark.evaluate_and_save_results(path, name)
# benchmark.evaluate_and_save_results(result, "constrained_onf_planner")
