import numpy as np
import torch
from matplotlib import pyplot as plt
from pytorch_lightning.utilities import AttributeDict

from neural_field_optimal_planner.collision_checker import CircleDirectedCollisionChecker, RectangleCollisionChecker
from neural_field_optimal_planner.planner_factory import PlannerFactory
from neural_field_optimal_planner.plotting_utils import *
from neural_field_optimal_planner.test_environment_builder import TestEnvironmentBuilder
import time
import torch.autograd

from neural_field_optimal_planner.utils.timer import timer

torch.autograd.set_detect_anomaly(False)
torch.autograd.profiler.profile(True)
torch.autograd.profiler.emit_nvtx(False)
start_time = time.time()
torch.random.manual_seed(100)
np.random.seed(400)

planner_parameters = AttributeDict(
    device="cpu",
    trajectory_length=100,
    collision_model=AttributeDict(
        mean=0,
        sigma=1,
        use_cos=True,
        bias=True,
        use_normal_init=True,
        angle_encoding=True,
        name="ONF"
    ),
    trajectory_initializer=AttributeDict(
        name="TrajectoryInitializer",
        resolution=0.05
    ),
    collision_optimizer=AttributeDict(
        lr=5e-2,
        betas=(0.9, 0.9)
    ),
    trajectory_optimizer=AttributeDict(
        lr=1e-2,
        betas=(0.9, 0.9)
    ),
    planner=AttributeDict(
        name="ConstrainedNERFOptPlanner",
        trajectory_random_offset=0.02,
        collision_weight=1,
        velocity_hessian_weight=0.5,
        random_field_points=10,
        init_collision_iteration=0,
        constraint_deltas_weight=20,
        multipliers_lr=0.1,
        init_collision_points=100,
        reparametrize_trajectory_freq=10,
        optimize_collision_model_freq=1,
        angle_weight=0.5,
        angle_offset=0.3,
        boundary_weight=1,
        collision_multipliers_lr=1e-3
    )
)


test_environment = TestEnvironmentBuilder().make_test_environment_with_angles()
# test_environment = TestEnvironmentBuilder().make_car_environment()
obstacle_points = test_environment.obstacle_points
collision_checker = CircleDirectedCollisionChecker(0.3, (0, 3, 0, 3))
# collision_checker = RectangleCollisionChecker((-0.2, 0.2, -0.2, 0.2), (0, 3, 0, 3))
# collision_checker = RectangleCollisionChecker((-0.3, 0.2, -0.3, 0.2), (0, 3, 0, 3))
collision_checker.update_obstacle_points(test_environment.obstacle_points)
collision_checker.update_boundaries(test_environment.bounds)

planner = PlannerFactory.make_constrained_onf_planner(collision_checker, planner_parameters)
goal_point = test_environment.goal_point
start_point = test_environment.start_point
trajectory_boundaries = test_environment.bounds

planner.init(start_point, goal_point, trajectory_boundaries)
device = planner._device
collision_model = planner._collision_model

is_show = False
fig = None
if is_show:
    fig = plt.figure(dpi=200)

start_optimization_time = time.time()
print("Preparation duration =", start_optimization_time - start_time)

iteration_durations = []
# with torch.profiler.profile(
#     activities=[
#         torch.profiler.ProfilerActivity.CPU,
#         torch.profiler.ProfilerActivity.CUDA,
#     ]
# ) as p:
for i in range(1000):
    start_iteration = time.time()
    planner.step()
    if is_show:
        trajectory = planner.get_path()
        fig.clear()
        prepare_figure(trajectory_boundaries)
        plot_planner_data(trajectory, collision_model, trajectory_boundaries, obstacle_points, device=device)
        # plot_nerf_opt_planner(planner)
        plot_collision_positions(planner.checked_positions, planner.truth_collision)
        plt.pause(0.01)
    end_iteration = time.time()
    iteration_durations.append(end_iteration - start_iteration)

# print(p.key_averages().table(
#     sort_by="self_cpu_time_total", row_limit=-1))

timer.print()
end_time = time.time()
iteration_durations = np.array(iteration_durations)
print("Iteration duration =", np.mean(iteration_durations) * 1000, "+-", np.std(iteration_durations) * 1000, "ms")
print("Optimization duration =", end_time - start_optimization_time)
print("Total duration =", end_time - start_time)
