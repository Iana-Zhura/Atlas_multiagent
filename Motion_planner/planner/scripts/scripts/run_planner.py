import numpy as np

import torch
from matplotlib import pyplot as plt
from pytorch_lightning.utilities import AttributeDict
import time
import sys
import os
import neural_field_optimal_planner
# sys.path.insert(1,'~/anaconda3/Atlas/install/planner/lib/python3.8/site-packages/planner/scripts/neural_field_optimal_planner')
# sys.path.append('~/anaconda3/Atlas/src/Motion_planner/planner/planner/scripts/neural_field_optimal_planner')
# sys.path.append('~/anaconda3/Atlas/src/Motion_planner/planner/planner/scripts/neural_field_optimal_planner/astar')
# sys.path.append('~/anaconda3/Atlas/src/Motion_planner/planner/planner/scripts/neural_field_optimal_planner/utils')


from neural_field_optimal_planner.collision_checker import CircleDirectedCollisionChecker, RectangleCollisionChecker
from neural_field_optimal_planner.planner_factory import PlannerFactory
from neural_field_optimal_planner.plotting_utils import *
from neural_field_optimal_planner.test_environment_builder import TestEnvironmentBuilder
from neural_field_optimal_planner.trajectory_3D import trajectory_to_3D
from neural_field_optimal_planner.trajectory_3D import calculate_path
from neural_field_optimal_planner.trajectory_2D import trajectory_to_2D

torch.random.manual_seed(100)
np.random.seed(400)

planner_parameters = AttributeDict(
    device="cpu",
    trajectory_length=50,
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


# test_environment = TestEnvironmentBuilder().make_dog_environment()
# obs_map = TestEnvironmentBuilder().map.obstacle_map_adaptive_h
# voxel_size = TestEnvironmentBuilder().map.voxel_size
# robot_max_h = TestEnvironmentBuilder().map.ROB_MAX_H

environment_builder = TestEnvironmentBuilder()
obs_map = environment_builder.get_map()[0]
voxel_size = environment_builder.get_map()[1]
robot_max_h = environment_builder.get_map()[2]

test_environment = environment_builder.make_dog_environment()

# test_environment = TestEnvironmentBuilder().make_car_environment()
obstacle_points = test_environment.obstacle_points
# collision_checker = CircleDirectedCollisionChecker(0.3, (0, 3, 0, 3))
# collision_checker = RectangleCollisionChecker((-0.2, 0.2, -0.2, 0.2), (0, 3, 0, 3))
collision_checker = RectangleCollisionChecker((-0.05, 0.05, -0.05, 0.05), (0, 5, 0, 5))
collision_checker.update_obstacle_points(test_environment.obstacle_points)

planner = PlannerFactory.make_constrained_onf_planner(collision_checker, planner_parameters)
goal_point = test_environment.goal_point
start_point = test_environment.start_point
trajectory_boundaries = test_environment.bounds

planner.init(start_point, goal_point, trajectory_boundaries)
device = planner._device
collision_model = planner._collision_model
fig = plt.figure(1, dpi=200)

def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    d = np.sqrt(np.power((x2 - x1), 2) + np.power((y2 - y1), 2))
    return d

base_path = os.getcwd()
delta = 20
start = time.time()
count = 0
for i in range(1000):
    
    planner.step()
    trajectory = planner.get_path() 
    
    fig.clear()
    prepare_figure(trajectory_boundaries)
    trajectory_3D, path_color = trajectory_to_3D(trajectory, obs_map, voxel_size, robot_max_h)
    # path_color = trajectory_to_2D(trajectory, obs_map, voxel_size, robot_max_h)
    # path_length = calculate_path(trajectory, 0)
    # print("The distance:", path_length)
    plot_planner_data(trajectory_3D, path_color, collision_model, trajectory_boundaries, obstacle_points, device=device)
    # plot_nerf_opt_planner(planner)
    # plot_collision_positions(planner.checked_positions, planner.truth_collision)
    plt.pause(0.01)
    if i == 999:
        np.save(f"{base_path}/src/Motion_planner/planner/scripts/Metrics/trajectory_3D.npy", trajectory)
    if i != 0:
       
        for j in range(delta,len(trajectory)-delta):
            
            dist = distance(trajectory[j][0], trajectory[j][1],
                    prev_trajectory[j][0], prev_trajectory[j][1])
            
            if dist <= 0.0004 and any(path_color) != "red":
                count += 1
                print("Distance:", dist)
                if count > 80:  
                    break

        else:
            prev_trajectory = trajectory
            continue
        prev_trajectory = trajectory
        # print(i)
        print(f"Planner run took: {time.time() - start} seconds")
        np.save(f"{base_path}/src/Motion_planner/planner/scripts/Metrics/trajectory_3D.npy", trajectory)
        break
    else:
        prev_trajectory = trajectory
         
        