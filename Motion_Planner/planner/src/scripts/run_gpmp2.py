#! /usr/bin/env python
import argparse

from neural_field_optimal_planner.astar.astar_trajectory_initializer import AstarTrajectoryInitializer
from neural_field_optimal_planner.utils.universal_factory import UniversalFactory

paths = [
    "/usr/local/cython/gtsam",
    "/home/mikhail/research/gpmp2/cmake-build-release/cython/gpmp2",
    "/home/mikhail/research/gpmp2/gpmp2_python"
]
import sys
for path in paths:
    if path not in sys.path:
        sys.path.append(path)

import numpy as np
import torch
from matplotlib import pyplot as plt
from neural_field_optimal_planner.utils.position2 import Position2
from pytorch_lightning.utilities import AttributeDict

from neural_field_optimal_planner.benchmark_adapter import BenchmarkAdapter
from neural_field_optimal_planner.benchmark_adapter.benchmark_collision_checker import BenchmarkCollisionChecker
from neural_field_optimal_planner.planner_factory import PlannerFactory
from neural_field_optimal_planner.plotting_utils import prepare_figure, plot_planner_data
from gtsam import *
from gpmp2 import *
import gpmp2
from gpmp2_python.datasets.generate2Ddataset import generate2Ddataset, Dataset
from gpmp2_python.robots.generateArm import generateArm
from gpmp2_python.utils.plot_utils import *
from gpmp2_python.utils.signedDistanceField2D import signedDistanceField2D
import cv2

parser = argparse.ArgumentParser()
parser.add_argument("settings")
parser.add_argument("--show", default=False)
args = parser.parse_args()

print("Start with config", args.settings)
benchmark = BenchmarkAdapter(args.settings)
print("Benchmark adapter initialized")
collision_checker = BenchmarkCollisionChecker(benchmark, benchmark.bounds())
print("Collision checker initialized")
goal_point = benchmark.goal().as_vec()
start_point = benchmark.start().as_vec()
trajectory_boundaries = benchmark.bounds()
print(trajectory_boundaries)
factory = UniversalFactory([AstarTrajectoryInitializer])
parameters = AttributeDict(
    name="AstarTrajectoryInitializer",
    resolution=0.5
)
trajectory_initializer = factory.make_from_parameters(parameters, collision_checker=collision_checker)
print(trajectory_initializer)
trajectory = torch.zeros(100, 3, requires_grad=True, device="cpu")
trajectory_initializer.initialize_trajectory(trajectory, torch.tensor(start_point[None]), torch.tensor(goal_point[None]))
trajectory = trajectory.cpu().detach().numpy()

resolution = 0.5
dataset = Dataset()
dataset.cols = int((trajectory_boundaries[1] - trajectory_boundaries[0]) // resolution + 1)  # x
dataset.rows = int((trajectory_boundaries[3] - trajectory_boundaries[2]) // resolution + 1)  # y
dataset.origin_x = trajectory_boundaries[0]
dataset.origin_y = trajectory_boundaries[2]
dataset.cell_size = resolution

grid_x, grid_y = np.meshgrid(np.linspace(trajectory_boundaries[0], trajectory_boundaries[1], dataset.cols),
                             np.linspace(trajectory_boundaries[2], trajectory_boundaries[3], dataset.rows))
grid = np.stack([grid_x, grid_y, np.zeros_like(grid_x)], axis=2).reshape(-1, 3)
dataset.map = collision_checker.check_collision(Position2.from_vec(grid)).reshape(dataset.rows, dataset.cols)
field = signedDistanceField2D(dataset.map, dataset.cell_size)
origin_point2 = Point2(dataset.origin_x, dataset.origin_y)
sdf = PlanarSDF(origin_point2, dataset.cell_size, field)

if args.show:
    figure1 = plt.figure(dpi=200)
    axis1 = figure1.gca()  # for 3-d, set gca(projection='3d')
    plotSignedDistanceField2D(
        figure1, axis1, field, dataset.origin_x, dataset.origin_y, dataset.cell_size
    )
# plt.pause(2)
## settings
total_time_sec = 10.0
total_time_step = 99
total_check_step = 50.0
delta_t = total_time_sec / total_time_step
check_inter = int(total_check_step / total_time_step - 1)

use_GP_inter = True

# point robot model
pR = PointRobot(2, 1)
spheres_data = np.asarray([0.0, 0.0, 0.0, 0.0, 1])
nr_body = spheres_data.shape[0]
sphere_vec = BodySphereVector()
sphere_vec.push_back(
    BodySphere(spheres_data[0], spheres_data[4], Point3(spheres_data[1:4]))
)
pR_model = PointRobotModel(pR, sphere_vec)

# GP
Qc = np.identity(2)
Qc_model = noiseModel_Gaussian.Covariance(Qc)

# Obstacle avoid settings
cost_sigma = 0.1
epsilon_dist = 1

# prior to start/goal
pose_fix = noiseModel_Isotropic.Sigma(2, 0.0001)
vel_fix = noiseModel_Isotropic.Sigma(2, 0.0001)

start_conf = start_point[:2]
start_vel = np.asarray([0, 0])
end_conf = goal_point[:2]
end_vel = np.asarray([0, 0])
avg_vel = (end_conf / total_time_step) / delta_t
# avg_vels = trajectory[:1] - trajectory

graph = NonlinearFactorGraph()
init_values = Values()
for i in range(0, total_time_step + 1):
    key_pos = symbol(ord("x"), i)
    key_vel = symbol(ord("v"), i)

    # % initialize as straight line in conf space
    pose = start_conf * float(total_time_step - i) / float(
        total_time_step
    ) + end_conf * i / float(total_time_step)
    print(pose)
    pose = trajectory[i, :2]
    vel = avg_vel
    init_values.insert(key_pos, pose)
    init_values.insert(key_vel, vel)

    #% start/end priors
    if i == 0:
        graph.push_back(PriorFactorVector(key_pos, start_conf, pose_fix))
        graph.push_back(PriorFactorVector(key_vel, start_vel, vel_fix))
    elif i == total_time_step:
        graph.push_back(PriorFactorVector(key_pos, end_conf, pose_fix))
        graph.push_back(PriorFactorVector(key_vel, end_vel, vel_fix))

    # GP priors and cost factor
    if i > 0:
        key_pos1 = symbol(ord("x"), i - 1)
        key_pos2 = symbol(ord("x"), i)
        key_vel1 = symbol(ord("v"), i - 1)
        key_vel2 = symbol(ord("v"), i)

        temp = GaussianProcessPriorLinear(
            key_pos1, key_vel1, key_pos2, key_vel2, delta_t, Qc_model
        )
        graph.push_back(temp)

        #% cost factor
        graph.push_back(
            ObstaclePlanarSDFFactorPointRobot(
                key_pos, pR_model, sdf, cost_sigma, epsilon_dist
            )
        )

        #% GP cost factor
        if use_GP_inter and check_inter > 0:
            for j in range(1, check_inter + 1):
                tau = j * (total_time_sec / total_check_step)
                graph.add(
                    ObstaclePlanarSDFFactorGPPointRobot(
                        key_pos1,
                        key_vel1,
                        key_pos2,
                        key_vel2,
                        pR_model,
                        sdf,
                        cost_sigma,
                        epsilon_dist,
                        Qc_model,
                        delta_t,
                        tau,
                    )
                )

parameters = DoglegParams()
optimizer = DoglegOptimizer(graph, init_values, parameters)
optimizer.optimizeSafely()
result = optimizer.values()

points = []
for i in range(total_time_step + 1):
    conf = getPoint2FromValues(symbol(ord("x"), i), result)
    points.append([conf.x(), conf.y()])
path = np.array(points)

x1 = dataset.origin_x
x2 = dataset.origin_x + (dataset.cols - 1) * dataset.cell_size
y1 = dataset.origin_y
y2 = dataset.origin_y + (dataset.rows - 1) * dataset.cell_size

x, y = np.meshgrid(np.linspace(x1, x2, dataset.cols), np.linspace(y1, y2, dataset.rows))

if args.show:
    plt.figure(dpi=200)
    plt.gca().pcolormesh(x, y, 255 - dataset.map, cmap='gray', shading='auto')
    plt.scatter(path[:, 0], path[:, 1], s=20)
    plt.plot(path[:, 0], path[:, 1])

    plt.gca().axis("off")
    plt.gca().set_aspect('equal')
    # plt.pause(10)
    plt.show()
trajectory = path
angles = np.zeros(trajectory.shape[0])
delta = trajectory[2:] - trajectory[:-2]
angles[1:-1] = np.arctan2(delta[:, 1], delta[:, 0])
delta = trajectory[-1] - trajectory[-2]
angles[-1] = np.arctan2(delta[1], delta[0])
delta = trajectory[1] - trajectory[0]
angles[0] = np.arctan2(delta[1], delta[0])
trajectory = np.concatenate([trajectory, angles[:, None]], axis=1)
benchmark.evaluate_and_save_results(trajectory, "gpmp2")
