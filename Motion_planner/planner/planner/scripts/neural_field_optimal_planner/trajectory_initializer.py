import numpy as np
import torch

from .torch_math import wrap_angle


class TrajectoryInitializer(object):
    def __init__(self, collision_checker, init_angles_with_trajectory=False):
        self._collision_checker = collision_checker
        self._init_angles_with_trajectory = init_angles_with_trajectory

    def initialize_trajectory(self, trajectory, start_point, goal_point):
        with torch.no_grad():
            trajectory_length = trajectory.shape[0] + 2
            trajectory[:, 0] = torch.linspace(
                start_point[0, 0], goal_point[0, 0], trajectory_length)[1:-1]
            trajectory[:, 1] = torch.linspace(
                start_point[0, 1], goal_point[0, 1], trajectory_length)[1:-1]
            self.initialize_angle(trajectory, start_point, goal_point)

    def initialize_angle(self, trajectory, start_point, goal_point):
        with torch.no_grad():
            trajectory_length = trajectory.shape[0] + 2
            delta_angle = wrap_angle(goal_point[0, 2] - start_point[0, 2])
            goal_angle = delta_angle + start_point[0, 2]
            trajectory[:, 2] = torch.linspace(
                start_point[0, 2], goal_angle, trajectory_length)[1:-1]
        if self._init_angles_with_trajectory:
            return self.initialize_angle_with_trajectory_direction(trajectory, start_point, goal_point)

    @staticmethod
    def initialize_angle_with_trajectory_direction(trajectory, start_point, goal_point):
        with torch.no_grad():
            full_trajectory = torch.cat([start_point, trajectory, goal_point], dim=0)
            x = full_trajectory[2:, 0] - full_trajectory[:-2, 0]
            y = full_trajectory[2:, 1] - full_trajectory[:-2, 1]
            angles = torch.atan2(y, x)
            weights = torch.cat([torch.linspace(0., 1, int((trajectory.shape[0]) // 2)),
                                 torch.linspace(1., 0, int((trajectory.shape[0] + 1) // 2))], dim=0)
            delta_angles = wrap_angle(angles - trajectory[:, 2]) * weights
            trajectory[:, 2] = trajectory[:, 2] + delta_angles
            print(angles)
            print(goal_point[0, 2])