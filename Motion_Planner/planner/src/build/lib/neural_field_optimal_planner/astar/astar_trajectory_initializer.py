import numpy as np
import torch

from .jps import JPS
from ..trajectory_initializer import TrajectoryInitializer
from ..utils.math import reparametrize_path
from ..utils.position2 import Position2


class AstarTrajectoryInitializer(TrajectoryInitializer):
    def __init__(self, collision_checker, resolution, init_angles_with_trajectory=False):
        super().__init__(collision_checker, init_angles_with_trajectory)
        self._resolution = resolution

    def initialize_trajectory(self, trajectory, start_point, goal_point):
        start = start_point[0].detach().cpu().numpy()
        goal = goal_point[0].detach().cpu().numpy()
        path = self.calculate_astar_path(start, goal)
        trajectory_length = trajectory.shape[0] + 2
        reparametrized_path = reparametrize_path(np.concatenate([start[None, :2], path, goal[None, :2]], axis=0),
                                                 trajectory_length)
        with torch.no_grad():
            trajectory[:, :2] = torch.tensor(reparametrized_path[1:-1].astype(np.float32), device=trajectory.device)
        self.initialize_angle(trajectory, start_point, goal_point)

    def calculate_astar_path(self, start, goal):
        boundaries = self._collision_checker.get_boundaries()
        x_cells = int((boundaries[1] - boundaries[0]) // self._resolution) + 1
        y_cells = int((boundaries[3] - boundaries[2]) // self._resolution) + 1
        start_x_cell = int((start[0] - boundaries[0]) // self._resolution)
        start_y_cell = int((start[1] - boundaries[2]) // self._resolution)
        goal_x_cell = int((goal[0] - boundaries[0]) // self._resolution)
        goal_y_cell = int((goal[1] - boundaries[2]) // self._resolution)
        x, y = np.meshgrid(range(x_cells), range(y_cells))
        x = x.reshape(-1) * self._resolution + self._resolution / 2 + boundaries[0]
        y = y.reshape(-1) * self._resolution + self._resolution / 2 + boundaries[2]
        positions = Position2(x, y, np.ones_like(x) * 3 * np.pi / 4)
        collisions = self._collision_checker.check_collision(positions)
        matrix = collisions.reshape(y_cells, x_cells)
        matrix[goal_y_cell, goal_x_cell] = False
        planner = JPS(matrix, jps=False)
        path = planner.find_path((start_y_cell, start_x_cell), (goal_y_cell, goal_x_cell))
        final_path = np.zeros_like(path).astype(np.float32)
        final_path[:, 0] = path[:, 1] * self._resolution + self._resolution / 2 + boundaries[0]
        final_path[:, 1] = path[:, 0] * self._resolution + self._resolution / 2 + boundaries[2]
        return final_path
