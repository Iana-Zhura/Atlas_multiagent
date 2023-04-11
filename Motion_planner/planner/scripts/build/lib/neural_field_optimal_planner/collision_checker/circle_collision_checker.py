import numpy as np

from neural_field_optimal_planner.collision_checker import CollisionChecker


class CircleCollisionChecker(CollisionChecker):
    def __init__(self, robot_radius, boundaries=None):
        super().__init__(boundaries)
        self._robot_radius = robot_radius

    def check_collision(self, test_positions):
        distances = np.linalg.norm(test_positions[None] - self._obstacle_points[:, None], axis=2)
        result = np.any(distances < self._robot_radius, axis=0)
        return result | self._check_boundaries_collision(test_positions)
