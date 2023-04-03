import numpy as np

from neural_field_optimal_planner.collision_checker import CollisionChecker


class RectangleCollisionChecker(CollisionChecker):
    def __init__(self, box, collision_boundaries=None):
        super().__init__(collision_boundaries)
        self._box = box

    def check_collision(self, test_positions):
        x, y = self._calculate_transformed_obstacle_points(test_positions)
        result = x > self._box[0]
        result &= x < self._box[1]
        result &= y > self._box[2]
        result &= y < self._box[3]
        return np.any(result, axis=1) | self._check_boundaries_collision(test_positions.translation)

    def _calculate_transformed_obstacle_points(self, positions):
        positions = positions.inv()
        x, y = self._obstacle_points.T
        x1 = x[None, :] * np.cos(positions.rotation[:, None]) - y[None, :] * np.sin(
            positions.rotation[:, None]) + positions.x[:, None]
        y1 = x[None, :] * np.sin(positions.rotation[:, None]) + y[None, :] * np.cos(
            positions.rotation[:, None]) + positions.y[:, None]
        return x1, y1
