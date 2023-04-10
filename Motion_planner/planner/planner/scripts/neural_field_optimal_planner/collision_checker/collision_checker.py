import numpy as np


class CollisionChecker(object):
    def __init__(self, collision_boundaries=None):
        self._obstacle_points = np.zeros((0, 2))
        self._boundaries = collision_boundaries

    def check_collision(self, test_positions):
        return self._check_boundaries_collision(test_positions)

    def _check_boundaries_collision(self, test_positions):
        if self._boundaries is None:
            return False
        result = test_positions[:, 0] > self._boundaries[1]
        result |= test_positions[:, 0] < self._boundaries[0]
        result |= test_positions[:, 1] > self._boundaries[3]
        result |= test_positions[:, 1] < self._boundaries[2]
        return result

    def update_obstacle_points(self, points):
        self._obstacle_points = points

    def update_boundaries(self, boundaries):
        self._boundaries = boundaries

    def get_boundaries(self):
        return self._boundaries
