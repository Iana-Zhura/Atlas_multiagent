import numpy as np

from ..collision_checker import CollisionChecker


class BenchmarkCollisionChecker(CollisionChecker):
    def __init__(self, benchmark, boundaries=None):
        super().__init__(boundaries)
        self._benchmark = benchmark

    def check_collision(self, test_positions):
        return self._benchmark.is_collision(test_positions) | super().check_collision(test_positions.as_vec())
