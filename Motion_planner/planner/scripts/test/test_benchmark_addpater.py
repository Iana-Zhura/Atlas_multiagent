import numpy as np

import neural_field_optimal_planner.benchmark_adapter
import unittest

from neural_field_optimal_planner.utils.position2 import Position2


class TestBenchmarkAdapter(unittest.TestCase):
    def setUp(self) -> None:
        self._benchmark_adapter = neural_field_optimal_planner.benchmark_adapter.BenchmarkAdapter(
            "test_benchmark/2022-01-14_17-19-42_config.json")

    def test_init(self):
        self.assertIsNotNone(self._benchmark_adapter)

    def test_is_collision_true(self):
        test_position = Position2.from_vec(np.array([[10, -70, 0]]))
        collision = self._benchmark_adapter.is_collision(test_position)
        self.assertTrue(collision)

    def test_is_collision_false(self):
        test_position = Position2.from_vec(np.array([[20, -50, 0]]))
        collision = self._benchmark_adapter.is_collision(test_position)
        self.assertFalse(collision)

    def test_bounds(self):
        bounds = self._benchmark_adapter.bounds()
        self.assertAlmostEqual(bounds[0], 0.03, 1)
        self.assertAlmostEqual(bounds[1], 124.4, 1)
        self.assertAlmostEqual(bounds[2], -81.26, 1)
        self.assertAlmostEqual(bounds[3], -0.03, 1)

    def test_evaluate_and_save_result(self):
        path = np.array([[0, 0, 0], [10, -10, 0]])
        self._benchmark_adapter.evaluate_and_save_results(path, "test_planner")

    def test_start(self):
        start = self._benchmark_adapter.start()
        self.assertEqual(start.x, 7.5)
        self.assertEqual(start.y, -10)
        self.assertEqual(start.rotation, -1.58)

    def test_goal(self):
        goal = self._benchmark_adapter.goal()
        self.assertEqual(goal.x, 116)
        self.assertEqual(goal.y, -70)
        self.assertEqual(goal.rotation, -1.58)


