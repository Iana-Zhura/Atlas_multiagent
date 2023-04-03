import numpy as np
from pybench_mr import BenchmarkAdapterImpl, Position
from ..utils.position2 import Position2


class BenchmarkAdapter(object):
    def __init__(self, config_path):
        self._benchmark_adapter_impl = BenchmarkAdapterImpl(config_path)

    def is_collision(self, positions):
        positions = [Position(x.x, x.y, x.rotation) for x in positions.as_array()]
        return np.array(self._benchmark_adapter_impl.collides_positions(positions))

    def bounds(self):
        return self._benchmark_adapter_impl.bounds()

    def evaluate_and_save_results(self, path, planner_name):
        path = [Position(x[0], x[1], x[2]) for x in path]
        return self._benchmark_adapter_impl.evaluateAndSaveResult(path, planner_name)

    def start(self):
        start_position = self._benchmark_adapter_impl.start()
        start_position = Position2(start_position.x, start_position.y, start_position.angle)
        return start_position

    def goal(self):
        goal_position = self._benchmark_adapter_impl.goal()
        goal_position = Position2(goal_position.x, goal_position.y, goal_position.angle)
        return goal_position

    def evaluate_path(self, path):
        path = [Position(x[0], x[1], x[2]) for x in path]
        return self._benchmark_adapter_impl.evaluatePath(path)
