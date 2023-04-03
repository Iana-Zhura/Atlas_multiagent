import numpy as np
import scipy.interpolate

from ..utils.math import unfold_angles, wrap_angles
from ..utils.position2 import Position2


class PathPostprocessor(object):
    def __init__(self, minimal_distance=0.001, distance_step=0.05):
        self._distance_step = distance_step
        self._minimal_distance = minimal_distance

    def process(self, trajectory):
        if len(trajectory) < 3:
            return trajectory
        trajectory = trajectory.as_vec()
        trajectory = self._filter_trajectory(trajectory)
        parametrization = self._calculate_parametrization(trajectory)
        total_distance = self._calculate_total_distance(trajectory)
        point_count = int(total_distance / self._distance_step)
        new_parametrization = np.linspace(0, 1, point_count)
        trajectory = self._reparametrize_trajectory(trajectory, parametrization, new_parametrization)
        minimal_filter_index = self._find_minimal_filter_index(trajectory)
        return Position2.from_vec(trajectory[minimal_filter_index:])

    @staticmethod
    def _calculate_total_distance(trajectory):
        distances = np.linalg.norm(trajectory[1:, :2] - trajectory[:-1, :2], axis=1) + 1e-6
        return np.sum(distances)

    @staticmethod
    def _calculate_parametrization(trajectory):
        distances = np.linalg.norm(trajectory[1:, :2] - trajectory[:-1, :2], axis=1) + 1e-6
        cum_distances = np.cumsum(distances)
        cum_distances = np.concatenate([np.zeros(1), cum_distances], axis=0)
        return cum_distances / cum_distances[-1]

    def _filter_trajectory(self, trajectory):
        previous_point = trajectory[-1]
        result = [trajectory[-1]]
        for x in reversed(trajectory[1:-1]):
            distance = np.linalg.norm(previous_point[:2] - x[:2])
            if distance > self._minimal_distance:
                result.append(x)
                previous_point = x
        result.append(trajectory[0])
        return np.array(list(reversed(result)))

    @staticmethod
    def _reparametrize_trajectory(trajectory, old_parametrization, new_parametrization):
        trajectory[:, 2] = unfold_angles(trajectory[:, 2])
        interpolated_trajectory = scipy.interpolate.interp1d(old_parametrization, trajectory, kind="quadratic", axis=0,
                                                             fill_value="extrapolate")
        return interpolated_trajectory(new_parametrization)

    def _find_minimal_filter_index(self, trajectory):
        directions = self._find_directions(trajectory)
        minimal_index = 1
        if len(directions) > 0:
            other_direction = np.nonzero(directions != directions[0])[0]
            if len(other_direction) > 0 and other_direction[0] < 6:
                minimal_index = max(other_direction[0], minimal_index)
        return minimal_index

    @staticmethod
    def _find_directions(trajectory):
        delta = trajectory[1:, :2] - trajectory[:-1, :2]
        mean_angle = trajectory[:-1, 2] + wrap_angles(trajectory[1:, 2] - trajectory[:-1, 2]) / 2
        return np.cos(mean_angle) * delta[:, 0] + np.sin(mean_angle) * delta[:, 1] > 0
