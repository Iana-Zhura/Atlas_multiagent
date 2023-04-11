import numpy as np
import torch
import torch.nn as nn
from torch.nn import functional

from .continuous_planner import ContinuousPlanner
from .utils.timer import timer


class NERFOptPlanner(ContinuousPlanner):
    def __init__(self, trajectory, collision_model, collision_checker, collision_optimizer, trajectory_optimizer,
                 trajectory_random_offset, collision_weight, velocity_hessian_weight, init_collision_iteration=100,
                 init_collision_points=100, reparametrize_trajectory_freq=10, optimize_collision_model_freq=1,
                 random_field_points=10, collision_loss_koef=1, course_random_offset=1.5, collision_point_count=100):
        torch.autograd.set_detect_anomaly(True)
        self._trajectory = trajectory
        device = self._trajectory.device
        self._goal_point = torch.zeros(1, 2, device=device)
        self._start_point = torch.zeros(1, 2, device=device)
        self._device = device
        self._collision_model = collision_model
        self._collision_checker = collision_checker
        self._collision_optimizer = collision_optimizer
        self._trajectory_optimizer = trajectory_optimizer
        self._collision_loss_function = nn.BCEWithLogitsLoss()
        self._random_sample_border = (0, 0, 0, 0)
        self._fine_random_offset = trajectory_random_offset
        self._collision_weight = collision_weight
        self._inv_hessian = self._calculate_inv_hessian(self._trajectory.shape[0], velocity_hessian_weight)
        self._init_collision_iteration = init_collision_iteration
        self._init_collision_points = init_collision_points
        self._reparametrize_trajectory_freq = reparametrize_trajectory_freq
        self._optimize_collision_model_freq = optimize_collision_model_freq
        self._random_field_points = random_field_points
        self._step_count = 0
        self._collision_loss_koef = collision_loss_koef
        self._previous_trajectory = None
        self._collision_positions = np.zeros((0, 2))
        self._collision_positions_times = np.zeros(0)
        self._course_random_offset = course_random_offset
        self._collision_point_count = collision_point_count
        self.checked_positions = np.zeros((0, 3))
        self.truth_collision = np.zeros(0, dtype=np.bool)

    def _calculate_inv_hessian(self, point_count, velocity_hessian_weight):
        hessian = velocity_hessian_weight * self._calculate_velocity_hessian(point_count) + np.eye(point_count)
        inv_hessian = np.linalg.inv(hessian)
        return torch.tensor(inv_hessian.astype(np.float32), device=self._device)

    @staticmethod
    def _calculate_velocity_hessian(point_count):
        hessian = np.zeros((point_count, point_count), dtype=np.float32)
        for i in range(point_count):
            hessian[i, i] = 4
            if i > 0:
                hessian[i, i - 1] = -2
                hessian[i - 1, i] = -2
        return hessian

    def step(self):
        timer.tick("step")
        if self._step_count % self._optimize_collision_model_freq == 0:
            self._optimize_collision_model()
        self._optimize_trajectory()
        timer.tick("reparametrize_trajectory")
        if self._step_count % self._reparametrize_trajectory_freq == 0:
            with torch.no_grad():
                self.reparametrize_trajectory()
        timer.tock("reparametrize_trajectory")
        self._step_count += 1
        timer.tock("step")

    def full_trajectory(self):
        return torch.cat([self._start_point, self._trajectory, self._goal_point], dim=0)

    def _optimize_collision_model(self, positions=None):
        timer.tick("optimize_collision_model")
        if positions is None:
            if self._previous_trajectory is None:
                self._previous_trajectory = self._trajectory.detach().clone()
            positions = self._sample_collision_checker_points(self._previous_trajectory)
            self._previous_trajectory = self._trajectory.detach().clone()
        self._collision_model.requires_grad_(True)
        self._collision_optimizer.zero_grad()
        predicted_collision = self._calculate_predicted_collision(positions)
        truth_collision = self._calculate_truth_collision(positions)
        truth_collision = torch.tensor(truth_collision.astype(np.float32)[:, None], device=self._device)
        loss = self._collision_loss_function(predicted_collision, truth_collision)
        loss.backward()
        self._collision_optimizer.step()
        timer.tock("optimize_collision_model")

    def _calculate_truth_collision(self, positions):
        self.checked_positions = positions.copy()
        self.truth_collision = self._collision_checker.check_collision(positions)
        return self.truth_collision

    def _calculate_predicted_collision(self, positions):
        return self._collision_model(torch.tensor(positions.astype(np.float32), device=self._device))

    def _sample_collision_checker_points(self, trajectory=None):
        positions = self._random_intermediate_positions(trajectory).detach().cpu().numpy()
        course_positions = self._offset_positions(positions, self._course_random_offset)
        fine_positions = self._offset_positions(positions, self._fine_random_offset)
        times = np.concatenate([self._collision_positions_times, np.zeros(len(fine_positions))], axis=0)
        positions = np.concatenate([self._collision_positions, fine_positions], axis=0)
        self._collision_positions, self._collision_positions_times = self._resample_collision_positions(positions,
                                                                                                        times)
        return np.concatenate(
            [course_positions, self._collision_positions, self._sample_random_field_points(self._random_field_points)],
            axis=0)

    def _random_intermediate_positions(self, trajectory=None):
        if trajectory is None:
            trajectory = self._trajectory
        t = torch.tensor(np.random.rand(trajectory.shape[0] - 1).astype(np.float32), device=self._device)[:, None]
        return trajectory[1:] * (1 - t) + trajectory[:-1] * t

    def _offset_positions(self, positions, offset):
        return positions + np.random.randn(positions.shape[0], 2) * offset

    def _resample_collision_positions(self, positions, times):
        with torch.no_grad():
            predicted_collision = self._calculate_predicted_collision(positions)
            weights = torch.sigmoid(predicted_collision).detach().cpu().numpy()[:, 0]
            weights = weights * np.exp(-times * 0.03) + 1e-6
            weights = weights / np.sum(weights)
        if len(positions) < self._collision_point_count:
            return positions, times
        replace = np.count_nonzero(weights > 1e-6) < self._collision_point_count
        indices = np.random.choice(len(positions), self._collision_point_count, replace=replace, p=weights)
        times = times + 1
        return positions[indices], times[indices]

    def _sample_random_field_points(self, points_count):
        random_points = np.random.rand(points_count, 2)
        random_points[:, 0] = self._random_sample_border[0] + random_points[:, 0] * (
                self._random_sample_border[1] - self._random_sample_border[0])
        random_points[:, 1] = self._random_sample_border[2] + random_points[:, 1] * (
                self._random_sample_border[3] - self._random_sample_border[2])
        return random_points

    def _optimize_trajectory(self):
        self._collision_model.requires_grad_(False)
        self._trajectory_optimizer.zero_grad()
        loss = self.trajectory_loss()
        timer.tick("trajectory_backward")
        loss.backward()
        timer.tock("trajectory_backward")
        timer.tick("inv_hes_grad_multiplication")
        self._trajectory.grad = self._inv_hessian @ self._trajectory.grad
        timer.tock("inv_hes_grad_multiplication")
        timer.tick("trajectory_optimizer_step")
        self._trajectory_optimizer.step()
        timer.tock("trajectory_optimizer_step")

    def trajectory_loss(self):
        collision_positions = self._random_intermediate_positions()
        return self.distance_loss() + self.trajectory_collision_loss(collision_positions) * self._collision_weight

    def distance_loss(self):
        full_trajectory = self.full_trajectory()
        delta = full_trajectory[1:] - full_trajectory[:-1]
        return torch.sum(delta ** 2)

    def trajectory_collision_loss(self, collision_positions):
        collision_probabilities = self._collision_model(collision_positions)
        collision_probabilities = nn.functional.softplus(collision_probabilities)
        return torch.sum(collision_probabilities)

    def boundary_loss(self):
        loss = torch.relu(-self._trajectory[:, 0] + self._random_sample_border[0]) ** 2
        loss.add_(torch.relu(self._trajectory[:, 0] - self._random_sample_border[1]) ** 2)
        loss.add_(torch.relu(-self._trajectory[:, 1] + self._random_sample_border[2]) ** 2)
        loss.add_(torch.relu(self._trajectory[:, 1] - self._random_sample_border[3]) ** 2)
        return torch.sum(loss)

    def get_path(self):
        return self.full_trajectory().detach().cpu().numpy()

    def init(self, start_point, goal_point, boundaries):
        self._start_point = torch.tensor(start_point.astype(np.float32), device=self._device)[None]
        self._goal_point = torch.tensor(goal_point.astype(np.float32), device=self._device)[None]
        self._random_sample_border = boundaries
        self._init_trajectory()
        self._init_collision_model()
        self._step_count = 0

    def _init_trajectory(self):
        with torch.no_grad():
            trajectory_length = self._trajectory.shape[0] + 2
            self._trajectory[:, 0] = torch.linspace(
                self._start_point[0, 0], self._goal_point[0, 0], trajectory_length)[1:-1]
            self._trajectory[:, 1] = torch.linspace(
                self._start_point[0, 1], self._goal_point[0, 1], trajectory_length)[1:-1]

    def _init_collision_model(self):
        for i in range(self._init_collision_iteration):
            positions = self._sample_random_field_points(self._init_collision_points)
            self._optimize_collision_model(positions)

    def update_goal_point(self, goal_point):
        self._goal_point = torch.tensor(goal_point.astype(np.float32), device=self._device)[None]
        with torch.no_grad():
            delta = torch.sum((self._trajectory - self._goal_point) ** 2, dim=1)
            min_index = torch.argmin(delta)
            self._trajectory.data[min_index:] = self._goal_point
            self.reparametrize_trajectory()
        self._step_count = 0

    def update_start_point(self, start_point):
        self._start_point = torch.tensor(start_point.astype(np.float32), device=self._device)[None]
        with torch.no_grad():
            delta = torch.sum((self._trajectory - self._start_point) ** 2, dim=1)
            min_index = torch.argmin(delta)
            self._trajectory.data[:min_index] = self._start_point
            self.reparametrize_trajectory()
        self._step_count = 0

    def set_boundaries(self, boundaries):
        self._random_sample_border = boundaries
        self._step_count = 0

    def reparametrize_trajectory(self):
        full_trajectory = self.full_trajectory()
        distances = self._calculate_distances()
        normalized_distances = distances / torch.sum(distances)
        cdf = torch.cumsum(normalized_distances, dim=0)
        cdf = torch.cat([torch.zeros(1, device=self._device), cdf], dim=0)
        uniform_samples = torch.linspace(0, 1, len(full_trajectory), device=self._device)[1:-1]
        indices = torch.searchsorted(cdf, uniform_samples)
        index_above = torch.where(indices > len(full_trajectory) - 1, len(full_trajectory) - 1, indices)
        index_bellow = torch.where(indices - 1 < 0, 0, indices - 1)
        cdf_above = torch.gather(cdf, 0, index_above)
        cdf_bellow = torch.gather(cdf, 0, index_bellow)
        index_above = torch.repeat_interleave(index_above[:, None], self._trajectory.shape[1], dim=1)
        index_bellow = torch.repeat_interleave(index_bellow[:, None], self._trajectory.shape[1], dim=1)
        trajectory_above = torch.gather(full_trajectory, 0, index_above)
        trajectory_bellow = torch.gather(full_trajectory, 0, index_bellow)
        denominator = cdf_above - cdf_bellow
        denominator = torch.where(denominator < 1e-5, torch.ones_like(denominator) * 1e-5, denominator)
        t = (uniform_samples - cdf_bellow) / denominator
        trajectory = (1 - t[:, None]) * trajectory_bellow + t[:, None] * trajectory_above
        self._trajectory.data = trajectory

    def _calculate_distances(self):
        full_trajectory = self.full_trajectory()
        return torch.norm(full_trajectory[1:] - full_trajectory[:-1], dim=1)
