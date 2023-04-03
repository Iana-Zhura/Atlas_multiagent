import numpy as np
import scipy.interpolate


def calculate_curvature(x, y, t):
    dx = np.gradient(x, t)
    dy = np.gradient(y, t)
    d2x = np.gradient(dx, t)
    d2y = np.gradient(dy, t)
    return (dx * d2y - dy * d2x) / (dx ** 2 + dy ** 2) ** (3 / 2)


def find_orthogonal_projection(trajectory, point, parametrization):
    distances = np.linalg.norm(trajectory[:, :2] - point[:2], axis=1)
    index = np.argmin(distances)
    minimal_distance = np.min(distances)
    trajectory_delta = trajectory[1:] - trajectory[:-1]
    dx = trajectory_delta[:, 0]
    dy = trajectory_delta[:, 1]
    scalar_product = (point[0] - trajectory[:-1, 0]) * dx + (point[1] - trajectory[:-1, 1]) * dy
    coefficients = scalar_product / (np.linalg.norm(trajectory_delta, axis=1) ** 2 + 1e-6)
    mask = (coefficients > 0) & (coefficients < 1)
    projections = trajectory[:-1] + coefficients[:, None] * trajectory_delta
    projection_distances = np.linalg.norm(projections[:, :2] - point[:2], axis=1)
    projection_distances = np.where(mask, projection_distances, np.inf)
    minimal_projection_index = np.argmin(projection_distances)
    minimal_projection_distance = np.min(projection_distances)
    if minimal_distance < minimal_projection_distance:
        return parametrization[index]
    parametrization_delta = parametrization[minimal_projection_index + 1] - parametrization[minimal_projection_index]
    return parametrization[minimal_projection_index] + coefficients[minimal_projection_index] * parametrization_delta


def wrap_angles(angles):
    return (angles + np.pi) % (2 * np.pi) - np.pi


def unfold_angles(angles):
    angles = wrap_angles(angles)
    delta = angles[1:] - angles[:-1]
    delta = np.where(delta > np.pi, delta - 2 * np.pi, delta)
    delta = np.where(delta < -np.pi, delta + 2 * np.pi, delta)
    return angles[0] + np.concatenate([np.zeros(1), np.cumsum(delta)], axis=0)


def calculate_tangent(x, y, t):
    dx = np.gradient(x, t)
    dy = np.gradient(y, t)
    return unfold_angles(np.arctan2(dy, dx))


def sinc(x, epsilon=1e-4):
    x = np.where(np.abs(x) > epsilon, x, np.sign(x) * epsilon)
    return np.sin(x) / x


def reparametrize_path(path, point_count):
    distances = np.linalg.norm(path[1:] - path[:-1], axis=1) + 1e-6
    cum_distances = np.cumsum(distances)
    cum_distances = np.concatenate([np.zeros(1), cum_distances], axis=0)
    parametrization = cum_distances / cum_distances[-1]
    new_parametrization = np.linspace(0, 1, point_count)
    interpolated_trajectory = scipy.interpolate.interp1d(parametrization, path, kind="quadratic", axis=0,
                                                         fill_value="extrapolate")
    return interpolated_trajectory(new_parametrization)