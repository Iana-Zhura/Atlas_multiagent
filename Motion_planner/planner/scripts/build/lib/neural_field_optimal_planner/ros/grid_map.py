import numpy as np

from ..utils.position2 import Position2


class GridMap(object):
    def __init__(self, data, resolution, origin, threshold=0.5):
        self._map = data
        self._resolution = resolution
        self._origin = origin
        self._threshold = threshold
        self._points = None

    def as_point_cloud(self):
        if self._points is not None:
            return self._points
        indices = np.array(np.nonzero(self._map > self._threshold)).T[:, ::-1]
        points = indices * self._resolution + np.ones(2) * self._resolution / 2.
        self._points = self._origin.apply(points)
        return self._points

    @property
    def boundaries(self):
        height, width = self._map.shape
        bottom = self._origin.y
        left = self._origin.x
        top = bottom + height * self._resolution
        right = left + width * self._resolution
        return left, right, bottom, top

    @classmethod
    def from_ros_occupancy_grid(cls, message):
        width = message.info.width
        height = message.info.height
        resolution = message.info.resolution
        origin = Position2.from_ros_pose(message.info.origin)
        map_img = np.array(message.data).reshape(height, width)
        map_img = np.where(map_img == -1, 0, map_img)
        map_img = map_img.astype(np.float32) / 100
        return cls(map_img, resolution, origin)
