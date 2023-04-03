import threading

import nav_msgs.msg
import numpy as np
import rospy

from .grid_map import GridMap


class MapAdapter(object):
    def __init__(self, map_topic_name):
        self._map = None
        self._mutex = threading.Lock()
        self._map_subscriber = rospy.Subscriber(map_topic_name, nav_msgs.msg.OccupancyGrid, self._callback)

    def _callback(self, message):
        with self._mutex:
            self._map = GridMap.from_ros_occupancy_grid(message)

    @property
    def point_cloud(self):
        with self._mutex:
            if self._map is None:
                return np.zeros((0, 2))
            return self._map.as_point_cloud()

    @property
    def boundaries(self):
        with self._mutex:
            if self._map is None:
                return None
            return self._map.boundaries
