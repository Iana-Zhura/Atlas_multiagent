import numpy as np
import rospy

import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


class CollisionCheckerAdapter(object):
    def __init__(self, collision_checker, point_topic_name, map_adapter=None):
        self._map_adapter = map_adapter
        self._collision_checker = collision_checker
        self._subscriber = rospy.Subscriber(point_topic_name, PointCloud2, self._callback)

    def check_collision(self, test_positions):
        return self._collision_checker.check_collision(test_positions)

    def _callback(self, message):
        obstacle_points = np.array([point for point in pc2.read_points(message)])
        if len(obstacle_points) == 0:
            obstacle_points = np.zeros((0, 2))
        else:
            obstacle_points = obstacle_points[:, :2]
        if self._map_adapter is not None:
            map_point_points = self._map_adapter.point_cloud
            obstacle_points = np.concatenate([obstacle_points, map_point_points], axis=0)
        self._collision_checker.update_obstacle_points(obstacle_points)
        self._collision_checker.update_boundaries(self._map_adapter.boundaries)
