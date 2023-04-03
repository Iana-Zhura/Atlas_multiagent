try:
    import geometry_msgs.msg
except ImportError:
    print("ROS is not sourced. To enable ROS features, please source it")
import numpy as np
from scipy.spatial.transform import Rotation


class Position2(object):
    def __init__(self, x, y, angle):
        self._x = x
        self._y = y
        self._angle = angle

    @property
    def rotation(self):
        return self._angle

    @property
    def translation(self):
        return np.array([self._x, self._y]).T

    @property
    def x(self):
        return self._x

    @property
    def y(self):
        return self._y

    @classmethod
    def from_vec(cls, vec):
        if isinstance(vec, list) or len(vec.shape) == 1:
            return cls(vec[0], vec[1], vec[2])
        return cls(vec[:, 0], vec[:, 1], vec[:, 2])

    def as_vec(self):
        return np.array([self._x, self._y, self._angle]).T

    @classmethod
    def from_array(cls, positions):
        x = np.array([x_.x for x_ in positions])
        y = np.array([x_.y for x_ in positions])
        angle = np.array([x_.rotation for x_ in positions])
        return cls(x, y, angle)

    def as_array(self):
        return [self.__class__(self._x[i], self._y[i], self._angle[i]) for i in range(len(self))]

    @classmethod
    def from_ros_pose(cls, message):
        return cls(message.position.x, message.position.y, cls._angle_from_ros_quaternion(message.orientation))

    def as_ros_pose(self):
        message = geometry_msgs.msg.Pose()
        message.position.x = self.x
        message.position.y = self.y
        message.orientation = self._ros_quaternion_from_angle(self.rotation)
        return message

    @classmethod
    def from_ros_transform(cls, message):
        return cls(message.translation.x, message.translation.y, cls._angle_from_ros_quaternion(message.rotation))

    @staticmethod
    def _angle_from_ros_quaternion(quaternion):
        q = [quaternion.x, quaternion.y, quaternion.z, quaternion.w]
        return Rotation.from_quat(q).as_euler("zxy", degrees=False)[0]

    @staticmethod
    def _ros_quaternion_from_angle(angle):
        q = Rotation.from_euler("xyz", np.array([0, 0, angle])).as_quat()
        q = geometry_msgs.msg.Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        return q

    @classmethod
    def global_from_local(cls, source_position, local_position):
        return source_position * local_position

    def __len__(self):
        if len(self._x.shape) == 0:
            return 1
        return self._x.shape[0]

    def __mul__(self, other):
        x1 = other.x * np.cos(self._angle) - other.y * np.sin(self._angle) + self._x
        y1 = other.x * np.sin(self._angle) + other.y * np.cos(self._angle) + self._y
        a1 = (other.rotation + self._angle + np.pi) % (2 * np.pi) - np.pi
        return self.__class__(x1, y1, a1)

    def inv(self):
        x = -self.x * np.cos(self.rotation) - self.y * np.sin(self.rotation)
        y = self.x * np.sin(self.rotation) - self.y * np.cos(self.rotation)
        return self.__class__(x, y, -self.rotation)

    def apply(self, points):
        x, y = points.T
        x1 = x * np.cos(self._angle) - y * np.sin(self._angle) + self._x
        y1 = x * np.sin(self._angle) + y * np.cos(self._angle) + self._y
        return np.stack([x1, y1], axis=1)
