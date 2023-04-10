import rospy
import tf2_ros

from ..utils.position2 import Position2


class TransformReceiver(object):
    def __init__(self, tf_buffer, base_frame, parent_frame):
        self._buffer = tf_buffer
        self._base_frame = base_frame
        self._parent_frame = parent_frame

    def get_transform(self):
        result = self._get_transform(self._buffer, self._base_frame, self._parent_frame, rospy.Time(0))
        while result is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
            result = self._get_transform(self._buffer, self._base_frame, self._parent_frame, rospy.Time(0))
        return result

    @staticmethod
    def _get_transform(buffer_, child_frame, parent_frame, stamp):
        try:
            transform = buffer_.lookup_transform(parent_frame, child_frame, stamp)
            return Position2.from_ros_transform(transform.transform)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logwarn(f"[TransformReceiver]- Get transform from {child_frame} to {parent_frame} at "
                          f"stamp {stamp} has problem " + str(msg))
            return None

    def _get_transform_with_timestamp(self):
        try:
            transform = self._buffer.lookup_transform(self._parent_frame, self._base_frame, rospy.Time(0))
            return Position2.from_ros_transform(transform.transform), transform.header.stamp.to_sec()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as msg:
            rospy.logdebug(str(msg))
            return None

    def get_transform_with_timestamp(self):
        result = self._get_transform_with_timestamp()
        while result is None and not rospy.is_shutdown():
            rospy.sleep(0.1)
            result = self._get_transform_with_timestamp()
        return result
