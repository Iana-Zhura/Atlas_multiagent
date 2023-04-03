import rospy
import visualization_msgs.msg


class PlannerResultVisualizer(object):
    def __init__(self, topic_name):
        self._publisher = rospy.Publisher(topic_name, visualization_msgs.msg.MarkerArray, queue_size=1)

    def publish_result(self, path):
        markers = self._make_path_markers(path)
        self._publisher.publish(visualization_msgs.msg.MarkerArray(markers))

    def _make_path_markers(self, path):
        return [self._make_arrow_marker(point, i) for i, point in enumerate(path.as_array())]

    @staticmethod
    def _make_arrow_marker(point, index):
        marker = visualization_msgs.msg.Marker()
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns = "path"
        marker.id = index
        marker.type = 0
        marker.pose = point.as_ros_pose()
        marker.color.r = 1
        marker.color.g = 0
        marker.color.b = 0
        marker.color.a = 1
        marker.lifetime = rospy.Duration(1)
        marker.scale.x = 0.2
        marker.scale.y = 0.03
        marker.scale.z = 0.03
        return marker
