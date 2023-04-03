import threading
import time

import geometry_msgs.msg
import nav_msgs.msg
import rospy

from ..utils.position2 import Position2


class GoalPlannerAdapter(object):
    def __init__(self, planner, map_adapter, robot_state, goal_topic_name, path_topic_name, planning_timeout,
                 planner_rate, is_point=True, result_visualizer=None, path_postprocessor=None):
        self._planner = planner
        self._map_adapter = map_adapter
        self._robot_state = robot_state
        self._is_planning = False
        self._planning_timeout = planning_timeout
        self._is_point = is_point
        self._mutex = threading.Lock()
        self._path_publisher = rospy.Publisher(path_topic_name, nav_msgs.msg.Path, queue_size=1)
        self._planner_timer = rospy.Timer(rospy.Duration(1 / planner_rate), self._planner_timer_callback)
        self._goal_subscriber = rospy.Subscriber(goal_topic_name, geometry_msgs.msg.PoseStamped, self._callback)
        self._result_visualizer = result_visualizer
        self._path_postprocessor = path_postprocessor

    def _callback(self, message):
        start_point = self._point_representation(self._robot_state.position)
        goal_point = self._point_representation(Position2.from_ros_pose(message.pose))
        boundaries = self._map_adapter.boundaries
        if boundaries is None:
            rospy.logwarn("[GoalPlannerAdapter] - Boundaries is None, map is not yet received")
            rospy.logwarn("[GoalPlannerAdapter] - Planning goal is skipped")
            return
        with self._mutex:
            self._planner.init(start_point, goal_point, boundaries)
            self._is_planning = True

    def _point_representation(self, position):
        if self._is_point:
            return position.translation
        return position.as_vec()

    def _planner_timer_callback(self, _):
        with self._mutex:
            if not self._is_planning:
                return
            start_point = self._point_representation(self._robot_state.position)
            self._planner.update_start_point(start_point)
            start_planning_time = time.time()
            while time.time() - start_planning_time < self._planning_timeout:
                self._planner.step()
            path = self._planner.get_path()
        if self._is_point:
            path = [Position2.from_vec([x[0], x[1], 0]) for x in path]
        else:
            path = [Position2.from_vec([x[0], x[1], x[2]]) for x in path]
        path = Position2.from_array(path)
        if self._path_postprocessor is not None:
            path = self._path_postprocessor.process(path)
        self._publish_path(path)
        if self._result_visualizer is not None:
            self._result_visualizer.publish_result(path)

    def _publish_path(self, path):
        message = nav_msgs.msg.Path()
        message.header.frame_id = "map"
        message.header.stamp = rospy.Time().now()
        message.poses = [geometry_msgs.msg.PoseStamped(pose=x.as_ros_pose()) for x in path.as_array()]
        self._path_publisher.publish(message)
