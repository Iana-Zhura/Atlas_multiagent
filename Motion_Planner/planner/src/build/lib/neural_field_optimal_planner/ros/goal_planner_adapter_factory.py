from .collision_checker_adapter import CollisionCheckerAdapter
from .goal_planner_adapter import GoalPlannerAdapter
from .map_adapter import MapAdapter
from .path_postprocessor import PathPostprocessor
from .robot_state import RobotState
from .transform_receiver_factory import TransformReceiverFactory
from ..collision_checker import CircleDirectedCollisionChecker, RectangleCollisionChecker
from ..planner_factory import PlannerFactory
from .planner_result_visualizer import PlannerResultVisualizer


class GoalPlannerAdapterFactory(object):
    @staticmethod
    def make_goal_planner_adapter():
        transform_receiver_factory = TransformReceiverFactory()
        robot_state = RobotState(transform_receiver_factory)
        map_adapter = MapAdapter(map_topic_name="map")
        collision_checker = CircleDirectedCollisionChecker(robot_radius=0.3)
        # collision_checker = RectangleCollisionChecker((-0.34, 0.4, -0.27, 0.27))
        # collision_checker = RectangleCollisionChecker((-0.2, 0.2, -0.2, 0.2))
        collision_checker_adapter = CollisionCheckerAdapter(collision_checker, point_topic_name="obstacle_points",
                                                            map_adapter=map_adapter)
        # planner = PlannerFactory.make_onf_planner(collision_checker_adapter)
        planner = PlannerFactory.make_constrained_onf_planner(collision_checker_adapter)
        result_visualizer = PlannerResultVisualizer("pytorch_motion_planner_visualization")
        path_postprocessor = PathPostprocessor()
        return GoalPlannerAdapter(planner, map_adapter, robot_state, goal_topic_name="/move_base_simple/goal",
                                  path_topic_name="/path", planning_timeout=0.1, planner_rate=10, is_point=False,
                                  result_visualizer=result_visualizer, path_postprocessor=path_postprocessor)
