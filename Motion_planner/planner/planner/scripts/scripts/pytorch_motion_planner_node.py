#!/usr/bin/env python
import rospy

from neural_field_optimal_planner.ros.goal_planner_adapter_factory import GoalPlannerAdapterFactory

if __name__ == '__main__':
    try:
        rospy.init_node("local_planner")
        movement_simulation_adapter = GoalPlannerAdapterFactory().make_goal_planner_adapter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
