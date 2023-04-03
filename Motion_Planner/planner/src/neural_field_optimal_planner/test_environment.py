class TestEnvironment(object):
    def __init__(self, start_point, goal_point, bounds, obstacle_points):
        self.start_point = start_point
        self.goal_point = goal_point
        self.bounds = bounds
        self.obstacle_points = obstacle_points
