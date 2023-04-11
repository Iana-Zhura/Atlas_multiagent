import abc


class ContinuousPlanner(object):
    @abc.abstractmethod
    def init(self, start_point, goal_point, boundaries):
        raise NotImplementedError()

    @abc.abstractmethod
    def step(self):
        raise NotImplementedError()

    @abc.abstractmethod
    def get_path(self):
        raise NotImplementedError()

    @abc.abstractmethod
    def set_boundaries(self, boundaries):
        raise NotImplementedError()

    @abc.abstractmethod
    def update_goal_point(self, goal_point):
        raise NotImplementedError()

    @abc.abstractmethod
    def update_start_point(self, start_point):
        raise NotImplementedError()
