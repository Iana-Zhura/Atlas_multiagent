from .circle_collision_checker import CircleCollisionChecker


class CircleDirectedCollisionChecker(CircleCollisionChecker):
    def check_collision(self, test_positions):
        return super().check_collision(test_positions.translation)
