import numpy as np

from .test_environment import TestEnvironment
from neural_field_optimal_planner.extract_obs import OBSTACLE_MAP
import os
from glob import glob

cur_path = os.getcwd()


class TestEnvironmentBuilder(object):
    def __init__(self) -> None:
        self.path_ply = 'results/release/semseg/sample200.ply'
        self.path_npz = 'results/release/semseg/sample200.npz'
        self.map = OBSTACLE_MAP(self.path_ply, self.path_npz)

    
    def get_map(self):
        return self.map.obstacle_map_adaptive_h, self.map.voxel_size, self.map.ROB_MAX_H
    
    @staticmethod
    def make_test_environment():
        goal_point = np.array([2.5, 2.5], dtype=np.float32)
        start_point = np.array([0.5, 0.5], dtype=np.float32)
        trajectory_boundaries = (-0.1, 3.1, -0.1, 3.1)
        return TestEnvironment(start_point, goal_point, trajectory_boundaries,
                               TestEnvironmentBuilder._obstacle_points())

    @staticmethod
    def _obstacle_points():
        collision_points1_y = np.linspace(0, 2, 10)
        collision_points1 = np.stack([np.ones(10) * 1.15, collision_points1_y], axis=1)
        collision_points2 = collision_points1.copy()
        collision_points2[:, 0] = 1.85
        collision_points2[:, 1] += 1
        # print(collision_points1)
        return np.concatenate([collision_points1, collision_points2], axis=0)


    @staticmethod
    def test_obstacle_points():
        collision_points1_y = np.linspace(0, 2, 10)
        collision_points1 = np.stack([np.ones(10) * 1.15, collision_points1_y], axis=1)
        collision_points2 = collision_points1.copy()
        collision_points2[:, 0] = 1.85
        collision_points2[:, 1] += 1
        return np.concatenate([collision_points1, collision_points2], axis=0)

    # @staticmethod
    def map_obstacle_points_3D(self):

        obstacles = []
        voxel_size = self.map.voxel_size
        obstacle_map = self.map.obstacle_map_adaptive_h
        for x in range(obstacle_map.shape[0]):
            for y in range(obstacle_map.shape[1]):
                if obstacle_map[x, y] == -1:    
                    obstacles.append((y*voxel_size, x*voxel_size))
                    # obstacle_y.append(y*voxel_size) 
        return np.array(obstacles)



     # @staticmethod
    def map_obstacle_points_2D(self):

        obstacles = []
        voxel_size = self.map.voxel_size
        obstacle_map = self.map.obstacle_map_adaptive_h
        for x in range(obstacle_map.shape[0]):
            for y in range(obstacle_map.shape[1]):
                if (obstacle_map[x, y] < 0.4):  
                    obstacles.append((y*voxel_size, x*voxel_size))
                
                
        return np.array(obstacles)

    @staticmethod
    def _point_line(start, end, point_count):
        x = np.linspace(start[0], end[0], point_count)
        y = np.linspace(start[1], end[1], point_count)
        return np.stack([x, y], axis=1)

    @staticmethod
    def make_test_environment_with_angles(self):
        goal_point = np.array([2.5, 1.5, 0], dtype=np.float32)
        start_point = np.array([0.5, 0.5, 0], dtype=np.float32)
        trajectory_boundaries = (-0.1, 3.1, -0.1, 3.1)
        
        return TestEnvironment(start_point, goal_point, trajectory_boundaries,
                               TestEnvironmentBuilder.map_obstacle_points_3D(self))

    def make_car_environment(self):
        goal_point = np.array([2, 2.7, 0], dtype=np.float32)
        start_point = np.array([0.5, 1.5, 0], dtype=np.float32)
        trajectory_boundaries = (-0.1, 3.1, -0.1, 3.1)
        return TestEnvironment(start_point, goal_point, trajectory_boundaries,
                               TestEnvironmentBuilder._car_obstacle_points())

    # @staticmethod
    def make_dog_environment(self):
        # goal_point = np.array([4.4, 4.8, 0], dtype=np.float32)
        # start_point = np.array([4, 1.8, 0], dtype=np.float32)

        "_________________Under arch______________"
        # start_point = np.array([2.6, 2.0, 0], dtype=np.float32)
        # goal_point = np.array([3.6, 3.8, 0], dtype=np.float32)
        "_________________Through tables______________"
        # start_point = np.array([5.6, 2.6, 0], dtype=np.float32)
        # goal_point = np.array([6.3, 4.0, 0], dtype=np.float32)
        start_point = np.array([3.3, 1.5, 0], dtype=np.float32)
        goal_point = np.array([4.8, 6.0, 0], dtype=np.float32)
        print("______________________________________________")
        print("The start point is:",start_point)
        trajectory_boundaries = (1., 8, 1., 8)
        # trajectory_boundaries = (0.1, 3.1, 0.1, 3.1)
        return TestEnvironment(start_point, goal_point, trajectory_boundaries,
                               TestEnvironmentBuilder.map_obstacle_points_3D(self))

    @staticmethod
    def _car_obstacle_points():
        y1 = 2.3
        x1 = 1.6
        return np.concatenate([
            TestEnvironmentBuilder._point_line((0, y1), (x1, y1), 10),
            TestEnvironmentBuilder._point_line((x1, y1), (x1, 3), 10),
            TestEnvironmentBuilder._point_line((2.5, y1), (2.5, 3), 10),
            TestEnvironmentBuilder._point_line((2.5, y1), (3, y1), 10),
        ])