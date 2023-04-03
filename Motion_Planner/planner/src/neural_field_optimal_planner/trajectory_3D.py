import numpy as np
import math


''' Here we add the robot height for each point of trajectory'''

def trajectory_to_3D(plotted_trajectory, obs_map, voxel_size, robot_max_h):
  path = []
  point_color = []
 
  # print(len(plotted_trajectory))
  
  for point in plotted_trajectory: 
   
    # print(point[0], point[1])
    x = int(round(point[0]/voxel_size))
    y = int(round(point[1]/voxel_size))
    # print(obs_map[y, x])
    if obs_map[y, x] < robot_max_h:
        z = obs_map[y, x]
        path.append((x*voxel_size, y*voxel_size, point[2], z))
        if z == -1:
          point_color.append('red')
        else:
          point_color.append('yellow')
    else:
        z = robot_max_h
        path.append((x*voxel_size, y*voxel_size, point[2], z))
        point_color.append('blue')
  return np.array(path), point_color


def calculate_distance(starting_x, starting_y, destination_x, destination_y):
    distance = math.hypot(destination_x - starting_x, destination_y - starting_y)  # calculates Euclidean distance (straight-line) distance between two points
    # print('Segment Dist: ', distance)
    return distance

def calculate_path(selected_map, dist_travel=0):
    for i in range(len(selected_map)-1):
        dist_travel += calculate_distance(selected_map[i-len(selected_map)+1][0], selected_map[i-len(selected_map)+1][1], selected_map[i][0], selected_map[i][1])
    return dist_travel


 
