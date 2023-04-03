import numpy as np




def trajectory_to_2D(plotted_trajectory, obs_map, voxel_size, robot_max_h):
  
  point_color = []

  # print(len(plotted_trajectory))
  
  for point in plotted_trajectory:
   
    # print(point[0], point[1])
    x = int(round(point[0]/voxel_size))
    y = int(round(point[1]/voxel_size))
    # print(obs_map[y, x])
    if obs_map[y, x] == -1:
        point_color.append('red')
        
     
        
    else:
    
        point_color.append('blue')
  return point_color
