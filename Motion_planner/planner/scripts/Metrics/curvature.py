from math import atan2, sin, cos, pi, sqrt
import numpy as np
import os

def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    d = np.sqrt(np.power((x2 - x1), 2) + np.power((y2 - y1), 2))
    return d

def normalize_angle(angle: float) -> float:
    return atan2(sin(angle), cos(angle))

def path_length(path: np.ndarray) -> float: # x, y [N, 2]
    return np.sum(np.sqrt(np.power((path[1:,0] - path[:-1,0]),2) + np.power((path[1:,1] - path[:-1,1]),2)))

def slope(x1: float, y1: float, x2: float, y2: float) -> float:
    dy = y2 - y1
    dx = x2 - x1
    return normalize_angle(atan2(dy,dx))

def get_cusps(path: np.ndarray) -> int: # x, y, theta [N, 3]
    cusps = 0
    for i in range(1,len(path)-1):
        yaw_prev = slope(path[i-1][0], path[i-1][1], path[i][0], path[i][1])
        yaw_next = slope(path[i][0], path[i][1], path[i+1][0], path[i+1][1])
        yaw_change = normalize_angle(yaw_next-yaw_prev)
        if yaw_change > 60*pi/180:
            cusps += 1
    return cusps

def AOL(path: np.ndarray) -> float: # x, y, theta [N, 3]
    path_len = path_length(path[:,:2])
    total_yaw_change = 0
    for i in range(1,len(path)-1):
        yaw_prev = slope(path[i-1][0], path[i-1][1], path[i][0], path[i][1])
        yaw_next = slope(path[i][0], path[i][1], path[i+1][0], path[i+1][1])
        yaw_change = normalize_angle(yaw_next-yaw_prev)
        total_yaw_change += yaw_change
    return total_yaw_change/path_len

def norm_curv(path: np.ndarray) -> float: # x, y, theta [N, 3]
    x1=x2=x3=y1=y2=y3 = 0
    normalized_k = 0
    eps = 1e-2
    max_curvature = 1e300

    for i in range(0,len(path)-2):
        x1 = path[i][0]
        y1 = path[i][1]
        while (True):
            i += 1
            if (i >= len(path)):
                return normalized_k
            x2 = path[i][0]
            y2 = path[i][1]
            if (distance(x1, y1, x2, y2) > 0.3):
                break

        while (True):
            i += 1
            if (i >= len(path)):
                return normalized_k
            x3 = path[i][0]
            y3 = path[i][1]
            if (distance(x2, y2, x3, y3) > 0.3):
                break

        if ((abs(abs(x1)-abs(x2)) < eps and abs(abs(y1)-abs(y2)) < eps) or \
            (abs(abs(x2)-abs(x3)) < eps and abs(abs(y2)-abs(y3)) < eps)):
            continue

        if (abs(abs(x1)-abs(x3)) < eps and abs(abs(y1)-abs(y3)) < eps):
            print("Infinite curvature in case the path goes a step backwards, skip calculation")
            continue
        
        cx = (np.power(x3, 2) * (-y1 + y2) + np.power(x2, 2) * (y1 - y3) - \
           (np.power(x1, 2) + (y1 - y2) * (y1 - y3)) * (y2 - y3)) / \
          (2. * (x3 * (-y1 + y2) + x2 * (y1 - y3) + x1 * (-y2 + y3)))
        
        cy = (-(np.power(x2, 2) * x3) + np.power(x1, 2) * (-x2 + x3) + \
           x3 * (np.power(y1, 2) - np.power(y2, 2)) + \
           x1 * (np.power(x2, 2) - np.power(x3, 2) + np.power(y2, 2) - \
                 np.power(y3, 2)) + \
           x2 * (np.power(x3, 2) - np.power(y1, 2) + np.power(y3, 2))) / \
          (2. * (x3 * (y1 - y2) + x1 * (y2 - y3) + x2 * (-y1 + y3)))

        radius = sqrt(np.power(x1 - cx, 2) + np.power(y1 - cy, 2))
        ki = 1. / radius
        ki = min(ki, max_curvature)
        normalized_k += ki * (distance(x1, y1, x2, y2) + distance(x2, y2, x3, y3))
        
    return normalized_k


def max_curv(path: np.ndarray) -> float: # x, y, theta [N, 3]
    x1=x2=x3=y1=y2=y3=v1x=v1y=v2x=v2y = 0
    eps = 1e-2
    max_curvature = 0
    inf = 1e300

    if len(path) == 0:
        return inf

    for i in range(0,len(path)-2):
        x1 = path[i][0]
        y1 = path[i][1]
        x2 = path[i+1][0]
        y2 = path[i+1][1]
        x3 = path[i+2][0]
        y3 = path[i+2][1]
        
        if ((abs(abs(x1)-abs(x2)) < eps and abs(abs(y1)-abs(y2)) < eps) or \
            (abs(abs(x2)-abs(x3)) < eps and abs(abs(y2)-abs(y3)) < eps)):
            continue
        
        if (abs(abs(x1)-abs(x3)) < eps and abs(abs(y1)-abs(y3)) < eps):
            print("Infinite curvature in case the path goes a step backwards, skip calculation")
            continue
        
        v1x = x1 - x2
        v1y = y1 - y2
        v2x = x3 - x2
        v2y = y3 - y2
        v1 = sqrt(v1x * v1x + v1y * v1y)
        v2 = sqrt(v2x * v2x + v2y * v2y)
        v1x = (0.5 * v1x * (v1 + v2)) / v1
        v1y = (0.5 * v1y * (v1 + v2)) / v1
        v2x = (0.5 * v2x * (v1 + v2)) / v2
        v2y = (0.5 * v2y * (v1 + v2)) / v2

        x1 = x2 + v1x
        y1 = y2 + v1y
        x3 = x2 + v2x
        y3 = y2 + v2y

        k_i = 2 * abs((x2 - x1) * (y3 - y1) - (x3 - x1) * (y2 - y1)) / \
                (sqrt(((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1)) * \
                    ((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1)) * \
                    ((x3 - x2) * (x3 - x2) + (y3 - y2) * (y3 - y2))))

        if (k_i > max_curvature):
            max_curvature = k_i

    return max_curvature





base_path = os.getcwd()


trajectory_3D = np.load(f"{base_path}/trajectory_3D.npy") # t x y theta
trajectory_2D = np.load(f"{base_path}/trajectory_2D.npy") # t x y theta


trajectories = [trajectory_3D, trajectory_2D]
# poses = [poses1, poses5, poses2, poses3, poses4]
aols = []
norm_curvs = []
max_curvs = []
path_lens = []
cusps = []

for trajectory in trajectories:
    path = np.array(trajectory[:,1:])
    cusps.append(get_cusps(path))
    aols.append(AOL(path))
    norm_curvs.append(norm_curv(path))
    max_curvs.append(max_curv(path))
    path_lens.append(path_length(path[:,:2]))



print("_____Max_curv___________________Path_length__3D:", max_curvs[0], path_lens[0])

print("_____Max_curv___________________Path_length__2D:", max_curvs[1], path_lens[1])