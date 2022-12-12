import os
import re

import matplotlib.pyplot as plt
import numpy as np
import glob
from scipy.spatial.transform import Rotation as R


def create_dir(directory:str):
    if not os.path.exists(directory):
        os.makedirs(directory)


def parse_initial_dataset(path: str):
    files = glob.glob(f'{path}*[0-9].txt')
    files = sorted(files, key=lambda x:int(re.findall("(\d+)",x)[-1]))

    poses = np.zeros((len(files),6))
    coords = np.zeros((len(files),3))
    angles = np.zeros((len(files),3))


    for i, f in enumerate(files):
        
    #print(np.loadtxt(i))
        with open(f) as f_input:
            lines = f_input.readlines()
            
        # poses[i] = []
        numbers = []
        for line in lines:
            m = re.sub(r'\[|\]' ,'', line)
            m = [float(s) for s in m.split()]
            numbers += m 
        poses[i] = numbers
        
    coords = poses[:,:3]
    angles = poses[:,3:]

    return angles, coords


def get_rot_matrix_from_euler(angles):
    # R = np.array(angles)
    # R, J = cv2.Rodrigues(R)
    
    tx = angles[0]
    ty = angles[1] 
    tz = angles[2]
    
    Rx = [[1, 0, 0], [0, np.cos(tx), -np.sin(tx)], [0, np.sin(tx), np.cos(tx)]]
    Ry = [[np.cos(ty), 0, np.sin(ty)], [0, 1, 0], [-np.sin(ty), 0, np.cos(ty)]]
    Rz = [[np.cos(tz), -np.sin(tz), 0], [np.sin(tz), np.cos(tz), 0], [0, 0, 1]]
    R = np.matmul(Rz, np.matmul(Ry, Rx))
    return R


def get_rot_matrix_from_euler_nipun(angles:list):
    Rx = np.array([[1,                   0,                  0],
                    [0,   np.cos(angles[0]), -np.sin(angles[0])],
                    [0,   np.sin(angles[0]),  np.cos(angles[0])]])

    Ry = np.array([[ np.cos(angles[1]),    0,   np.sin(angles[1])],
                    [                 0,    1,                   0],
                    [-np.sin(angles[1]),    0,   np.cos(angles[1])]])

    Rz = np.array([[np.cos(angles[2]),  -np.sin(angles[2]), 0],
                    [np.sin(angles[2]),   np.cos(angles[2]), 0],
                    [                0,                   0, 1]])

    # rot = Rx@Ry@Rz 
    rot = Rz@Ry@Rx 
    return rot


def get_translation(coord):

    offset_base_to_rgb = np.array([10, 32, 13])/1000
    #From vicon link to base link
    
    T = np.array(coord)
    T = T*10**(-6)
    T += offset_base_to_rgb
    return T


def get_intrinsic(fx, fy, cx, cy):
    return np.array([[fx, 0 , cx],
                       [0, fy, cy],
                       [0, 0, 1]])


def prepare_data(indir:str, outdir:str):
    
    transform = np.zeros((4,4))

    # TODO: load it from .yaml/.json file
    # Parameters for realsense D435i
    fx = 615.671 # focal dist of cam lense [mm]
    fy = 615.962 # 
    cx = 328.001 # optical center
    cy = 241.31


    angles, coord = parse_initial_dataset(indir) # get raw angle data
    
    # TODO: change it to Nipun's rotations
    tx = -90*(np.pi/180) # -90
    ty = 0.776*(np.pi/180) # 0.776
    tz = -90*(np.pi/180) # -90
    
    # Update: now we are not gona use it
    R_base_to_camera_color_opt = get_rot_matrix_from_euler((tx, ty, tz))
    
    # R_base_to_camera_color_opt = np.identity(3)

    create_dir(outdir + 'pose')
    for i, (ang, c) in enumerate(zip(angles, coord)):
        
        R = get_rot_matrix_from_euler_nipun(ang)
        R = R_base_to_camera_color_opt @ R
        transform[0:3, 0:3] = R
       
        T = get_translation(c)
        print(T)
        transform[0:3, 3] = T
        transform[3, 3] = 1
        np.savetxt(f'{outdir}pose/{i+1:08}.txt', transform)
    
    cam_param= get_intrinsic(fx, fy, cx, cy)
    np.savetxt(f'{outdir}intrinsics.txt', cam_param)
    