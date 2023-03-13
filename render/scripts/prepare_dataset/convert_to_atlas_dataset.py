import os
import re
import glob

import numpy as np
from scipy.spatial.transform import Rotation 


def create_dir(directory:str):
    if not os.path.exists(directory):
        os.makedirs(directory)


def parse_initial_dataset(path: str):
    files = glob.glob(f'{path}*[0-9].txt')
    files = sorted(files, key=lambda x:int(re.findall("(\d+)",x)[-1]))

    poses = np.zeros((len(files),7))
    coords = np.zeros((len(files),3))
    angles = np.zeros((len(files),4))


    for i, f in enumerate(files):
        
    #print(np.loadtxt(i))
        with open(f, 'r') as f_input:
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


def parse_dataset(indir):
    files = glob.glob(f'{indir}*[0-9].txt')
    files = sorted(files, key=lambda x:int(re.findall("(\d+)",x)[-1]))

    RT_arr = np.zeros((len(files), 4,4))

    for i,file in enumerate(files):
        RT_arr[i] = np.loadtxt(file)
    return RT_arr


def get_rot_matrix_from_quatr(angles:list):
    # R = np.array(angles)
    # R, J = cv2.Rodrigues(R)
    
    
    r = Rotation.from_quat([[angles[0], angles[1], angles[2], angles[3]]])

    
    return r.as_matrix()


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

    # offset_base_to_rgb = np.array([10, 32, 13])/1000
    #From vicon link to base link
    offset_vicon_to_base = np.array([8, 10, 32])/1000
    T = np.array(coord)
    T += offset_vicon_to_base
    # T += offset_base_to_rgb
    return T


def get_intrinsic(fx, fy, cx, cy):
    return np.array([[fx, 0 , cx],
                       [0, fy, cy],
                       [0, 0, 1]])


def prepare_data(indir:str, outdir:str):
    
    transform = np.zeros((4,4))

    # TODO: load it from .yaml/.json file
    # Parameters for realsense D435i


    # fx = 1364.558 # focal dist of cam lense [mm]
    # fy = 1364.338 # 
    # cx = 640 # optical center
    # cy = 310
    fx = 1367 # focal dist of cam lense [mm]
    fy = 1365 # 
    cx = 968 # optical center
    cy = 550


    angles, coord = parse_initial_dataset(indir) # get raw angle data
    # coord = rescale(coord)
    # TODO: change it to Nipun's rotations
    # tx = -90*(np.pi/180) # -90
    # ty = 0*(np.pi/180) # 0.776
    # tz = -90*(np.pi/180) # -90
    # r = Rotation.from_rotvec([tx, ty, tz])
    # R_base_to_camera_color_opt = r.as_matrix()
    tx = -0.495
    ty = 0.501
    tz = -0.498
    tw = 0.506
    # Update: now we are not gona use it
    R_base_to_camera_color_opt = get_rot_matrix_from_quatr((tx, ty, tz, tw))
    
    # R_base_to_camera_color_opt = np.identity(3)

    create_dir(outdir + 'pose')
    for i, (ang, c) in enumerate(zip(angles, coord)):
        
        R = get_rot_matrix_from_quatr(ang)
        print(R)
        R = R @ R_base_to_camera_color_opt
        transform[0:3, 0:3] = R
       
        T = get_translation(c)
        # T = rescale(T)
        # print(T)
        transform[0:3, 3] = T
        transform[3, 3] = 1
        #transform = np.linalg.inv(transform)
        np.savetxt(f'{outdir}pose/{i+1:08}.txt', transform)
    
    cam_param= get_intrinsic(fx, fy, cx, cy)
    np.savetxt(f'{outdir}intrinsics.txt', cam_param)

def rescale(coord): 
    return coord/1.5
    

def main():
    outdir = '/home/iana/anaconda3/Atlas/src/render/scripts/DATAROOT/sample/sample1/'
    indir = '/home/iana/Atlas/dataset_drone_dog/poses_raw/'
    prepare_data(indir, outdir)
   
main()
