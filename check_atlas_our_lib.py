import numpy as np
from matplotlib import pyplot as plt 
from mpl_toolkits import mplot3d
from convert_to_atlas_dataset import *

class RT_PLOT():
    def __init__(self):
        self.glob_coord = np.array([[1,0,0], [0,1,0],[0,0,1]])

    def rot_mat(self, angles:list):
        Rx = np.array([[1,                   0,                  0],
                       [0,   np.cos(angles[0]), -np.sin(angles[0])],
                       [0,   np.sin(angles[0]),  np.cos(angles[0])]])

        Ry = np.array([[ np.cos(angles[1]),    0,   np.sin(angles[1])],
                       [                 0,    1,                   0],
                       [-np.sin(angles[1]),    0,   np.cos(angles[1])]])

        Rz = np.array([[np.cos(angles[2]),  -np.sin(angles[2]), 0],
                       [np.sin(angles[2]),   np.cos(angles[2]), 0],
                       [                0,                   0, 1]])

        rot = Rx@Ry@Rz 
        return rot

    def get_R_T_matrix(self, ang, disp):
        ang = -np.array(ang)*np.pi/180
        # get rotation matrix
        R = np.array(self.glob_coord @ self.rot_mat(ang))
        # make RT matrix
        T = np.array(np.zeros([4,4]))
        for i in range(3):
            for j in range(3):
                if np.abs(R[i,j]) < (1.0e-15) :
                    R[i,j] = 0
                T[i,j] = R[i,j]
        for i in range(4):
            if i != 3:
                T[i,3] = disp[i]
            else:
                T[3,3] = 1
        return R, T

    def plot_coords_and_rots(self, disp_arr = None, ang_arr = None, rt_arr=None, plot_rot_axes=False, view_pt=(10,10)):
        assert np.any(disp_arr) or np.any(rt_arr)

        # X_axes = np.zeros(len(ang_arr), 2,3)
        # Y_axes = np.zeros(len(ang_arr), 2,3)
        # Z_axes = np.zeros(len(ang_arr), 2,3)

        fig = plt.figure(dpi=250)
        ax = plt.axes(projection='3d')

        # plot global coordinate
        ax.plot([0,1], [0,0], [0,0], 'r-') # - blue: x_ax 
        ax.plot([0,0], [0,1], [0,0], 'g-') # - green: y_ax
        ax.plot([0,0], [0,0], [0,1], 'b-') # - cian: z_ax

        if np.any(rt_arr):
            if plot_rot_axes:
                for RT in rt_arr:
                    # objetc coordinate
                    R = RT[:3,:3]
                    # R = RT[:3,:3].T.copy()
                    # T = RT.T.copy()
                    T = RT
                    new_coord = self.glob_coord @ R
                    new_coord = np.array(new_coord)
                    scale = 1
                    
                    x_ax = np.array([[T[0,3], T[0,3]+new_coord[0,0]*scale], [T[1,3], T[1,3]+new_coord[0,1]*scale], [T[2,3],T[2,3]+new_coord[0,2]*scale]])
                    y_ax = np.array([[T[0,3], T[0,3]+new_coord[1,0]*scale], [T[1,3], T[1,3]+new_coord[1,1]*scale], [T[2,3],T[2,3]+new_coord[1,2]*scale]])
                    z_ax = np.array([[T[0,3], T[0,3]+new_coord[2,0]*scale], [T[1,3], T[1,3]+new_coord[2,1]*scale], [T[2,3],T[2,3]+new_coord[2,2]*scale]])
                    
                    # plot object coordinate
                    ax.plot(x_ax[0], x_ax[1], x_ax[2], 'r--') # blue: x_ax
                    ax.plot(y_ax[0], y_ax[1], y_ax[2], 'g--') # blue: x_ax
                    ax.plot(z_ax[0], z_ax[1], z_ax[2], 'b--') # blue: x_ax
            ax.plot(rt_arr[:,0,3], rt_arr[:,1,3], rt_arr[:,2,3], color='yellow')
              
        if np.any(disp_arr):
            ax.plot(disp_arr[:,0], disp_arr[:,1], disp_arr[:,2], color='yellow')
            if plot_rot_axes and np.any(ang_arr):
                for ang, disp in zip(ang_arr, disp_arr):
                    # objetc coordinate
                    R, T = self.get_R_T_matrix(ang, disp)
                    new_coord = self.glob_coord @ R
                    new_coord = np.array(new_coord)
                    scale = 1 
                    
                    x_ax = [[T[0,3], T[0,3]+new_coord[0,0]*scale], [T[1,3], T[1,3]+new_coord[0,1]*scale], [T[2,3],T[2,3]+new_coord[0,2]*scale]]
                    y_ax = [[T[0,3], T[0,3]+new_coord[1,0]*scale], [T[1,3], T[1,3]+new_coord[1,1]*scale], [T[2,3],T[2,3]+new_coord[1,2]*scale]]
                    z_ax = [[T[0,3], T[0,3]+new_coord[2,0]*scale], [T[1,3], T[1,3]+new_coord[2,1]*scale], [T[2,3],T[2,3]+new_coord[2,2]*scale]]

                    # plot object coordinate
                    ax.plot(x_ax[0], x_ax[1], x_ax[2], 'r--') # blue: x_ax
                    ax.plot(y_ax[0], y_ax[1], y_ax[2], 'g--') # blue: x_ax
                    ax.plot(z_ax[0], z_ax[1], z_ax[2], 'b--') # blue: x_ax
            
        ax.legend(['x_ax', 'y_ax', 'z_ax']);
        ax.view_init(elev=view_pt[0], azim=view_pt[1])

        plt.show();

def main():
    disp = np.array([[1,1,1],[2,2,2]]) #
    ang = np.array([[30,0,0],[30,0,0]]) # [deg]
    # files_path = '/home/dendenmushi/projects/some_stuff/atlas_sample_poses/'
    files_path = '/home/dendenmushi/projects/some_stuff/dataset_true-20221212T121046Z-001/dataset_true/drone_view/pose/'

    RT_arr = parse_dataset(files_path)
    rt_plt = RT_PLOT()
    rt_plt.plot_coords_and_rots(rt_arr=RT_arr[:], plot_rot_axes=False ,view_pt=(50,30))

if __name__ == '__main__':
    main()
