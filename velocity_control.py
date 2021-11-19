# COLLISION AVOIDANCE MECHANISM
import numpy as np
import matplotlib.pyplot as plt
import time
from mpl_toolkits import mplot3d
from enum import Enum
import logging

from RRTStar import RRTStar
from arm import Arm
from objects import Object

logger = logging.getLogger(__name__)
logging.basicConfig()
logger.setLevel(logging.INFO)

# Constants
THRESHOLD_DIST = 0.5

# Convert xyz-data to a parametrized curve
# calculate all distances between the points
# generate the coordinates on the curve by cumulative summing
# interpolate the x- and y-coordinates independently with respect to the new coords
def interpolate(P, step, show_interplot=False):
    x, y, z = P.T
    xd = np.diff(x)
    yd = np.diff(y)
    zd = np.diff(z)
    dist = np.sqrt(xd**2 + yd**2 + zd**2)
    u = np.cumsum(dist)
    u = np.hstack([[0], u])

    # t = np.linspace(0, u.max(), size) #interpolate using # points (80)
    t = np.arange(0, u.max()+step, step)
    xn = np.interp(t, u, x)
    yn = np.interp(t, u, y)
    zn = np.interp(t, u, z)

    if show_interplot:
        f = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot(x, y, z, 'o', alpha=0.3)
        ax.plot(xn, yn, zn, 'ro', markersize=2)
        ax.set_xlim([-2.5, 2.5])
        ax.set_ylim([-2.5, 2.5])
        ax.set_zlim([0.0, 3.0])
    
    path = np.column_stack((xn, yn, zn))

    return path

def find_intersection(path1, path2, arm1, arm2, animate=False):
    animate = False

    # check whether any pts in paths are within threshold
    # idx1 = np.where(path1 == arm1.get_position())[0][0]     # get start index
    # idx2 = np.where(path2 == arm2.get_position())[0][0]     # get start index
    #TODO: debug here, path1[0] != arm pos??
    idx1 = 0
    idx2 = 0
    path_range = min(path1[idx1:,:].shape[0], path2[idx2:,:].shape[0])    # get minimum of both remaining paths
    logging.debug("PATH RANGES: {}, {}".format(path1[idx1:,:].shape[0], path2[idx2:,:].shape[0]))

    intersect_pts1 = []
    intersect_pts2 = []
    for i in range(path_range-1):
        # idx1 = np.where(path1 == arm1.get_position())[0][0] + i
        # idx2 = np.where(path2 == arm2.get_position())[0][0] + i
        idx1 = i
        idx2 = i
        # print("IDX: {}, {}".format(idx1, idx2))
        arm_dist = euclidean_distance(path1[idx1], path2[idx2])

        logging.debug("POS: {}, {}".format(path1[idx1], path2[idx2]))
        logging.debug("ARM_DIST: {}".format(arm_dist))

        if (arm_dist <= THRESHOLD_DIST).all():
            intersect_pts1.append([path1[idx1,0], path1[idx1,1], path1[idx1,2]])
            intersect_pts2.append([path1[idx1,0], path1[idx1,1], path1[idx1,2]])
            
            plt.plot(path1[idx1,0], path1[idx1,1], path1[idx1,2], 'o', color='cyan')
            plt.plot(path2[idx2,0], path2[idx2,1], path2[idx2,2], 'o', color='cyan')
    
    return (np.array(intersect_pts1), np.array(intersect_pts2))

# path 1 init vel is 0.08, path2 init vel is 0.08
def update_velocity(path1, path2, vel1, vel2, idx=None):
    # NOTE: currently arm2 speeds up, arm 1 only resumes init vel once arm2 reaches last col pt
    if idx == None:
        new_path1 = interpolate(path1, vel1)
        new_path2 = interpolate(path2, vel2)
    else:
        new_path2_to_col = interpolate(path2[:idx,:], vel2)    # interpolate until collision pt
        
        # want path1 to have same length as path 2 to resume init vel at same time
        # first interpolate pts until last collision point
        # then only count N pts from that path, where N is new_path2's shape
        new_path1_to_col = interpolate(path1[:idx,:], vel1)[:new_path2_to_col.shape[0]-1,:]
    
        # find where to reset slow path to normal velocity
        fast_path_idx = new_path2_to_col.shape[0]
        init_path_idx = (fast_path_idx*vel2)/0.08
        slow_path_idx = (fast_path_idx*vel2)/vel1

        path_reset_idx = round((init_path_idx/slow_path_idx)*fast_path_idx)
        print("RESET SLOW: {}".format(path_reset_idx))
        # concat new vel path to collision and post-collision init vel path
        # this new path has adjusted vel until last collision point, then back to regular vel
        new_path2 = np.concatenate((new_path2_to_col, np.delete(path2, np.arange(0, idx), axis=0)), axis=0)

        new_path1 = np.concatenate((new_path1_to_col, np.delete(path1, np.arange(0,path_reset_idx-1), axis=0)), axis=0)

    return new_path1, new_path2

def euclidean_distance(point1, point2):
    distance = np.linalg.norm(point1-point2)
    return distance