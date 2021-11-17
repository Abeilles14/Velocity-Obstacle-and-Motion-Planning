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
THRESHOLD_DIST = 2

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

def update_velocity(path1, path2, vel1, vel2):
    new_path1 = interpolate(path1, vel1)
    new_path2 = interpolate(path2, vel2)
    return new_path1, new_path2

def euclidean_distance(point1, point2):
    distance = np.linalg.norm(point1-point2)
    return distance