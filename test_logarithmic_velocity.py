# COLLISION AVOIDANCE MECHANISM
import numpy as np
import matplotlib.pyplot as plt
import time
from mpl_toolkits import mplot3d
from enum import Enum
import logging
import pickle
from scipy.interpolate import interp1d
from ground.base import get_context
from bentley_ottmann.planar import segments_intersect
from Geometry3D import *

from shapely.geometry import LineString, MultiPoint, shape
from RRTStar import RRTStar
from arm import Arm
from objects import Object
from arm import Arm

logger = logging.getLogger(__name__)
logging.basicConfig()
logger.setLevel(logging.INFO)

STEP_SIZE = 0.08 #controls speed of paths
THRESHOLD_DIST = 1

def main():

    animate = False

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlabel('X, [m]')
    ax.set_ylabel('Y, [m]')
    ax.set_zlabel('Z, [m]')
    ax.set_xlim([0,12])
    ax.set_ylim([0,12])
    ax.set_zlim([0,12])

    # l1 = np.array([[0.0,0.0,0.0], [10.0,8.0,8.0]])
    # l2 = np.array([[2.0,0.0,3.0], [6.0,8.0,8.0], [10.0,2.0,2.0]])
    l1 = np.array([[0.0,0.0,0.0], [10.0,8.0,8.0]])
    l2 = np.array([[2.0,0.0,3.0], [6.0,8.0,8.0], [10.0,8.0,8.0]])

    # initialize arms
    # start at 1 pts of each lines, end at last pt of each line
    arm1 = Arm(name="PSM1", position=l1[0], destination=l1[l1.shape[0]-1], velocity=STEP_SIZE, home=l1[0])
    arm2 = Arm(name="PSM2", position=l2[0], destination=l2[l2.shape[0]-1], velocity=STEP_SIZE, home=l2[0])

    plt.plot(l1[:,0], l1[:,1], l1[:,2], 'o', color='orange')
    plt.plot(l2[:,0], l2[:,1], l2[:,2], 'o', color='blue')

    # initialize paths
    path1 = interpolate(l1, STEP_SIZE)
    path2 = interpolate(l2, STEP_SIZE)

    # draw planned paths
    for i in range(l1.shape[0]-1):
        ax.plot([l1[i,0], l1[i+1,0]], [l1[i,1], l1[i+1,1]], [l1[i,2], l1[i+1,2]], color = 'orange', linewidth=1, zorder=15)
    for i in range(l2.shape[0]-1):
        ax.plot([l2[i,0], l2[i+1,0]], [l2[i,1], l2[i+1,1]], [l2[i,2], l2[i+1,2]], color = 'blue', linewidth=1, zorder=15)

    arm1.set_position(path1[0])    # start at pt. 0 of path1
    arm2.set_position(path2[0])    # start at pt. 60 of path2

    # now that we have the intersection zones in both paths, adjust speed and animate
    intersect1, intersect2 = find_intersection(path1, path2, arm1, arm2)  
    intersect_pts1, intersect_pts2 = np.array(intersect1), np.array(intersect2)
    new_path1, new_path2 = avoid_collision(intersect_pts1, intersect_pts2, path1, path2, arm1, arm2)
    run_path(new_path1, new_path2, arm1, arm2)

    # logger.info("INTERSECTIONS: {}".format(intersect1))
    # plt.show()

def euclidean_distance(point1, point2):
    distance = np.linalg.norm(point1-point2)
    return distance

def avoid_collision(intersect_pts1, intersect_pts2, path1, path2, arm1, arm2):
    # get start index for path change (current arm pos)
    idx1 = np.where(path1 == arm1.get_position())[0][0]     # get start index
    idx2 = np.where(path2 == arm2.get_position())[0][0]     # get start index

    # if collision detected, adjust path velocities
    if (intersect_pts1.shape[0] != 0) and (intersect_pts2.shape[0] != 0):
        logger.info("COLLISION DETECTED!")
        logger.debug("INTERSECTIONS: {}, SHAPE: {}".format(intersect_pts1, intersect_pts1.shape[0]))
        # temp pre-set velocities:
        # NOTE: start point of new paths is arm current location!! need to iter from 0 when plotting
        new_path1, new_path2 = update_velocity(path1[idx1:,:], path2[idx2:,:], vel1=0.07, vel2=0.09)
        # set new path and last collision point
        logger.info("UPDATED VELOCITY FOR COLLISION AVOIDANCE")
        print("Arm1: {}, Arm2: {}".format(0.07, 0.09))
    else:
        # reset paths velocities if no more intersections
        logger.info("NO COLLISION DETECTED!")
        new_path1, new_path2 = update_velocity(path1[idx1:,:], path2[idx2:,:], vel1=STEP_SIZE, vel2=STEP_SIZE)
        print("Arm1: {}, Arm2: {}".format(STEP_SIZE, STEP_SIZE))
        # arm1_sm.set_path(new_path1, np.empty(3))
        # arm2_sm.set_path(new_path2, np.empty(3))

    return new_path1, new_path2

def run_path(path1, path2, arm1, arm2):
    # check whether any pts in paths are within threshold
    # idx1 = np.where(path1 == arm1.get_position())[0][0]     # get start index
    # idx2 = np.where(path2 == arm2.get_position())[0][0]     # get start index
    
    idx1 = 0
    idx2 = 0
    # path_range = min(path1[idx1:,:].shape[0], path2[idx2:,:].shape[0])    # get minimum of both remaining paths
    # print("PATH RANGES: {}, {}".format(path1[idx1:,:].shape[0], path2[idx2:,:].shape[0]))

    intersect1 = []
    intersect2 = []
    i = 0
    while(idx1 != path1[idx1:,:].shape[0] or idx2 != path2[idx2:,:].shape[0]):
        # idx1 = np.where(path1 == arm1.get_position())[0][0] + i
        # idx2 = np.where(path2 == arm2.get_position())[0][0] + i
        idx1 = i
        idx2 = i

        if idx1 < path1.shape[0]:
            plt.plot(path1[idx1,0], path1[idx1,1], path1[idx1,2], 'o', color='red', markersize=1)
        if idx2 < path2.shape[0]:
            plt.plot(path2[idx2,0], path2[idx2,1], path2[idx2,2], 'o', color='red', markersize=1)
            
        i += 1
        plt.pause(0.0005)

    plt.show()

def find_intersection(path1, path2, arm1, arm2):
    # check whether any pts in paths are within threshold
    # get start index for path change (current arm pos)
    idx1 = np.where(path1 == arm1.get_position())[0][0]     # get start index
    idx2 = np.where(path2 == arm2.get_position())[0][0]     # get start index
    path_range = min(path1[idx1:,:].shape[0], path2[idx2:,:].shape[0])    # get minimum of both remaining paths
    logger.info("PATH RANGES: {}, {}".format(path1[idx1:,:].shape[0], path2[idx2:,:].shape[0]))

    intersect1 = []
    intersect2 = []
    for i in range(path_range-1):
        idx1 = np.where(path1 == arm1.get_position())[0][0] + i
        idx2 = np.where(path2 == arm2.get_position())[0][0] + i
        # print("IDX: {}, {}".format(idx1, idx2))
        arm_dist = euclidean_distance(path1[idx1], path2[idx2])

        logger.debug("POS: {}, {}".format(path1[idx1], path2[idx2]))
        logger.debug("ARM_DIST: {}".format(arm_dist))
        # plt.plot(path1[idx1,0], path1[idx1,1], path1[idx1,2], 'o', color='red', markersize=1)
        # plt.plot(path2[idx2,0], path2[idx2,1], path2[idx2,2], 'o', color='red', markersize=1)
        
        if (arm_dist <= THRESHOLD_DIST).all():
            logger.debug("COLLISION IMMINENT!")
            intersect1.append([path1[idx1,0], path1[idx1,1], path1[idx1,2]])
            intersect2.append([path1[idx1,0], path1[idx1,1], path1[idx1,2]])
            plt.plot(path1[idx1,0], path1[idx1,1], path1[idx1,2], 'o', color='cyan')
            plt.plot(path2[idx2,0], path2[idx2,1], path2[idx2,2], 'o', color='cyan')

    return intersect1, intersect2

def update_velocity(path1, path2, vel1, vel2, idx=None):
    """ Update velocity of both arm paths.

    Keyword Arguments:
    path1 -- arm 1 path
    path2 -- arm 2 path
    vel 1 -- arm 1 wanted vel
    vel 2 -- arm 2 wanted vel
    idx -- index at which we want vel to reset to init
    """

    if idx == None:     # no collision
        new_path1 = interpolate(path1, vel1)
        new_path2 = interpolate(path2, vel2)
    else:






        new_path2_to_col = interpolate(path2[:idx,:], vel2)    # interpolate until collision pt
        
        # want path1 to have same length as path 2 to resume init vel at same time
        # TODO: fix this if using log, won't work! need to interpolate till point
        # first interpolate pts until collision point
        # then only count N pts from that path, where N is new_path2's shape
        new_path1_to_col = interpolate(path1[:idx,:], vel1)[:new_path2_to_col.shape[0]-1,:]
    









        # find where to reset slow path to normal velocity
        fast_path_idx = new_path2_to_col.shape[0]
        init_path_idx = (fast_path_idx*vel2)/0.08
        slow_path_idx = (fast_path_idx*vel2)/vel1

        path_reset_idx = round((init_path_idx/slow_path_idx)*fast_path_idx)

        # concat new vel path to collision and post-collision init vel path
        # this new path has adjusted vel until last collision point, then back to regular vel
        new_path2 = np.concatenate((new_path2_to_col, np.delete(path2, np.arange(0, idx), axis=0)), axis=0)
        new_path1 = np.concatenate((new_path1_to_col, np.delete(path1, np.arange(0,path_reset_idx-1), axis=0)), axis=0)

    return new_path1, new_path2

def interpolate(P, step, show_interplot=False):
    x, y, z = P.T
    xd = np.diff(x)
    yd = np.diff(y)
    zd = np.diff(z)
    dist = np.sqrt(xd**2 + yd**2 + zd**2)
    _u = np.cumsum(dist)
    u = np.hstack([[0], _u])

    # t = np.linspace(0, u.max(), size) #interpolate using # points (80)
    t = np.logspace(0, u.max()+step, endpoint=True, base=1.2)   # base start, base stop, base
    # t = np.arange(0, u.max()+step, step)  # start val, end val, steps         


    xn = np.interp(t, u, x)     # TODO: make sure fp and xp len matches
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

if __name__ == '__main__':
    main()