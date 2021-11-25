# COLLISION AVOIDANCE MECHANISM
import numpy as np
import matplotlib.pyplot as plt

import logging

from arm import Arm
from velocity_control import *
from arm import Arm

logger = logging.getLogger(__name__)
logging.basicConfig()
logger.setLevel(logging.INFO)


INIT_VEL = 0.05 #controls speed of paths
INC_VEL = 0.05
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

    l1 = np.array([[0.0,0.0,0.0], [6.5,7.0,7.0], [9.0,1.0,2.0]])
    l2 = np.array([[2.0,0.0,3.0], [4.0,6.0,6.0], [5.0,4.0,4.0], [10.0,6.0,6.0]])
    # l1 = np.array([[0.0,0.0,0.0], [10.0,8.0,8.0]])
    # l2 = np.array([[2.0,0.0,3.0], [4.0,6.0,6.0], [10.0,2.0,2.0]])

    # initialize arms
    # start at 1 pts of each lines, end at last pt of each line
    arm1 = Arm(name="PSM1", position=l1[0], destination=l1[l1.shape[0]-1], velocity=INIT_VEL, home=l1[0])
    arm2 = Arm(name="PSM2", position=l2[0], destination=l2[l2.shape[0]-1], velocity=INIT_VEL, home=l2[0])

    plt.plot(l1[:,0], l1[:,1], l1[:,2], 'o', color='orange')
    plt.plot(l2[:,0], l2[:,1], l2[:,2], 'o', color='blue')

    # initialize paths
    path1 = linear_interpolation(l1, INIT_VEL)
    path2 = linear_interpolation(l2, INIT_VEL)

    for i in range(l1.shape[0]-1):
        ax.plot([l1[i,0], l1[i+1,0]], [l1[i,1], l1[i+1,1]], [l1[i,2], l1[i+1,2]], color = 'orange', linewidth=1, zorder=15)
    for i in range(l2.shape[0]-1):
        ax.plot([l2[i,0], l2[i+1,0]], [l2[i,1], l2[i+1,1]], [l2[i,2], l2[i+1,2]], color = 'blue', linewidth=1, zorder=15)


    arm1.set_position(path1[0])    # start at pt. 0 of path1
    arm2.set_position(path2[0])    # start at pt. 60 of path2
    check_collision = True
    arm1_collision, arm2_collision = np.empty(3), np.empty(3)
    idx1 = 0
    idx2 = 0
    i = 0

    while(idx1 != path1.shape[0] or idx2 != path2.shape[0]):
        # idx1 = i
        # idx2 = i

        if check_collision:
            # check whether any pts in paths are within threshold
            # get start index for path change (current arm pos)
            intersect_pts1, intersect_pts2 = find_intersection(path1, path2, arm1, arm2)
            
            if intersect_pts1.size > 0 and intersect_pts2.size > 0:
                # now that we have the intersection zones in both paths, adjust speed and animate  
                # update current path ONLY to first OR last collision point, keep initial path post collision pt
                print("COLLISION DETECTED")
                arm1_collision = intersect_pts1[0]
                arm2_collision = intersect_pts2[0]
                print(arm1_collision, arm2_collision)
                path1_col_idx = np.where(path1 == arm1_collision)[0][0]
                path2_col_idx = np.where(path2 == arm2_collision)[0][0]

                # choose whether to speed up arm nearest or furthest to goal
                # update paths such that speed is inc/dec until collision point only
                path1, path2 = update_velocity(p_fast=path1, p_slow=path2, vel=INC_VEL, idx_fast=path1_col_idx, idx_slow=path2_col_idx)
                logger.info("INCREASED {} VELOCITY, DECREASED {} VELOCITY".format(arm1.get_name(), arm2.get_name()))

            check_collision = False
        else:
            if (path1[0] == arm1_collision).all() or (path2[0] == arm2_collision).all():
                print("RESETTING VELOCITY")
                plt.plot(path1[0,0], path1[0,1], path1[0,2], 'o', color='yellow', markersize=5)
                path1, path2 = update_velocity(p_fast=path1, p_slow=path2, vel=INIT_VEL)
                arm1_collision, arm2_collision = np.empty(3), np.empty(3)
                check_collision = True

        # print("PLOTTING {}, {}".format(path1[0], path2[0]))
        if idx1 < path1.shape[0]:
            plt.plot(path1[0,0], path1[0,1], path1[0,2], 'o', color='red', markersize=1)
            path1 = np.delete(path1, 0, axis=0)
            arm1.set_position(path1[0])    # start at pt. 0 of path1
        if idx2 < path2.shape[0]:
            plt.plot(path2[0,0], path2[0,1], path2[0,2], 'o', color='red', markersize=1)
            path2 = np.delete(path2, 0, axis=0)
            arm2.set_position(path2[0])    # start at pt. 60 of path2

        # i += 1
        plt.pause(0.0005)
        
        # run_path(new_path1, new_path2, arm1, arm2)

    logger.info("INTERSECTIONS: {}".format(intersect_pts1))
    plt.show()

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
        new_path1, new_path2 = adjust_arm_velocity(path1, path2, vel1=0.07, vel=0.08)
        # set new path and last collision point
        logger.info("UPDATED VELOCITY FOR COLLISION AVOIDANCE")
        print("Arm1: {}, Arm2: {}".format(0.07, 0.09))
    else:
        # reset paths velocities if no more intersections
        logger.info("NO COLLISION DETECTED!")
        new_path1, new_path2 = adjust_arm_velocity(path, path2, vel=INIT_VEL)
        print("Arm1: {}, Arm2: {}".format(INIT_VEL, INIT_VEL))
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

if __name__ == '__main__':
    main()