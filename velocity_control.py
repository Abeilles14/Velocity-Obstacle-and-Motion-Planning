# COLLISION AVOIDANCE MECHANISM
import numpy as np
import matplotlib.pyplot as plt
import math
from mpl_toolkits import mplot3d
from enum import Enum
import logging

from RRTStar import RRTStar
from arm import Arm
from objects import Object
from constants import *
from test_nonlinear_velocity import quadratic_interpolation
from utils import *

logger = logging.getLogger(__name__)
logging.basicConfig()
logger.setLevel(logging.INFO)

def find_intersection(path1, path2, arm1, arm2, animate=False):
    animate = False

    # check whether any pts in paths are within threshold
    # idx1 = np.where(path1 == arm1.get_position())[0][0]     # get start index
    # idx2 = np.where(path2 == arm2.get_position())[0][0]     # get start index
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

        if (arm_dist <= COLLISION_RANGE).all():
            intersect_pts1.append([path1[idx1,0], path1[idx1,1], path1[idx1,2]])
            intersect_pts2.append([path2[idx2,0], path2[idx2,1], path2[idx2,2]])
            
            # plt.plot(path1[idx1,0], path1[idx1,1], path1[idx1,2], 'o', color='cyan')
            # plt.plot(path2[idx2,0], path2[idx2,1], path2[idx2,2], 'o', color='cyan')
    
    return (np.array(intersect_pts1), np.array(intersect_pts2))

# path 1 init vel is 0.08, path2 init vel is 0.08

def update_velocity(p_fast, p_slow, vel, idx_fast=None, idx_slow=None):
    """ Update velocity of both arm paths.
    Keyword Arguments:
    path1 -- arm 1 path
    path2 -- arm 2 path
    vel 1 -- arm 1 wanted vel
    vel 2 -- arm 2 wanted vel
    idx -- index at which we want vel to reset to init
    """
    if idx_fast == None and idx_slow == None:     # no collision, reset vels
        fast_path = linear_interpolation(p_fast, vel)
        slow_path = linear_interpolation(p_slow, vel)
    else:
        # choose whether to speed up arm nearest or furthest to goal
        # speed up arm to a constant velocity
        logger.debug("fast path: {}".format(p_fast))
        logger.debug("fast_path indexed: {}".format(p_fast[:idx_fast+1,:]))
        new_col_fast_path = linear_interpolation(p_fast[:idx_fast+1,:], vel)    # interpolate until collision pt 2
        new_col_slow_path = nonlinear_interpolation(p_slow[:idx_slow+1,:], INIT_VEL)

        # concat new vel path to collision and post-collision init vel path
        # this new path has adjusted vel until last collision point, then back to regular vel
        fast_path = np.concatenate((new_col_fast_path, np.delete(p_fast, np.arange(0, idx_fast), axis=0)), axis=0)
        slow_path = np.concatenate((new_col_slow_path, np.delete(p_slow, np.arange(0, idx_slow), axis=0)), axis=0)

    return fast_path, slow_path

def common_goal_collision(path1, path2, arm1, arm2):
    if path1.shape[0] < path2.shape[0]: # arm1 nearer to goal, slow arm2
        new_path1, new_path2 = update_velocity(p_fast=path1, p_slow=path2, vel=INC_VEL, idx_fast=path1.shape[0]-1, idx_slow=path2.shape[0]-1)
        logger.info("DECREASED {} VELOCITY".format(arm2.get_name()))
    else:  # arm2 nearer to goal, slow arm1
        new_path2, new_path1 = update_velocity(p_fast=path2, p_slow=path1, vel=INC_VEL, idx_fast=path2.shape[0]-1, idx_slow=path1.shape[0]-1)
        logger.info("DECREASED {} VELOCITY".format(arm1.get_name()))
    
    return new_path1, new_path2

def adjust_arm_velocity(path1, path2, path1_col_idx, path2_col_idx, arm1, arm2):
    if DECCEL_ARM == DeccelArm.FURTHEST_FROM_GOAL:
        if path1.shape[0] < path2.shape[0]: # arm1 nearer to goal, slow arm2
            arm2.set_velocity(0)
            arm1.set_velocity(INIT_VEL)
            new_path1, new_path2 = update_velocity(p_fast=path1, p_slow=path2, vel=INC_VEL, idx_fast=path1_col_idx, idx_slow=path2_col_idx)
            logger.info("DECREASED {} VELOCITY".format(arm2.get_name()))
        else:  # arm2 nearer to goal, slow arm 1
            arm1.set_velocity(0)
            arm2.set_velocity(INIT_VEL)
            new_path2, new_path1 = update_velocity(p_fast=path2, p_slow=path1, vel=INC_VEL, idx_fast=path2_col_idx, idx_slow=path1_col_idx)
            logger.info("DECREASED {} VELOCITY".format(arm1.get_name()))
    elif DECCEL_ARM == DeccelArm.NEAREST_TO_GOAL:
        if path1.shape[0] > path2.shape[0]: # arm1 further from goal, slow arm2
            arm2.set_velocity(0)
            arm1.set_velocity(INIT_VEL)
            new_path1, new_path2 = update_velocity(p_fast=path1, p_slow=path2, vel=INC_VEL, idx_fast=path1_col_idx, idx_slow=path2_col_idx)
            logger.info("DECREASED {} VELOCITY".format(arm2.get_name()))
        else:  # arm2 further to goal, slow arm1
            arm1.set_velocity(0)
            arm2.set_velocity(INIT_VEL)
            new_path2, new_path1 = update_velocity(p_fast=path2, p_slow=path1, vel=INC_VEL, idx_fast=path2_col_idx, idx_slow=path1_col_idx)
            logger.info("DECREASED {} VELOCITY".format(arm1.get_name()))
    elif DECCEL_ARM == DeccelArm.GOAL_NEAREST_OTHER_ARM:
        # if arm1 goal nearer to arm2 than arm2 goal nearer to arm1, slow arm1
        if euclidean_distance(arm1.get_position(), arm2.get_destination()) >= euclidean_distance(arm2.get_position(), arm1.get_destination()):
            arm1.set_velocity(0)
            arm2.set_velocity(INIT_VEL)
            new_path2, new_path1 = update_velocity(p_fast=path2, p_slow=path1, vel=INC_VEL, idx_fast=path2_col_idx, idx_slow=path1_col_idx)
            logger.info("DECREASED {} VELOCITY".format(arm1.get_name()))
        else:  # if arm1 goal further from arm2 than arm2 goal further from to arm1, slow arm2
            arm2.set_velocity(0)
            arm1.set_velocity(INIT_VEL)
            new_path1, new_path2 = update_velocity(p_fast=path1, p_slow=path2, vel=INC_VEL, idx_fast=path1_col_idx, idx_slow=path2_col_idx)
            logger.info("DECREASED {} VELOCITY".format(arm2.get_name()))


    return new_path1, new_path2

def euclidean_distance(point1, point2):
    distance = np.linalg.norm(point1-point2)
    return distance