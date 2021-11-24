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

logger = logging.getLogger(__name__)
logging.basicConfig()
logger.setLevel(logging.INFO)

# Convert xyz-data to a parametrized curve
# calculate all distances between the points
# generate the coordinates on the curve by cumulative summing
# interpolate the x- and y-coordinates independently with respect to the new coords
def linear_interpolation(P, step, show_interplot=False):
    x, y, z = P.T
    xd = np.diff(x)
    yd = np.diff(y)
    zd = np.diff(z)
    dist = np.sqrt(xd**2 + yd**2 + zd**2)
    _u = np.cumsum(dist)
    u = np.hstack([[0], _u])

    # for const speed:
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

def nonlinear_interpolation(P, step, show_interplot=False):
    x, y, z = P.T
    xd = np.diff(x)
    yd = np.diff(y)
    zd = np.diff(z)
    dist = np.sqrt(xd**2 + yd**2 + zd**2)
    _u = np.cumsum(dist)
    u = np.hstack([[0], _u])

    # for const speed:
    t = quadratic_range(0, u.max(), step)

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


def quadratic_range(lb, ub, step):
    li = 0.01 # last interval (min accel)
    
    # first, solve equation system to find tf, a, b, c
    # c = lb
    # a + b + c = step
    # a*tf**2 + b*tf + c = ub
    # 2*a*tf + b = li

    b = 2*ub - np.sqrt(4*ub**2 - 4*ub*step + li**2)
    a = step - b
    c = 0
    tf = (li - b)/(2*(step - b))  # tf = total # of samples
    
    arr = []
    for x in range(round(tf)):
        # quadratic equation:
        y = a*x**2 + b*x + c
        arr.append(y)

    return np.array(arr)

def logarithmic_range(lb, ub, step):
    li = 0.005

    # first, solve equation system to find a, t, b, c
    # y = logA(t-B) + c
    # y' = 1/(ln(A)*(t-B))
    # choose A by tuning while A>1, start with 1.2 maybe, more gradual decel small A, fast decel big A
    a = 1.2
    c = math.log(li*math.log(a, math.e), math.e)/(math.log(a, math.e) + ub)
    b = 1 - a**(lb - c)

    arr = []
    # for x in range(round(tf)):
    #     # quadratic equation:
    #     y = a*x**2 + b*x + c
    #     arr.append(y)

    # return np.array(arr)

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

        if (arm_dist <= THRESHOLD_DIST).all():
            intersect_pts1.append([path1[idx1,0], path1[idx1,1], path1[idx1,2]])
            intersect_pts2.append([path2[idx2,0], path2[idx2,1], path2[idx2,2]])
            
            plt.plot(path1[idx1,0], path1[idx1,1], path1[idx1,2], 'o', color='cyan')
            plt.plot(path2[idx2,0], path2[idx2,1], path2[idx2,2], 'o', color='cyan')
    
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
        new_col_fast_path = linear_interpolation(p_fast[:idx_fast,:], vel)    # interpolate until collision pt 2
        new_col_slow_path = nonlinear_interpolation(p_slow[:idx_slow,:], INIT_VEL)

        # concat new vel path to collision and post-collision init vel path
        # this new path has adjusted vel until last collision point, then back to regular vel
        fast_path = np.concatenate((new_col_fast_path, np.delete(p_fast, np.arange(0, idx_fast), axis=0)), axis=0)
        slow_path = np.concatenate((new_col_slow_path, np.delete(p_slow, np.arange(0, idx_slow), axis=0)), axis=0)

    return fast_path, slow_path

def euclidean_distance(point1, point2):
    distance = np.linalg.norm(point1-point2)
    return distance