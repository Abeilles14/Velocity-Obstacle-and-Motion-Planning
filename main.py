# STATE MACHINE FOR 3D PICK AND PLACE SIMULATION
import numpy as np
from numpy.linalg import norm
from math import *
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon
from random import random
from scipy.spatial import ConvexHull
from matplotlib import path
import time
from mpl_toolkits import mplot3d
from enum import Enum

from utils import init_fonts
from path_shortening import shorten_path
from obstacles import Parallelepiped
from objects import Object
from arm import Arm
from rrt3D import RRTStar
from arm_state_machine import ArmStateMachine, ArmState

### CONSTANTS ###
pause_time = 0.0005

# Object start and goal positions
# start = np.array([0.0, 0.0, 0.0])
# goal =  np.array([0.0, 0.5, 2.5])
OBJ1_START = np.array([0.0, 0.0, 0.0])
OBJ1_GOAL =  np.array([0.0, 1, 2.5])

### PARAMETERS ###
show_RRT = False

### Obstacles ###
def add_obstacle(obstacles, pose, dim):
	obstacle = Parallelepiped()
	obstacle.dimensions = dim
	obstacle.pose = pose
	obstacles.append(obstacle)
	return obstacles

def main():
    ### SET UP ENV ###
    init_fonts()
    fig = plt.figure(figsize=(15,15))
    ax = plt.axes(projection='3d')
    ax.set_xlabel('X, [m]')
    ax.set_ylabel('Y, [m]')
    ax.set_zlabel('Z, [m]')
    ax.set_xlim([-2.5, 2.5])
    ax.set_ylim([-2.5, 2.5])
    ax.set_zlim([0.0, 3.0])

     ### SET UP OBSTACLES ###
    # obstacles_poses = [ [-0.8, 0., 1.5], [ 1., 0., 1.5], [ 0., 1., 1.5], [ 0.,-1., 1.5] ]
    # obstacles_dims  = [ [1.4, 1.0, 0.2], [1.0, 1.0, 0.2], [3.0, 1.0, 0.2], [3.0, 1.0, 0.2] ]
    obstacles_poses = [[-0.8, 0., 1.5], [ 0., 1., 1.5], [ 0.,-1., 1.5]]
    obstacles_dims  = [[1.4, 1.0, 0.3], [3.0, 1.0, 0.3], [3.0, 1.0, 0.3]]

    obstacles = []
    for pose, dim in zip(obstacles_poses, obstacles_dims):
        obstacles = add_obstacle(obstacles, pose, dim)

    for obstacle in obstacles: obstacle.draw(ax)

    arm1 = Arm("PSM1", 5, OBJ1_START)
    obj1 = Object("OBJ1", arm1, OBJ1_START, OBJ1_GOAL)
    arm1_sm = ArmStateMachine(ax, obstacles, arm1, obj1)
    # RRTStar(ax, obstacles, OBJ1_START, OBJ1_GOAL)

    while (arm1_sm.state != ArmState.DONE): #should be HOME
        arm1_sm.run_once()

if __name__ == '__main__':
    main()
