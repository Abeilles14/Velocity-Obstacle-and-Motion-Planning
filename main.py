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
from RRTStar import RRTStar
from arm_state_machine import ArmStateMachine, ArmState

### CONSTANTS ###
pause_time = 0.0005
ARM1_HOME_POS = np.array([0.0, 0.0, 0.0])
OBJ1 = np.array([0.0, 1.0, 2.5])
BOWL =  np.array([2, 0.0, 1.0])


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
    fig = plt.figure(figsize=(10,10))
    ax = plt.axes(projection='3d')
    ax.set_xlabel('X, [m]')
    ax.set_ylabel('Y, [m]')
    ax.set_zlabel('Z, [m]')
    ax.set_xlim([-2.5, 2.5])
    ax.set_ylim([-2.5, 2.5])
    ax.set_zlim([0.0, 3.0])

    ### SET UP STATIC OBSTACLES AND BOWL ###
    # obstacles_poses = [ [-0.8, 0., 1.5], [ 1., 0., 1.5], [ 0., 1., 1.5], [ 0.,-1., 1.5] ]
    # obstacles_dims  = [ [1.4, 1.0, 0.2], [1.0, 1.0, 0.2], [3.0, 1.0, 0.2], [3.0, 1.0, 0.2] ]
    obstacles_poses = [[-0.8, 0., 1.5], [ 0., 1., 1.5], [ 0.,-1., 1.5]]
    obstacles_dims  = [[1.4, 1.0, 0.3], [3.0, 1.0, 0.3], [3.0, 1.0, 0.3]]

    obstacles = []
    for pose, dim in zip(obstacles_poses, obstacles_dims):
        obstacles = add_obstacle(obstacles, pose, dim)

    for obstacle in obstacles: obstacle.draw(ax)

    ### SET UP OBJECTS ###
    ax.scatter3D(BOWL[0], BOWL[1], BOWL[2], color='red', s=800)
    # TODO: put in pick_and_place sm
    ax.scatter3D(ARM1_HOME_POS[0], ARM1_HOME_POS[1], ARM1_HOME_POS[2], color='blue', s=100)
    ax.scatter3D(OBJ1[0], OBJ1[1], OBJ1[2], color='green', s=100)

    arm1 = Arm("PSM1", 5, ARM1_HOME_POS, OBJ1)
    obj1 = Object("OBJ1", arm1, OBJ1)
    arm1_sm = ArmStateMachine(ax, obstacles, arm1, obj1, BOWL)

    # RRTStar(ax, obstacles, ARM1_HOME_POS, OBJ1)

    while (arm1_sm.state != ArmState.DONE): #should be HOME
        arm1_sm.run_once()

if __name__ == '__main__':
    main()
