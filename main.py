# 3D PICK AND PLACE SIMULATION
import numpy as np
import logging
from math import *
from matplotlib import pyplot as plt
from utils import init_fonts
from obstacles import Parallelepiped
from objects import Object
from arm import Arm
from arm_state_machine import ArmStateMachine, ArmState
from velocity_control import find_intersection, update_velocity, euclidean_distance
from constants import *

logger = logging.getLogger(__name__)
logging.basicConfig()
logger.setLevel(logging.DEBUG)

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
    # bowl
    ax.scatter3D(BOWL[0], BOWL[1], BOWL[2], color='red', s=800)
    # TODO: put in pick_and_place sm
    ax.scatter3D(ARM1_HOME_POS[0], ARM1_HOME_POS[1], ARM1_HOME_POS[2], color='blue', s=100)
    ax.scatter3D(OBJ1[0], OBJ1[1], OBJ1[2], color='#99ccff', s=100)

    ax.scatter3D(ARM2_HOME_POS[0], ARM2_HOME_POS[1], ARM2_HOME_POS[2], color='green', s=100)
    ax.scatter3D(OBJ2[0], OBJ2[1], OBJ2[2], color='#99ff99', s=100)

    arm1 = Arm(name="PSM1", home=ARM1_HOME_POS, position=ARM1_HOME_POS, destination=OBJ1, velocity=INIT_VEL)
    arm2 = Arm(name="PSM2", home=ARM2_HOME_POS, position=ARM2_HOME_POS, destination=OBJ2, velocity=INIT_VEL)
    obj1 = Object(name="OBJ1", arm=arm1, position=OBJ1)
    obj2 = Object(name="OBJ2", arm=arm2, position=OBJ2)
    arm1_sm = ArmStateMachine(ax, obstacles, arm1, obj1, BOWL)
    arm2_sm = ArmStateMachine(ax, obstacles, arm2, obj2, BOWL)

    logger.info("Pick and Place Simulation Start")

    arm1_collision = np.empty(3)
    arm2_collision = np.empty(3)
    #### MAIN LOOP ####
    while (arm1_sm.state != ArmState.DONE) or (arm2_sm.state != ArmState.DONE): #should be HOME
        arm1_sm.run_once()
        arm2_sm.run_once()
        
        # check for intersection
        if arm1_sm.check_collisions or arm2_sm.check_collisions:
            logger.info("Checking Collisions...")
            path1 = arm1_sm.get_path()
            path2 = arm2_sm.get_path()

            intersect_pts1, intersect_pts2 = find_intersection(path1, path2, arm1, arm2)

            # if collision detected, adjust path velocities
            if (intersect_pts1.shape[0] != 0) and (intersect_pts2.shape[0] != 0):
                logger.info("COLLISION DETECTED!")
                logger.debug("INTERSECTIONS: {}, SHAPE: {}".format(intersect_pts2, intersect_pts2.shape[0]))

                # set collision point as first or last collision point in intersection pts
                #TODO: fix depending on arm speedup??? don't want to access collision_point directly
                if RESET_VELOCITY_AT == ResetPoint.LAST_POINT:
                    arm1_collision = intersect_pts1[-1,:]
                    arm2_collision = intersect_pts2[-1,:]
                elif RESET_VELOCITY_AT == ResetPoint.FIRST_POINT:
                    arm1_collision = intersect_pts1[0,:]
                    arm2_collision = intersect_pts2[0,:]

                logger.info("COLLISION POINTS: {}, {}".format(arm1_collision, arm2_collision))

                # update current path ONLY to first OR last collision point, keep initial path post collision pt
                path1_col_idx = np.where(path1 == arm1_collision)[0][0]
                path2_col_idx = np.where(path2 == arm2_collision)[0][0]

                # choose whether to speed up arm nearest or furthest to goal
                # update paths such that speed is inc/dec until collision point only
                if SPEED_UP_ARM == SpeedUpArm.NEAREST_TO_GOAL:
                    if path1.shape[0] < path2.shape[0]: # arm1 nearer to goal, speed up arm1
                        new_path1, new_path2 = update_velocity(p_fast=path1, p_slow=path2, vel=INC_VEL, idx_fast=path1_col_idx, idx_slow=path2_col_idx)
                        logger.info("INCREASED {} VELOCITY, DECREASED {} VELOCITY".format(arm1.get_name(), arm2.get_name()))
                    else:  # arm2 nearer to goal, speed up arm2
                        new_path2, new_path1 = update_velocity(p_fast=path2, p_slow=path1, vel=INC_VEL, idx_fast=path2_col_idx, idx_slow=path1_col_idx)
                        logger.info("INCREASED {} VELOCITY, DECREASED {} VELOCITY".format(arm2.get_name(), arm1.get_name()))
                elif SPEED_UP_ARM == SpeedUpArm.FURTHEST_FROM_GOAL:
                    if path1.shape[0] > path2.shape[0]: # arm1 further from goal, speed up arm1
                        new_path1, new_path2 = update_velocity(p_fast=path1, p_slow=path2, vel=INC_VEL, idx_fast=path1_col_idx, idx_slow=path2_col_idx)
                        logger.info("INCREASED {} VELOCITY, DECREASED {} VELOCITY".format(arm1.get_name(), arm2.get_name()))
                    else:  # arm2 further to goal, speed up arm2
                        new_path2, new_path1 = update_velocity(p_fast=path2, p_slow=path1, vel=INC_VEL, idx_fast=path2_col_idx, idx_slow=path1_col_idx)
                        logger.info("INCREASED {} VELOCITY, DECREASED {} VELOCITY".format(arm2.get_name(), arm1.get_name()))
            else:
                 # check if common goal
                if euclidean_distance(path1[-1,:], path2[-1,:]) <= THRESHOLD_DIST:
                    logger.info("ARMS MOVING TOWARD COMMON GOAL")
                    new_path1, new_path2 = common_goal_collision(path1, path2, arm1, arm2)
                else:
                    # reset paths velocities if no more intersections
                    logger.info("NO COLLISION DETECTED!")
                    new_path1, new_path2 = update_velocity(path1, path2, vel=INIT_VEL)
                    arm1_collision, arm2_collision = np.empty(3), np.empty(3)
            
            # set new path and collision point
            arm1_sm.set_path(new_path1, arm1_collision)
            arm2_sm.set_path(new_path2, arm2_collision)

    logger.info("Pick and Place Simulation End")

def common_goal_collision(path1, path2, arm1, arm2):
    # arm1_collision = path1[-1,:]
    # arm2_collision = path2[-1,:]

    if path1.shape[0] < path2.shape[0]: # arm1 nearer to goal, speed up arm1
        new_path1, new_path2 = update_velocity(p_fast=path1, p_slow=path2, vel=INC_VEL, idx_fast=path1.shape[0]-1, idx_slow=path2.shape[0]-1)
        logger.info("INCREASED {} VELOCITY, DECREASED {} VELOCITY".format(arm1.get_name(), arm2.get_name()))
    else:  # arm2 nearer to goal, speed up arm2
        new_path1, new_path2 = update_velocity(p_fast=path2, p_slow=path1, vel=INC_VEL, idx_fast=path2.shape[0]-1, idx_slow=path1.shape[0]-1)
        logger.info("INCREASED {} VELOCITY, DECREASED {} VELOCITY".format(arm1.get_name(), arm2.get_name()))
    return new_path1, new_path2

if __name__ == '__main__':
    main()
