# 3D PICK AND PLACE SIMULATION
import numpy as np
import logging
from math import *
from matplotlib import pyplot as plt
from utils import init_fonts, get_nearest_object, add_obstacle
from obstacles import Static_Obstacle
from objects import Object
from arm import Arm
from arm_state_machine import ArmStateMachine, ArmState
from velocity_control import *
from constants import *
import random
from numpy.linalg import norm

logger = logging.getLogger(__name__)
logging.basicConfig()
logger.setLevel(logging.INFO)

### PARAMETERS ###
show_RRT = False

### Objects ###

### Objects ###
def generate_objects():
    obj_list = []
    obj_count = 0

    # range: x E [0.0, 2.0], y E [-2.5, -2.5], z = 0.33
    while obj_count < 6:
        obj_x = random.uniform(-1.0, 1.0)
        obj_y = random.uniform(-1.5, 1.5)
        obj_z = 0.33
        pos = [obj_x, obj_y, obj_z]
        print(pos)

        # if object not within 0.3 dist of other obj, add to list
        if np.any(obj_list):
            valid = True
            for obj in obj_list:
                if euclidean_distance(obj.get_position(), pos) < 0.3:
                    valid = False

            if valid:
                new_obj = Object(name="OBJ{}".format(obj_count+1), position=pos)
                obj_list.append(new_obj)
                obj_count += 1
        else:
            new_obj = Object(name="OBJ{}".format(obj_count+1), position=pos)
            obj_list.append(new_obj)
            obj_count += 1

    for obj in obj_list: print(obj.get_position())
    return obj_list


def add_objects(ax, objects):
    # bowl
    ax.scatter3D(BOWL[0], BOWL[1], BOWL[2], color='red', s=600)

    # home pos
    ax.scatter3D(ARM1_HOME_POS[0], ARM1_HOME_POS[1], ARM1_HOME_POS[2], color='blue', s=100, alpha=0.8)
    ax.scatter3D(ARM2_HOME_POS[0], ARM2_HOME_POS[1], ARM2_HOME_POS[2], color='green', s=100, alpha=0.8)

    for obj in objects:
        pos = obj.get_position()
        ax.scatter3D(pos[0], pos[1], pos[2], color='red', s=70, alpha=0.8)

    return objects

def main():
    ### SET UP ENV ###
    init_fonts()
    fig = plt.figure(figsize=(10,10))
    ax = plt.axes(projection='3d')
    ax.set_xlabel('X, [m]')
    ax.set_ylabel('Y, [m]')
    ax.set_zlabel('Z, [m]')
    ax.set_xlim([-3, 3])
    ax.set_ylim([-3, 3])
    ax.set_zlim([0.0, 4])
    mngr = plt.get_current_fig_manager()
    # win in the lower right corner:
    mngr.window.wm_geometry("+500+0")

    ### SET UP STATIC OBSTACLES AND BOWL ###
    temp_obstacles = []
    obstacles = []
    for pose, dim in zip(OBSTACLE_POSES, OBSTACLE_DIMS):
        obstacles = add_obstacle(obstacles, pose, dim)

    for obstacle in obstacles: obstacle.draw(ax)

    obj_list = generate_objects()
    objects = add_objects(ax, obj_list)
    obj1, obj2 = None, None

    arm1 = Arm(name="BLUE", home=ARM1_HOME_POS, position=ARM1_HOME_POS, destination=ARM1_HOME_POS, velocity=INIT_VEL, color='#57b9fc')
    arm2 = Arm(name="GREEN", home=ARM2_HOME_POS, position=ARM2_HOME_POS, destination=ARM2_HOME_POS, velocity=INIT_VEL, color='#3baf4f')

    arm1_sm = ArmStateMachine(ax, obstacles, arm1, obj1, BOWL)
    arm2_sm = ArmStateMachine(ax, obstacles, arm2, obj2, BOWL)

    logger.info("Pick and Place Simulation Start")

    # default
    arm1_collision = np.empty(3)
    arm2_collision = np.empty(3)
   
    #### MAIN LOOP ####
    while (arm1_sm.state != ArmState.DONE) or (arm2_sm.state != ArmState.DONE):
        # if arm done picking last obj (full cycle), set next obj
        # get object nearest to each arm and set each arm sm
        if arm1_sm.pick_ready:
            obj1 = get_nearest_object(objects, arm1.get_position())
            arm1_sm.set_object(obj1)
            if obj1 != None:
                objects.remove(obj1)
                logger.info("ASSIGNING {} TO {}".format(arm1.get_name(), obj1.get_name()))
        if arm2_sm.pick_ready:
            obj2 = get_nearest_object(objects, arm2.get_position())
            arm2_sm.set_object(obj2)
            if obj2 != None:
                objects.remove(obj2)
                logger.info("ASSIGNING {} TO {}".format(arm2.get_name(), obj2.get_name()))
        

        arm1_sm.run_once()
        arm2_sm.run_once()

        # if stop count > 8, possible deadlock, share eachother's positions
        if arm1_sm.stop_count > 5 or arm2_sm.stop_count > 5:
            logger.info("OH NO, WE HAVE A DEADLOCK!! >:(")
            logger.info("ARM POS: {} {}, {} {}".format(arm1.get_name(), arm1.get_position(), arm2.get_name(), arm2.get_position()))
            logger.info("ARM DIST: {}".format(euclidean_distance(arm1.get_position(), arm2.get_position())))

            # make fast arm replan path
            if arm1.get_velocity() != INIT_VEL:
                arm2_sm.state = ArmState.PLANNING
                arm2_sm.compute_path = True
                slow_arm_pos = arm1.get_position()
                arm2_sm.collision_point = np.empty(3)
            elif arm2.get_velocity() != INIT_VEL:
                arm1_sm.state = ArmState.PLANNING
                arm1_sm.compute_path = True
                slow_arm_pos = arm2.get_position()
                arm1_sm.collision_point = np.empty(3)

            # create temp obstacle box around slow arm:
            logger.info("Setting Temp Obstacle at: {}".format(slow_arm_pos))
            
            obstacles = add_obstacle(obstacles, pose=slow_arm_pos, dim=ARM_DIMS)
            temp_obstacles.append(obstacles[-1])
            obstacles[-1].draw(ax)
            
            # reset stop counts
            arm1_sm.stop_count = 0
            arm2_sm.stop_count = 0

        path1 = arm1_sm.get_path()  # make sure that paths are already generated
        path2 = arm2_sm.get_path()

        # critical stop if arms in eachother safety zone
        # TODO: delete? not needed?
        # if euclidean_distance(arm1.get_position(), arm2.get_position()) <= 0.3:
        #     if arm1.get_velocity() != INIT_VEL:
        #         logger.info("{} CRITICAL STOP!!".format(arm1.get_name()))
        #         arm1_sm.pause = True
        #     else:
        #         logger.info("{} CRITICAL STOP!!".format(arm2.get_name()))
        #         arm2_sm.pause = True 
        # else:
        #     arm1_sm.pause = False
        #     arm2_sm.pause = False

        # check for intersection
        if arm1_sm.check_collisions or arm2_sm.check_collisions:
            logger.info("Checking Collisions...")
            
            # if one arm's path not computed, simply use current pos to compare
            if path1.shape[0] == 0: path1 = np.array([arm1.get_position()])
            if path2.shape[0] == 0: path2 = np.array([arm2.get_position()])

            # check if common goal
            if euclidean_distance(path1[-1], path2[-1]) <= COLLISION_RANGE:
                logger.info("ARMS MOVING TOWARD COMMON GOAL {}, {}".format(path1[-1], path2[-1]))
                point = ax.scatter3D(path1[-1,0], path1[-1,1], path1[-1,2], color='cyan', s=100)
                arm1_sm.set_temp_graphics(point)
                point = ax.scatter3D(path2[-1,0], path2[-1,1], path2[-1,2], color='cyan', s=100)
                arm2_sm.set_temp_graphics(point)
                
                new_path1, new_path2 = common_goal_collision(path1, path2, arm1, arm2)
            # check for possible collisions
            else:
                intersect_pts1, intersect_pts2 = find_intersection(path1, path2, arm1, arm2)

                # plot collision points
                for col_pt in intersect_pts1:
                    point, = ax.plot(col_pt[0], col_pt[1], col_pt[2], 'o', color='cyan')
                    arm1_sm.set_temp_graphics(point)
                for col_pt in intersect_pts2:
                    point, = ax.plot(col_pt[0], col_pt[1], col_pt[2], 'o', color='cyan')
                    arm2_sm.set_temp_graphics(point)

                # if collision detected, adjust path velocities
                if (intersect_pts1.shape[0] != 0) and (intersect_pts2.shape[0] != 0):
                    logger.info("COLLISION DETECTED!")
                    logger.debug("INTERSECTIONS 1: {}, SHAPE: {}".format(intersect_pts1, intersect_pts1.shape[0]))
                    logger.debug("INTERSECTIONS 2: {}, SHAPE: {}".format(intersect_pts2, intersect_pts2.shape[0]))

                    # set collision point as first or last collision point in intersection pts
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
                    new_path1, new_path2, = adjust_arm_velocity(path1, path2, path1_col_idx, path2_col_idx, arm1, arm2)
                    logger.info("ARM VEL: {}, {}".format(arm1.get_velocity(), arm2.get_velocity()))
                else:
                    # no collision, or reset paths velocities if collision avoided
                    logger.info("NO COLLISION DETECTED!")
                    new_path1, new_path2 = update_velocity(path1, path2, vel=INIT_VEL)
                    arm1.set_velocity(INIT_VEL)
                    arm2.set_velocity(INIT_VEL)
                    arm1_collision, arm2_collision = np.empty(3), np.empty(3)

                    # if previously deadlocked, remove temp obstacles
                    for obstacle in temp_obstacles:
                        obstacles.remove(obstacle)
                    temp_obstacles = []
            
            # set new path, collision point, and slow arm bool
            arm1_sm.set_path(new_path1, arm1_collision)
            arm2_sm.set_path(new_path2, arm2_collision)
        
        plt.pause(PAUSE_TIME)

    logger.info("Pick and Place Simulation End")
    plt.show()

if __name__ == '__main__':
    main()
