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
import logging
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

from RRTStar import RRTStar
from path_shortening import shorten_path
from obstacles import Static_Obstacle
from arm import Arm
from objects import Object
from velocity_control import linear_interpolation, euclidean_distance
from constants import *
from utils import add_obstacle, dump_graphics


logger = logging.getLogger(__name__)
logging.basicConfig()
logger.setLevel(logging.DEBUG)

class ArmState(Enum):
    PLANNING = 0,
    APPROACH_OBJECT = 1,
    GRAB_OBJECT = 2,
    APPROACH_DEST = 3,
    DROP_OBJECT = 4,
    HOME = 5,
    DONE = 6

class Goal(Enum):
    HOME = 0,
    OBJ = 1,
    BOWL = 2

class ArmStateMachine:
    def __init__(self, ax, obstacles, arm, obj, bowl,
                 closed_loop=False, home_when_done=True):
        self.ax = ax
        self.obstacles = obstacles
        self.arm = arm

        self.obj = obj
        self.bowl = bowl

        self.closed_loop = closed_loop
        self.home_when_done = home_when_done
        self.path = []
        self.compute_path = True
        self.pick_ready = True
        self.collision_point = np.empty(3)

        self.state = ArmState.PLANNING
        self.destination = Goal.OBJ
        self.check_collisions = True
        self.pause = False

        self.temp_graphics = []

        self.stop_count = 0
        self.other_arm_pos = None

        logger.debug("ArmStateMachine {}:__init__".format(self.arm.get_name()))

        self.state_functions = {
            ArmState.PLANNING: self._plan_path,
            ArmState.APPROACH_OBJECT: self._approach_object,
            ArmState.GRAB_OBJECT: self._grab_object,
            ArmState.APPROACH_DEST: self._approach_dest,
            ArmState.DROP_OBJECT: self._drop_object,
            ArmState.HOME: self._home,
            ArmState.DONE: self._done
        }
        self.next_functions = {
            ArmState.PLANNING: self._plan_path_next,
            ArmState.APPROACH_OBJECT: self._approach_object_next,
            ArmState.GRAB_OBJECT: self._grab_object_next,
            ArmState.APPROACH_DEST: self._approach_dest_next,
            ArmState.DROP_OBJECT: self._drop_object_next,
            ArmState.HOME: self._home_next,
            ArmState.DONE: self._done
        }

    ### STATE FUNCTIONS ###
    # if arm not at goal, and compute flag, plan path to next dest
    # returns to main loop after this to check for collisions and adjust path velocity
    def _plan_path(self):
        # dump previous graphics
        self.temp_graphics = dump_graphics(self.temp_graphics)
        valid_path = False

        # Call RRTS algo to plan and execute path
        logger.info("PLANNING {} PATH TO {}".format(self.arm.get_name(), self.arm.get_destination()))
        if not np.isclose(self.arm.get_position(), self.arm.get_destination(), atol=ABS_TOLERANCE).all():
            if self.compute_path:
                logger.info("COMPUTING {} NEW PATH PATH TO {}".format(self.arm.get_name(), self.arm.get_destination()))
                
                while(valid_path == False):
                    rrts_path = RRTStar(self.ax, self.obstacles, self.arm.get_position(), self.arm.get_destination())
                    if (rrts_path[-1] == self.arm.get_destination()).all():
                        valid_path = True
                    else:
                        # pauses and shows info if deadlock unsolvable
                        # likely due to arm dist < 0.3 and temp obstacle enclosing both arms
                        # cant create a valid rrts path since arm in obstacle
                        # TODO: fix it eventually - sorry no time to solve it now :(
                        logger.debug("{}, {}, {}".format(self.obstacles, self.arm.get_position(), self.arm.get_destination()))
                        logger.debug(rrts_path)
                        plt.show()

                # draw RRTStar path
                for i in range(rrts_path.shape[0]-1):
                    line_plt, = self.ax.plot([rrts_path[i,0], rrts_path[i+1,0]], [rrts_path[i,1], rrts_path[i+1,1]], [rrts_path[i,2], rrts_path[i+1,2]], color = 'orange', linewidth=1, zorder=15)
                    self.set_temp_graphics(line_plt)
                
                self.path = linear_interpolation(rrts_path, INIT_VEL)
                self.compute_path = False   # don't compute path again until destination

    # next state depending on set goal
    def _plan_path_next(self):
        if self.destination == Goal.OBJ:
            self.arm.set_color(self.arm.get_home_color())
            return ArmState.APPROACH_OBJECT
        elif self.destination == Goal.BOWL:
            self.arm.set_color(self.obj.get_color())
            return ArmState.APPROACH_DEST
        elif self.destination == Goal.HOME:
            self.arm.set_color(self.arm.get_home_color())
            return ArmState.HOME

    def _approach_object(self):
        logger.debug("{} APPROACHING {} at {}".format(self.arm.get_name(), self.obj.get_name(), self.obj.get_position()))
        self._execute_path()

    # if arm is at object, next grab object state
    def _approach_object_next(self):
        if np.isclose(self.arm.get_position(), self.arm.get_destination(), atol=ABS_TOLERANCE).all():
            return ArmState.GRAB_OBJECT
        else:
            return ArmState.APPROACH_OBJECT
    
    def _grab_object(self):
        logger.info("{} GRABBED {} at {}".format(self.arm.get_name(), self.obj.get_name(), self.obj.get_position()))
        
        # remove object from plot and set arm color
        self.obj.get_plot().remove()

    # set next destination and compute path flag, next planning state
    def _grab_object_next(self):
        self.destination = Goal.BOWL
        self.compute_path = True
        self.arm.set_destination(self.bowl)
        return ArmState.PLANNING

    def _approach_dest(self):
        logger.debug("{} APPROACHING DEST at {}".format(self.arm.get_name(), self.arm.get_destination()))
        self._execute_path()

    def _approach_dest_next(self):
        if np.isclose(self.arm.get_position(), self.arm.get_destination(), atol=ABS_TOLERANCE).all():
            return ArmState.DROP_OBJECT
        else:
            return ArmState.APPROACH_DEST 

    def _drop_object(self):
        logger.info("{} DROPPED {} at ".format(self.arm.get_name(), self.obj.get_name(), self.bowl))
        self.pick_ready = True  # get next object to pick in main

    def _drop_object_next(self):
        # set next obj to pick if any
        if self.obj == None:    # if no objects left, go home
            self.destination = Goal.HOME
            self.arm.set_destination(self.arm.get_home())
        else:                   # if object, go pick
            self.destination = Goal.OBJ
            self.arm.set_destination(self.obj.get_position())
        self.compute_path = True
        return ArmState.PLANNING

    def _home(self):
        logger.debug("{} APROACHING HOME at {}".format(self.arm.get_name(), self.arm.get_home()))
        self._execute_path()

    def _home_next(self):
        if np.isclose(self.arm.get_position(), self.arm.get_destination(), atol=ABS_TOLERANCE).all():
            self.temp_graphics = dump_graphics(self.temp_graphics)
            return ArmState.DONE
        else:
            return ArmState.HOME

    def _done(self):
        self.path = np.array([self.arm.get_position()])
        return ArmState.DONE

    ### END STATE FUNCTIONS ###

    # set arm position and plot
    def _execute_path(self):
        self.arm.set_position(self.path[0])
        logger.debug("{} Position: {}".format(self.arm.get_name(), self.arm.get_position()))

        point, = self.ax.plot(self.path[0,0], self.path[0,1], self.path[0,2], 'o', color=self.arm.get_color(), markersize=3)
        self.set_temp_graphics(point)

        # print("current pos: {}".format(self.arm.get_position()))

        self.path = np.delete(self.path, 0, axis=0)     # delete current point from path

    def _pause(self):
        # stay at current position, do nothing
        self.arm.set_position(np.array(self.path[0]))

    def run_once(self):
        logger.debug("{} Running state {}".format(self.arm.get_name(), self.state))

        # decide whether to check collisions in main after executing current state
        if self.state == ArmState.PLANNING:
            self.check_collisions = True
        else:
            self.check_collisions = False

        # check conditions for emergency stop
        self.pause = self.check_emergency_stop()
        
        # set arm color

        # if pause flag, pause arm movement, don't update state or pos
        if self.pause:
            self._pause()
        else:
            # execute the current state
            self.state_functions[self.state]()
            self.state = self.next_functions[self.state]()

            if (self.collision_point != np.empty(3)).all:
                # if at final collision point, go back into planning state to recheck collisions
                if (self.arm.get_position() == self.collision_point).all():
                    logger.info("{} Reached Collision Pt {}".format(self.arm.get_name(), self.collision_point))
                    logger.info("{} Resetting Velocity & Checking Collisions...".format(self.arm.get_name()))
                    self.collision_point = np.empty(3)  # reset collision point when reached
                    self.check_collisions = True

    def check_emergency_stop(self):
        # emergency stop if slow arm reaches dist from collision pt before regular arm
        # didn't spend much time on writing good code here
        if self.arm.get_velocity() != INIT_VEL and euclidean_distance(self.collision_point, self.path[0]) <= SAFETY_ZONE:
            logger.info("{} EMERGENCY STOP AT {}!!".format(self.arm.get_name(), self.arm.get_position()))
            logger.info("{} STOP COUNT: {}".format(self.arm.get_name(), self.stop_count))
            
            self.stop_count += 1    # this gets reset in main, deal with it
            return True

        return False

    def set_object(self, obj):
        self.obj = obj
        self.pick_ready = False

        if self.obj != None:
            self.arm.set_destination(self.obj.get_position())
            self.destination = Goal.OBJ
        else:
            self.arm.set_destination(self.arm.get_home())
            self.destination = Goal.HOME

    def get_path(self):
        return self.path

    def set_path(self, path, point=np.empty(3)):
        self.path = path
        self.collision_point = point

    def set_temp_graphics(self, point):
        self.temp_graphics.append(point)


