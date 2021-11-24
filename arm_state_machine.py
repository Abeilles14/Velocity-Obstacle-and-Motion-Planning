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

from RRTStar import RRTStar
from path_shortening import shorten_path
from obstacles import Parallelepiped
from arm import Arm
from objects import Object
from velocity_control import linear_interpolation
from constants import *

logger = logging.getLogger(__name__)
logging.basicConfig()
logger.setLevel(logging.INFO)

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
        self.collision_point = np.empty(3)

        self.state = ArmState.PLANNING
        self.destination = Goal.OBJ
        self.check_collisions = True

        logger.debug("ArmStateMachine:__init__")
        logger.debug('Arm: {}, Object: {}'.format(self.arm.get_name(), self.obj.get_name()))

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
        # Call RRTS algo to plan and execute path
        logger.info("PLANNING {} PATH TO {}".format(self.arm.get_name(), self.arm.get_destination()))
        if not np.isclose(self.arm.get_position(), self.arm.get_destination(), atol=ABS_TOLERANCE).all():
            if self.compute_path:
                rrts_path = RRTStar(self.ax, self.obstacles, self.arm.get_position(), self.arm.get_destination())
                sampled_path = linear_interpolation(rrts_path, INIT_VEL)
                self.path = sampled_path
                self.compute_path = False   # don't compute path again until destination

    # next state depending on set goal
    def _plan_path_next(self):
        if self.destination == Goal.OBJ:
            return ArmState.APPROACH_OBJECT
        elif self.destination == Goal.BOWL:
            return ArmState.APPROACH_DEST
        elif self.destination == Goal.HOME:
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
        logger.debug("{} GRABBING {} at {}".format(self.arm.get_name(), self.obj.get_name(), self.obj.get_position()))

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
        logger.debug("{} DROPPING {} at {}".format(self.arm.get_name(), self.obj.get_name(), self.bowl))

    def _drop_object_next(self):
        self.destination = Goal.HOME
        self.compute_path = True
        self.arm.set_destination(self.arm.get_home())
        return ArmState.PLANNING

        # elif len(self.world.objects) > 0:
        #     # there are objects left, find one and go to APPROACH_OBJECT
        #     closest_object = None
        #     if self.pick_closest_to_base_frame:
        #         # closest object to base frame
        #         closest_object = min(self.world.objects,
        #                             key=lambda obj : (self.world_to_psm_tf * obj.pos).Norm())
        #     else:
        #         # closest object to current position, only if we're running 
        #         closest_object = min(self.world.objects,
        #                             key=lambda obj : (self.world_to_psm_tf * obj.pos \
        #                                             - self.psm.get_current_position().p).Norm())
        #     self.object = closest_object
        #     return PickAndPlaceState.APPROACH_OBJECT
        # else:
        #     return PickAndPlaceState.HOME

        # return PickAndPlaceState.DROP_OBJECT

    def _home(self):
        logger.debug("{} APROACHING HOME at {}".format(self.arm.get_name(), self.arm.get_home()))
        self._execute_path()

    def _home_next(self):
        if np.isclose(self.arm.get_position(), self.arm.get_destination(), atol=ABS_TOLERANCE).all():
            return ArmState.DONE
        else:
            return ArmState.HOME

    def _done(self):
        return ArmState.DONE

    # def halt(self):
    #     # this sets the desired joint position to the current joint position
    #     self._set_arm_dest(self.arm.get_position(), blocking=False)

    # def is_done(self):
    #     if self.home_when_done:
    #         return self.state == ArmState.HOME
    #     else:
    #         return self.state == ArmState.DONE
    ### END STATE FUNCTIONS ###

    # set arm position and plot
    def _execute_path(self):
        self.arm.set_position(np.array([self.path[0,0], self.path[0,1], self.path[0,2]]))
        logger.debug("{} Position: {}".format(self.arm.get_name(), self.arm.get_position()))

        self.ax.plot(self.path[0,0], self.path[0,1], self.path[0,2], 'o', color='orange', markersize=3)
        self.path = np.delete(self.path, 0, axis=0)     # delete current point from path
        plt.pause(PAUSE_TIME)

    def run_once(self):
        logger.debug("Running state {}".format(self.state))

        # decide whether to check collisions in main after executing current state
        if self.state == ArmState.PLANNING:
            self.check_collisions = True
        else:
            self.check_collisions = False

        # execute the current state
        self.state_functions[self.state]()
        self.state = self.next_functions[self.state]()

        # if at final collision point, go back into planning state to recheck collisions
        # TODO: fix bug here, collision point reached never detected
        if (self.collision_point != np.empty(3)).all:
            if (self.arm.get_position() == self.collision_point).all():
                logger.info("{} Reached Collision Pt {}".format(self.arm.get_name(), self.collision_point))
                logger.info("Resetting Velocity & Checking Collisions...")
                self.collision_point = np.empty(3)  # reset collision point when reached
                self.state = ArmState.PLANNING      # recheck collisions and reset path velocity
                self.check_collisions = True

    def get_path(self):
        return self.path

    def set_path(self, path, point=np.empty(3)):
        self.path = path
        self.collision_point = point
        