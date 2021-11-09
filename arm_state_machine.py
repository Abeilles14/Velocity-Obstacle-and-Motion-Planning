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
from velocity_control import interpolate

logger = logging.getLogger(__name__)
logging.basicConfig()
logger.setLevel(logging.DEBUG)

### CONSTANTS ###
ARM_HOME_POS = [0.0, 0.0, 0.0]
PAUSE_TIME = 0.0001
ABS_TOLERANCE = 0.055
STEP_SIZE = 0.01 #controls speed of paths

class ArmState(Enum):
    APPROACH_OBJECT = 1,
    GRAB_OBJECT = 2,
    APPROACH_DEST = 3,
    DROP_OBJECT = 4,
    HOME = 5,
    DONE = 6

class ArmStateMachine:
    def __init__(self, ax, obstacles, arm, obj, bowl,
                 closed_loop=False, home_when_done=True, log_verbose=True):
        self.ax = ax
        self.obstacles = obstacles
        self.arm = arm
        self.obj = obj
        self.bowl = bowl
        self.closed_loop = closed_loop
        self.home_when_done = home_when_done
        self.log_verbose = log_verbose
        self.path = None
        self.compute_path = True

        self.state = ArmState.APPROACH_OBJECT

        if self.log_verbose:
            logger.debug("ArmStateMachine:__init__")
            logger.debug('Arm: {}, Object: {}'.format(self.arm.get_name(), self.obj.get_name()))

        self.state_functions = {
            ArmState.APPROACH_OBJECT : self._approach_object,
            ArmState.GRAB_OBJECT : self._grab_object,
            ArmState.APPROACH_DEST : self._approach_dest,
            ArmState.DROP_OBJECT : self._drop_object,
            ArmState.HOME : self._home,
        }
        self.next_functions = {
            ArmState.APPROACH_OBJECT : self._approach_object_next,
            ArmState.GRAB_OBJECT : self._grab_object_next,
            ArmState.APPROACH_DEST : self._approach_dest_next,
            ArmState.DROP_OBJECT : self._drop_object_next,
            ArmState.HOME : self._home_next
        }

    ### STATE FUNCTIONS ###
    def _approach_object(self):
        self.arm.set_destination(self.obj.get_position())
        if self.log_verbose:
            logger.debug("{} APPROACHING {} at {}".format(self.arm.get_name(), self.obj.get_name(), self.obj.get_position()))
        self._set_arm_dest(self.obj.get_position())

    def _approach_object_next(self):
        if np.isclose(self.arm.get_position(), self.arm.get_destination(), atol=ABS_TOLERANCE).all():
            self.compute_path = True
            return ArmState.GRAB_OBJECT
        else:
            return ArmState.APPROACH_OBJECT
    
    def _grab_object(self):
        if self.log_verbose:
            logger.debug("{} GRABBING {} at {}".format(self.arm.get_name(), self.obj.get_name(), self.obj.get_position()))

    def _grab_object_next(self):
        self.compute_path = True
        self.arm.set_destination(self.bowl)
        return ArmState.APPROACH_DEST

    def _approach_dest(self):
        if self.log_verbose:
            logger.debug("{} APPROACHING DEST at {}".format(self.arm.get_name(), self.arm.get_destination()))
        self._set_arm_dest(self.arm.get_destination())

    def _approach_dest_next(self):
        if np.isclose(self.arm.get_position(), self.arm.get_destination(), atol=ABS_TOLERANCE).all():
            self.compute_path = True
            return ArmState.DROP_OBJECT
        else:
            return ArmState.APPROACH_DEST 

    def _drop_object(self):
        if self.log_verbose:
            logger.debug("{} DROPPING {} at {}".format(self.arm.get_name(), self.obj.get_name(), self.bowl))

    def _drop_object_next(self):
        # early out if this is being controlled by the parent state machine
        self.compute_path = True
        if not self.closed_loop:
            if self.home_when_done:
                self.arm.set_destination(ARM_HOME_POS)
                return ArmState.HOME
            else:
                return ArmState.DONE

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
        if self.log_verbose:
            logger.debug("{} APROACHING HOME at {}".format(self.arm.get_name(), ARM_HOME_POS))
        self._set_arm_dest(self.arm.get_destination())

    def _home_next(self):
        # the home state is used for arm state machines that are completely 
        # finished executing as determined by the parent state machine
        return ArmState.DONE#ArmState.HOME 

    # def halt(self):
    #     # this sets the desired joint position to the current joint position
    #     self._set_arm_dest(self.arm.get_position(), blocking=False)

    def is_done(self):
        if self.home_when_done:
            return self.state == ArmState.HOME
        else:
            return self.state == ArmState.DONE
    ### END STATE FUNCTIONS ###

    def _set_arm_dest(self, dest):
        if self.log_verbose:
            logger.debug("Setting {} dest to {}".format(self.arm.get_name(), dest))
        # Call RRTS algo to plan and execute path
        if not np.isclose(self.arm.get_position(), dest, atol=ABS_TOLERANCE).all():
            if self.compute_path:
                path = RRTStar(self.ax, self.obstacles, self.arm.get_position(), dest)
                self.path = path
                self.compute_path = False
            self._execute_path(self.ax, self.path)

    def _execute_path(self, ax, P):
        logger.info("Executing Path")

        # velocity control
        step = self.arm.get_velocity()
        path = interpolate(P, step)

        self.arm.set_position(np.array([path[0,0], path[0,1], path[0,2]]))
        logger.debug("{} Position: {}".format(self.arm.get_name(), self.arm.get_position()))
        ax.plot(path[0,0], path[0,1], path[0,2], 'o', color='orange', markersize=3)
        self.path = np.delete(path, 0, axis=0)
        plt.pause(PAUSE_TIME)

    def run_once(self):
        if self.log_verbose:
            logger.debug("Running state {}".format(self.state))
        if self.is_done():
            return
        
        # execute the current state
        self.state_functions[self.state]()
        self.state = self.next_functions[self.state]()

    def get_path(self):
        return self.path