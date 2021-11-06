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

from utils import init_fonts
from rrt3D import RRTStar
from path_shortening import shorten_path
from obstacles import Parallelepiped
from arm import Arm
from objects import Object

### CONSTANTS ###
ARM_HOME_POS = [0.0, 0.0, 0.0]    #TODO

class ArmState(Enum):
    APPROACH_OBJECT = 1,
    GRAB_OBJECT = 2,
    APPROACH_DEST = 3,
    DROP_OBJECT = 4,
    HOME = 5,
    DONE = 6

class ArmStateMachine:
    def __init__(self, ax, obstacles, arm, obj, log_verbose=True):
        self.state = ArmState.APPROACH_OBJECT
        self.arm = arm
        self.obj = obj
        self.log_verbose = log_verbose
        self.ax = ax
        self.obstacles = obstacles

        if self.log_verbose:
            logging.debug("ArmStateMachine:__init__")
            logging.debug('Arm: {}, Object: {}'.format(self.arm.get_name(), self.obj.get_name()))

        self.state_functions = {
            ArmState.APPROACH_OBJECT : self._approach_object,
            ArmState.GRAB_OBJECT : self._grab_object,
            ArmState.APPROACH_DEST : self._approach_dest,
            ArmState.DROP_OBJECT : self._drop_object,
            # ArmState.HOME : self._home,
        }
        self.next_functions = {
            ArmState.APPROACH_OBJECT : self._approach_object_next,
            ArmState.GRAB_OBJECT : self._grab_object_next,
            ArmState.APPROACH_DEST : self._approach_dest_next,
            ArmState.DROP_OBJECT : self._drop_object_next,
            # ArmState.HOME : self._home_next
        }

    ### STATE FUNCTIONS ###
    def _approach_object(self):
        if self.log_verbose:
            logging.debug("Arm {} approaching Obj {} at {}".format(self.arm.get_name(), self.obj.get_name(), self.obj.position()))
        self._set_arm_dest(self.obj.position())

    def _approach_object_next(self):
        if (self.arm.position() == self.obj.position()).all():
            return ArmState.GRAB_OBJECT
        else:
            return ArmState.APPROACH_OBJECT
    
    def _grab_object(self):
        if self.log_verbose:
            logging.debug("Arm {} grabbing Obj {} at {}".format(self.arm.get_name(), self.obj.get_name(), self.obj.position()))

    def _grab_object_next(self):
        return ArmState.APPROACH_DEST

    def _approach_dest(self):
        if self.log_verbose:
            logging.debug("Arm {} approaching Obj {} goal at {}".format(self.arm.get_name(), self.obj.get_name(), self.obj.goal()))
        self._set_arm_dest(self.obj.goal())

    def _approach_dest_next(self):
        if (self.arm.position() == self.obj.goal()).all():
            return ArmState.DROP_OBJECT
        else:
            return ArmState.APPROACH_DEST 

    def _drop_object(self):
        if self.log_verbose:
            logging.debug("Arm {} dropping Obj {} at {}".format(self.arm.get_name(), self.obj.get_name(), self.obj.position()))

    def _drop_object_next(self):
        return ArmState.DONE
        # early out if this is being controlled by the parent state machine
        # if not self.closed_loop:
        #     if self.home_when_done:
        #         return PickAndPlaceState.HOME
        #     else:
        #         return PickAndPlaceState.DONE

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

        #return PickAndPlaceState.DROP_OBJECT

    # def _home(self):
    #     self._set_arm_dest(ARM_HOME_POS)

    # def _home_next(self):
    #     # the home state is used for arm state machines that are completely 
    #     # finished executing as determined by the parent state machine
    #     return ArmState.HOME 

    # def halt(self):
    #     # this sets the desired joint position to the current joint position
    #     self.psm.move(self.psm.get_current_position(), blocking=False)

    # def is_done(self):
    #     if self.home_when_done:
    #         return self.state == PickAndPlaceState.HOME and vector_eps_eq(self.psm.get_current_position().p, PSM_HOME_POS) 
    #     else:
    #         return self.state == PickAndPlaceState.DONE
    ### END STATE FUNCTIONS ###

    def _set_arm_dest(self, dest):
        if self.log_verbose:
            logging.debug("Setting arm {} dest to {}".format(self.arm.get_name(), dest))
        # Call RRTS algo to plan and execute path
        if (self.arm.position() != dest).all():
            RRTStar(self.ax, self.obstacles, self.arm.position(), dest)

    def run_once(self):
        if self.log_verbose:
            logging.debug("Running state {}".format(self.state))
        if self.state == ArmState.DONE: #self.is_done():
            return
        # execute the current state
        self.state_functions[self.state]()

        self.state = self.next_functions[self.state]()
