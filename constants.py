# PROGRAM CONSTANTS AND PARAMETERS
import numpy as np
from enum import Enum
from objects import Object

# clearly i didn't spend much time thinking about how to structure this program

##################
# CONSTANTS
##################

# at which collision point do we want to reset the speed?
class ResetPoint(Enum):
    FIRST_POINT = 1
    LAST_POINT = 2

# which arm do we want to increase vel
class DeccelArm(Enum):
    NEAREST_TO_GOAL = 1
    FURTHEST_FROM_GOAL = 2
    GOAL_NEAREST_OTHER_ARM = 3

class Interpolate(Enum):
    QUADRATIC = 1
    LOGARITHMIC = 2

ARM1_HOME_POS = np.array([0.0, 1.0, 3.0])
ARM2_HOME_POS = np.array([0.0, -1.0, 3.0])

# OBJECTS
OBJ1 = Object(name="OBJ1", position=[0.0, -1.0, 0.33])
OBJ2 = Object(name="OBJ2", position=[0.0, 0.0, 0.33])
OBJ3 = Object(name="OBJ3", position=[1.5, 1.0, 0.33])
OBJ4 = Object(name="OBJ4", position=[1.0, -2.0, 0.33])

OBJ_LIST = [OBJ1, OBJ2, OBJ3, OBJ4]

BOWL =  np.array([-1, 0.0, 0.6])


OBSTACLE_POSES = [[0, 0, 0]]
OBSTACLE_DIMS  = [[3.9776, 5.5616, 0.3]]    # table dimensions: 0.39776, 0.55616, 0.08 (m)
ARM_DIMS = [0.2, 0.2, 0.2]

INIT_VEL = 0.05
INC_VEL = 0.05
ABS_TOLERANCE = 0.055
COLLISION_RANGE = 0.3 # usually 0.3-0.5
SAFETY_ZONE = 0.1
LAST_INTERVAL = 0.0005

PAUSE_TIME = 0.0005

PICKING_DELAY = 10

####################
# PROGRAM PARAMS
####################

RESET_VELOCITY_AT = ResetPoint.FIRST_POINT
DECCEL_ARM = DeccelArm.GOAL_NEAREST_OTHER_ARM

N_POINTS = 10   # threshold * pt dist? (for UPDATE_VEL_AT)
