# PROGRAM CONSTANTS AND PARAMETERS
import numpy as np
from enum import Enum

# clearly i didn't spend much time thinking about how to structure this program

##################
# CONSTANTS
##################

# at which collision point do we want to reset the speed?
class ResetPoint(Enum):
    FIRST_POINT = 1
    LAST_POINT = 2

# which arm do we want to icrease vel
class SpeedUpArm(Enum):
    NEAREST_TO_GOAL = 1
    FURTHEST_FROM_GOAL = 2

# when do we want to change speed?
class CollisionUpdateVel(Enum):
    START = 1
    N_POINTS_BEFORE = 2

ARM1_HOME_POS = np.array([0.0, 1.0, 0.0])
ARM2_HOME_POS = np.array([0.0, -1.0, 0.0])

OBJ2 = np.array([0.0, 1.0, 2.5])
OBJ1 = np.array([-0.5, -1.0, 2.5])
BOWL =  np.array([2, 0.0, 1.0])

INIT_VEL = 0.05
ABS_TOLERANCE = 0.055
THRESHOLD_DIST = 0.5

PAUSE_TIME = 0.0005


####################
# PROGRAM PARAMS
####################

RESET_VELOCITY_AT = ResetPoint.FIRST_POINT
SPEED_UP_ARM = SpeedUpArm.NEAREST_TO_GOAL
UPDATE_VEL = CollisionUpdateVel.START

N_POINTS = 10   # threshold * pt dist? (for UPDATE_VEL_AT)
