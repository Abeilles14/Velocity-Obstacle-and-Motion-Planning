# PROGRAM CONSTANTS AND PARAMETERS
from enum import Enum


##################
# CONSTANTS
##################

# at which collision point do we want to reset the speed?
class ResetVelPoint(Enum):
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


####################
# PROGRAM PARAMS
####################

RESET_VELOCITY_AT = 2
SPEED_UP_ARM = 1
UPDATE_VEL_AT = 1

N_POINTS = 10   # threshold * pt dist?