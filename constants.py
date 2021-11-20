# PROGRAM CONSTANTS AND PARAMETERS
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


####################
# PROGRAM PARAMS
####################

RESET_VELOCITY_AT = ResetPoint.FIRST_POINT
SPEED_UP_ARM = SpeedUpArm.NEAREST_TO_GOAL
UPDATE_VEL = CollisionUpdateVel.START

N_POINTS = 10   # threshold * pt dist? (for UPDATE_VEL_AT)