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

# which arm do we want to icrease vel
class SpeedUpArm(Enum):
    NEAREST_TO_GOAL = 1
    FURTHEST_FROM_GOAL = 2

class Interpolate(Enum):
    QUADRATIC = 1
    LOGARITHMIC = 2

ARM1_HOME_POS = np.array([0.0, 1.0, 3.0])
ARM2_HOME_POS = np.array([0.0, -1.0, 3.0])

# OBJECTS
OBJ1 = Object(name="OBJ1", position=[-0.5, -1.0, 0.35])
OBJ2 = Object(name="OBJ2", position=[0.0, 1.0, 0.35])
OBJ3 = Object(name="OBJ3", position=[-2, -1.0, 0.35])
OBJ4 = Object(name="OBJ4", position=[1, -1.0, 0.35])

# OBJ2 = np.array([0.0, 1.0, 0.35])
# OBJ1 = np.array([-0.5, -1.0, 0.35])
# OBJ_LIST = np.array([[0.0, 1.0, 0.35], [-0.5, -1.0, 0.35], [-2, -1.0, 0.35], [1, -1.0, 0.35]])
OBJ_LIST = [OBJ1, OBJ2, OBJ3, OBJ4]

BOWL =  np.array([2, 0.0, 0.4])

# obstacles_poses = [[-0.8, 0., 1.5], [ 0., 1., 1.5], [ 0.,-1., 1.5]]
# obstacles_dims  = [[1.4, 1.0, 0.3], [3.0, 1.0, 0.3], [3.0, 1.0, 0.3]]
OBSTACLE_POSES = [[0, 0, 0]]
OBSTACLE_DIMS  = [[5.0, 3.0, 0.3]]

INIT_VEL = 0.08
INC_VEL = 0.08
ABS_TOLERANCE = 0.055
COLLISION_RANGE = 0.3 # 0.5 causes problems when going towards/away from common point
SAFETY_ZONE = 0.1
LAST_INTERVAL = 0.0005

PAUSE_TIME = 0.0005


####################
# PROGRAM PARAMS
####################

RESET_VELOCITY_AT = ResetPoint.FIRST_POINT
SPEED_UP_ARM = SpeedUpArm.NEAREST_TO_GOAL

N_POINTS = 10   # threshold * pt dist? (for UPDATE_VEL_AT)
