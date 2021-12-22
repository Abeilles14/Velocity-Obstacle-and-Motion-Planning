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
    CONSTANT = 1
    QUADRATIC = 2
    LOGARITHMIC = 3
    NATURAL = 4

### RRTStar Parameters ###
# maxiters  = 5000
# minDistGoal = 0.05 # Convergence criterion: success when the tree reaches within 0.25 in distance from the goal.
# d = 0.1#0.5 # [m], Extension parameter: this controls how far the RRT extends in each step.
    

# OBJECTS
# OBJ1 = Object(name="OBJ1", position=[0.0, -1.8, 0.33])
# OBJ2 = Object(name="OBJ2", position=[0.5, 0.0, 0.33])
# OBJ3 = Object(name="OBJ3", position=[1.5, 1.8, 0.33])
# OBJ4 = Object(name="OBJ4", position=[1.0, -2.0, 0.33])

# OBJ_LIST = [OBJ1, OBJ2, OBJ3, OBJ4]

COLORS = ['#458B74', '#8470FF', '#3D59AB', '#DC143C', '#CD1076', 'green', 
            'orange', '#87CEFA', '#AB82FF']

ARM1_HOME_POS = np.array([0.0, 1.0, 2.0])
ARM2_HOME_POS = np.array([0.0, -1.0, 2.0])

BOWL =  np.array([-1, 0.0, 0.8])


OBSTACLE_POSES = [[0, 0, 0]]
OBSTACLE_DIMS  = [[3.9776, 5.5616, 0.3]]    # table dimensions: 0.39776, 0.55616, 0.08 (m)
TEMP_OBS = [0.6, 0.6, 0.6]  # 0.6-0.7, init = 0.6, eval = 0.6

INIT_VEL = 0.08  # 0.05-0.08, init = 0.08, eval = 0.16 (0.08x1.8)
INC_VEL = INIT_VEL
ABS_TOLERANCE = 0.055
COLLISION_RANGE = 0.3  # 0.3-0.5, init = 0.3, eval = 0.3
SAFETY_ZONE = 0.3  # 0.1-0.3, init = 0.3, eval = 0.3
LAST_INTERVAL = 0.0005
MULTIPLIER = 0.008  # init = 0.008  # higher % = slower change, eval = 0.008
OBJ_SPACING = 0.4   # 0.3-0.5, init = 0.3, eval = 0.4

PAUSE_TIME = 0.0001 # 0.0001-0.0005, init = 0.0005, eval = 0.0001

PICKING_DELAY = 10

####################
# PROGRAM PARAMS
####################

RESET_VELOCITY_AT = ResetPoint.LAST_POINT  # init = FIRST_POINT
DECCEL_ARM = DeccelArm.GOAL_NEAREST_OTHER_ARM
VELOCITY_FUNC = Interpolate.NATURAL

