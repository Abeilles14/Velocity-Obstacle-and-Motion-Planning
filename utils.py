import numpy as np
from matplotlib import path
import matplotlib.pyplot as plt
from numpy.linalg import norm
import math

from constants import LAST_INTERVAL

def get_objects_for_arms(self, objects, arm_positions):
        '''
        Returns a dict of PSM index -> list of objects that are closest to that PSM
        '''
        result = dict()  # format: {0: [obj1, obj4], 1: [obj2, obj3]}
        for obj in objects:
            # find psm closest to obj, returns 0 or 1
            closest_arm_idx = arm_positions.index(
                min(arm_positions, key=lambda pos : (pos * obj).Norm()))
            
            if closest_arm_idx not in result:
                result[closest_arm_idx] = list()
            
            result[closest_arm_idx].append(obj)

        return result

def get_nearest_object(objects, arm_position):
    if not np.any(objects):
        return None

    closest_object = min(objects, key=lambda obj : norm((arm_position * obj.get_position())))
    return closest_object        

# Convert xyz-data to a parametrized curve
# calculate all distances between the points
# generate the coordinates on the curve by cumulative summing
# interpolate the x- and y-coordinates independently with respect to the new coords
def linear_interpolation(P, step, show_interplot=False):
    x, y, z = P.T
    xd = np.diff(x)
    yd = np.diff(y)
    zd = np.diff(z)
    dist = np.sqrt(xd**2 + yd**2 + zd**2)
    _u = np.cumsum(dist)
    u = np.hstack([[0], _u])

    # for const speed:
    t = np.arange(0, u.max()+step, step)
    xn = np.interp(t, u, x)
    yn = np.interp(t, u, y)
    zn = np.interp(t, u, z)

    if show_interplot:
        f = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot(x, y, z, 'o', alpha=0.3)
        ax.plot(xn, yn, zn, 'ro', markersize=2)
        ax.set_xlim([-2.5, 2.5])
        ax.set_ylim([-2.5, 2.5])
        ax.set_zlim([0.0, 3.0])
    
    path = np.column_stack((xn, yn, zn))

    return path

def nonlinear_interpolation(P, step, show_interplot=False):
    x, y, z = P.T
    xd = np.diff(x)
    yd = np.diff(y)
    zd = np.diff(z)
    dist = np.sqrt(xd**2 + yd**2 + zd**2)
    _u = np.cumsum(dist)
    u = np.hstack([[0], _u])
    t = natural_range(0, u.max()+step, step)
    xn = np.interp(t, u, x)
    yn = np.interp(t, u, y)
    zn = np.interp(t, u, z)

    if show_interplot:
        f = plt.figure()
        ax = plt.axes(projection='3d')
        ax.plot(x, y, z, 'o', alpha=0.3)
        ax.plot(xn, yn, zn, 'ro', markersize=2)
        ax.set_xlim([-2.5, 2.5])
        ax.set_ylim([-2.5, 2.5])
        ax.set_zlim([0.0, 3.0])
    
    path = np.column_stack((xn, yn, zn))

    return path


def quadratic_range(lb, ub, step):
    li = 0 # last interval (min accel)
    
    # first, solve equation system to find tf, a, b, c, where tf is the total # of points on new path
    # c = lb
    # a + b + c = step
    # a*tf**2 + b*tf + c = ub
    # 2*a*tf + b = li

    b = 2*ub - np.sqrt(4*ub**2 - 4*ub*step + li**2)
    a = step - b
    c = 0
    tf = (li - b)/(2*(step - b))  # tf = total # of samples
    
    arr = []
    for x in range(round(tf)):
        # quadratic equation:
        y = a*x**2 + b*x + c
        arr.append(y)

    return np.array(arr)

def log_range(lb, ub, step):
    li = 0.01 # last interval (min accel)
    
    # first, solve equation system to find a, t, b, c
    # y = logA(t-B) + c
    # y' = 1/(ln(A)*(t-B))
    # choose A by tuning while A>1, start with 1.2 maybe, more gradual decel small A, fast decel big A
    # C=LN(li*LN(A))/LN(A)+YN
    # B = 1 - A^(Y1-C)
    # tf = a^(lp-c)+b

    a = 4
    c = (math.log(li*math.log(a))/math.log(a)) + ub
    b = 1 - a**(step - c)
    tf = a**(ub-c)+b  # tf = total # of samples

    arr = []
    for x in range(round(tf)):
        # log equation:
        y = math.log((x-b), a) + c
        # y = 1/(math.log(a) *(x-b))
        arr.append(y)

    return np.array(arr)

def natural_range(lb, ub, step):
    li = 0.0005 # last interval (min accel)
    
    # first, solve equation system to find k, t, tf
    mult = 0.01866  # 1.866 %
    k = ub*(1+mult)
    t = -1/math.log(1-step/k)
    tf = math.log(1-ub/k)*(-t)
    
    arr = []
    for x in range(round(tf)):
        # natural equation:
        y = k*(1- math.e **((-x)/t))
        arr.append(y)

    return np.array(arr)
    
def isCollisionFreeVertex(obstacles, xy):
    collFree = True

    for obstacle in obstacles:
        hull = path.Path(obstacle)
        collFree = not hull.contains_points([xy])
        if hull.contains_points([xy]):
#             print 'collision'
            return collFree

    return collFree

def isCollisionFreeEdge(obstacles, closest_vert, xy):
    closest_vert = np.array(closest_vert); xy = np.array(xy)
    collFree = True
    l = norm(closest_vert - xy)
    map_resolution = 0.01; M = int(l / map_resolution)
    if M <= 2: M = 3
    t = np.linspace(0,1,M)
    for i in range(1,M-1):
        p = (1-t[i])*closest_vert + t[i]*xy # calculate configuration
        collFree = isCollisionFreeVertex(obstacles, p) 
        if collFree == False: return False

    return collFree


def init_fonts():
    SMALL_SIZE = 12
    MEDIUM_SIZE = 16
    BIGGER_SIZE = 26

    plt.rc('font', size=SMALL_SIZE)          # controls default text sizes
    plt.rc('axes', titlesize=BIGGER_SIZE)    # fontsize of the axes title
    plt.rc('axes', labelsize=MEDIUM_SIZE)    # fontsize of the x and y labels
    plt.rc('xtick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
    plt.rc('ytick', labelsize=SMALL_SIZE)    # fontsize of the tick labels
    plt.rc('legend', fontsize=MEDIUM_SIZE)   # legend fontsize
    plt.rc('figure', titlesize=BIGGER_SIZE)  # fontsize of the figure title