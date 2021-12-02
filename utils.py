import numpy as np
from matplotlib.patches import Polygon
from scipy.spatial import ConvexHull
from matplotlib import path
import matplotlib.pyplot as plt
from numpy.linalg import norm
import math

from constants import LAST_INTERVAL

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
    print("lengths: {}, {}, {}".format(len(t),len(u),len(x)))
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

    t = quadratic_range(0, u.max()+step, step)
    print("lengths: {}, {}, {}".format(len(t),len(u),len(x)))
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
    li = LAST_INTERVAL # last interval (min accel)
    
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

def logarithmic_range(lb, ub, step):
    li = LAST_INTERVAL

    # first, solve equation system to find a, t, b, c
    # y = logA(t-B) + c
    # y' = 1/(ln(A)*(t-B))
    # choose A by tuning while A>1, start with 1.2 maybe, more gradual decel small A, fast decel big A
    a = 1.2
    c = math.log(li*math.log(a, math.e), math.e)/(math.log(a, math.e) + ub)
    b = 1 - a**(lb - c)

    arr = []
    # for x in range(round(tf)):
    #     # quadratic equation:
    #     y = a*x**2 + b*x + c
    #     arr.append(y)

    # return np.array(arr)
    
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