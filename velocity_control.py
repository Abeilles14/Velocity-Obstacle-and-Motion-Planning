# COLLISION AVOIDANCE MECHANISM
import numpy as np
import matplotlib.pyplot as plt
import time
from mpl_toolkits import mplot3d
from enum import Enum
import logging

from RRTStar import RRTStar
from arm import Arm
from objects import Object

# Convert xyz-data to a parametrized curve
# calculate all distances between the points
# generate the coordinates on the curve by cumulative summing
# interpolate the x- and y-coordinates independently with respect to the new coords
def interpolate(P, step, show_interplot=False):
    x, y, z = P.T
    xd = np.diff(x)
    yd = np.diff(y)
    zd = np.diff(z)
    dist = np.sqrt(xd**2 + yd**2 + zd**2)
    u = np.cumsum(dist)
    u = np.hstack([[0], u])

    # t = np.linspace(0, u.max(), size) #interpolate using # points (80)
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

# def find_intersection(path1, path2, show_intersect=True):

