# COLLISION AVOIDANCE MECHANISM
import numpy as np
import matplotlib.pyplot as plt
import time
from mpl_toolkits import mplot3d
from enum import Enum
import logging
import pickle
from scipy.interpolate import interp1d

from shapely.geometry import LineString
from RRTStar import RRTStar
from arm import Arm
from objects import Object

def main():
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlabel('X, [m]')
    ax.set_ylabel('Y, [m]')
    ax.set_zlabel('Z, [m]')
    ax.set_ylim([0,12])
    ax.set_xlim([0,12])

    l1 = np.array([[0,0,0], [10,8,8]])
    l2 = np.array([[2,0,3], [6,8,8], [8,4,4]])

    plt.plot(l1[:,0], l1[:,1], l1[:,2], 'o', color='orange')
    plt.plot(l2[:,0], l2[:,1], l2[:,2], 'o', color='blue')

    for i in range(l1.shape[0]-1):
        ax.plot([l1[i,0], l1[i+1,0]], [l1[i,1], l1[i+1,1]], [l1[i,2], l1[i+1,2]], color = 'orange', linewidth=1, zorder=15)
    for i in range(l2.shape[0]-1):
        ax.plot([l2[i,0], l2[i+1,0]], [l2[i,1], l2[i+1,1]], [l2[i,2], l2[i+1,2]], color = 'blue', linewidth=1, zorder=15)

    # check 2d intersections x-y axis
    line1 = LineString(l1)
    line2 = LineString(l2)
    intersect_pts = line1.intersection(line2)

    pts_list = []
    for point in intersect_pts:
        pts_list.append([point.x, point.y, point.z])

    intersect_list = np.array(pts_list)
    print(intersect_list)
    
    # check 2d intersections x-z axis
    plt.plot(intersect_list[:,0], intersect_list[:,1], intersect_list[:,2], 'ro', color='cyan')
    
    # check 2d intersections y-z axis
    
    
    plt.show()

def find_intersection(path1, path2):
    idx = np.argwhere(np.diff(np.sign(path1 - path2))).flatten()
    plt.plot(path1[idx], path2[idx], 'ro')
    plt.show

if __name__ == '__main__':
    main()