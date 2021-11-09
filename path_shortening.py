#!/usr/bin/env python
import numpy as np
from numpy.linalg import norm
from matplotlib import pyplot as plt
from random import random
import logging

logger = logging.getLogger(__name__)
logging.basicConfig()
logger.setLevel(logging.DEBUG)

def isCollisionFreeVertex(obstacles, point):
    x,y,z = point
    for obstacle in obstacles:
        dx, dy, dz = obstacle.dimensions
        x0, y0, z0 = obstacle.pose
        if abs(x-x0)<=dx/2 and abs(y-y0)<=dy/2 and abs(z-z0)<=dz/2:
            return 0
    return 1

def isCollisionFreeEdge(obstacles, closest_vert, p):
    closest_vert = np.array(closest_vert); p = np.array(p)
    collFree = True
    l = norm(closest_vert - p)
    map_resolution = 0.01; M = int(l / map_resolution)
    if M <= 2: M = 20
    t = np.linspace(0,1,M)
    for i in range(1,M-1):
        point = (1-t[i])*closest_vert + t[i]*p # calculate configuration
        collFree = isCollisionFreeVertex(obstacles, point) 
        if collFree == False: return False

    return collFree

def shorten_path(P, obstacles, smoothiters=30, log_verbose=False):
    # INPUTS
    #   P - path to get smoothed (after RRT algorithm)
    #   obstacles - says where the obstacles are
    #   smoothiters - maximum number of smoothing iterations
    #
    # OUTPUTS
    #   P_smoothed - a path, same format as before:  
    #    P_smoothed = [q1 q2 q3 ... qM]
    #               where q1=qstart and qM=qgoal; in other words, the sequence
    #               of straight-line paths from q1 to q2, q2 to q3, etc., takes
    #               the robot from start to goal without collision
    m = P.shape[0]
    l = np.zeros(m)
    for k in range(1, m):
        l[k] = norm(P[k,:]-P[k-1,:]) + l[k-1] # find all of the straight-line distances
    iters = 0
    while iters < smoothiters:
        s1 = random()*l[m-1] 
        s2 = random()*l[m-1]
        if s2 < s1:
            temps = s1
            s1 = s2
            s2 = temps
        for k in range(1, m):
            if s1 < l[k]:
                i = k - 1
                break
        for k in range(i, m):
            if s2 < l[k]:
                j = k - 1
                break
        if (j <= i):
            iters = iters + 1
            continue
        t1 = (s1 - l[i]) / (l[i+1]-l[i])
        gamma1 = (1 - t1)*P[i,:] + t1*P[i+1,:]
        t2 = (s2 - l[j]) / (l[j+1]-l[j])
        gamma2 = (1 - t2)*P[j,:] + t2*P[j+1,:]
        
        collisionFree = isCollisionFreeEdge(obstacles, gamma1, gamma2)
        if collisionFree == 0:
            iters = iters + 1
            continue
        # print (round(l[i],2), round(s1,2), round(l[i+1],2))
        # plt.plot(P[i,0], P[i,1], 'ro', markersize=10, color='red')
        # plt.plot(gamma1[0], gamma1[1], 'ro', markersize=10, color='green')
        # plt.plot(P[i+1,0], P[i+1,1], 'ro', markersize=10, color='blue')
        # plt.plot(P[j,0], P[j,1], 'ro', markersize=10, color='red')
        # plt.plot(gamma2[0], gamma2[1], 'ro', markersize=10, color='green')
        # plt.plot(P[j+1,0], P[j+1,1], 'ro', markersize=10, color='blue')
        # plt.plot([gamma1[0], gamma2[0]], [gamma1[1], gamma2[1]], color='k', linewidth=5)

        # print (round(l[j],2), round(s2,2), round(l[j+1],2))
        P = np.vstack([P[:(i+1),:], gamma1, gamma2, P[(j+1):m,:]])
        m = P.shape[0]
        l = np.zeros(m)
        for k in range(1, m):
            l[k] = norm( P[k,:] - P[k-1,:] ) + l[k-1]
        iters = iters + 1
#         plt.plot(P[:,0], P[:,1], '--', linewidth=3)
    
    if log_verbose:
        logging.debug("Final Path Length: {}".format(len(P)))
        logging.debug("Final Path: {}".format(P))
    
    return P