# STATE MACHINE FOR 3D PICK AND PLACE SIMULATION
import numpy as np
from numpy.linalg import norm
from math import *
from matplotlib import pyplot as plt
from matplotlib.patches import Polygon
from random import random
from scipy.spatial import ConvexHull
from matplotlib import path
import time
from mpl_toolkits import mplot3d
from enum import Enum
import logging

from utils import init_fonts
from path_shortening import shorten_path
from obstacles import Parallelepiped

### CONSTANTS ###
pause_time = 0.0005

### PARAMETERS ###
show_RRT = False

class Node3D:
    def __init__(self):
        self.p     = [0, 0, 0]
        self.cost     = 0
        self.costPrev = 0

def isCollisionFreeVertex(obstacles, point):
    x,y,z = point
    for obstacle in obstacles:
	    dx, dy, dz = obstacle.dimensions
	    x0, y0, z0 = obstacle.pose
	    if abs(x-x0)<=dx/2 and abs(y-y0)<=dy/2 and abs(z-z0)<=dz/2:
	        return 0
    return 1

def isCollisionFreeEdge(obstacles, closest_vert, p):
    closest_vert = np.array(closest_vert)
    p = np.array(p)
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

def closestNode3D(rrt, p):
    distance = []
    for node in rrt:
        distance.append(sqrt((p[0] - node.p[0])**2 + (p[1] - node.p[1])**2 + (p[2] - node.p[2])**2))
    distance = np.array(distance)
    
    dmin = min(distance)
    ind_min = distance.tolist().index(dmin)
    closest_node = rrt[ind_min]

    return closest_node

def RRTStar(ax, obstacles, starts, goals, log_verbose=False):
    # parameters
    animate = 1

    # RRT Initialization
    maxiters  = 5000
    nearGoal = False # This will be set to true if goal has been reached
    minDistGoal = 0.05 # Convergence criterion: success when the tree reaches within 0.25 in distance from the goal.
    d = 0.1#0.5 # [m], Extension parameter: this controls how far the RRT extends in each step.
    
    start = np.array([0.0, 0.0, 0.0])
    goal =  np.array([0.0, 1.0, 2.5])
    ax.scatter3D(start[0], start[1], start[2], color='green', s=100)
    ax.scatter3D(goal[0], goal[1], goal[2], color='red', s=100)

    # Initialize RRT. The RRT will be represented as a 2 x N list of points.
    # So each column represents a vertex of the tree.
    rrt = []    # list of vertex
    start_node = Node3D()
    start_node.p = start
    start_node.cost = 0
    start_node.costPrev = 0
    rrt.append(start_node)

    ### RRT ALGORITHM ###
    start_time = time.time()
    iters = 0
    while not nearGoal and iters < maxiters:
        # Sample point
        rnd = random()
        # With probability 0.05, sample the goal. This promotes movement to the goal.
        if rnd < 0.10:
            p = goal
        else:
            p = np.array([random()*5-2.5, random()*5-2.5, random()*3]) # Should be a 3 x 1 vector
            
        # Check if sample is collision free
        collFree = isCollisionFreeVertex(obstacles, p)
        # If it's not collision free, continue with loop
        if not collFree:
            iters += 1
            continue

        # If it is collision free, find closest point in existing tree. 
        closest_node = closestNode3D(rrt, p)
        
        # Extend tree towards xy from closest_vert. Use the extension parameter
        # d defined above as your step size. In other words, the Euclidean
        # distance between new_vert and closest_vert should be d.
        new_node = Node3D()
        new_node.p = closest_node.p + d * (p - closest_node.p)
        new_node.cost = len(rrt)
        new_node.costPrev = closest_node.cost

        # draw RRT node tree
        if show_RRT:
            ax.plot([closest_node.p[0], new_node.p[0]], [closest_node.p[1], new_node.p[1]], [closest_node.p[2], new_node.p[2]],color = 'b', zorder=5)
            plt.pause(pause_time)
        
        # Check if new vertice is in collision
        collFree = isCollisionFreeEdge(obstacles, closest_node.p, new_node.p)
        # If it's not collision free, continue with loop
        if not collFree:
            iters += 1
            continue
        
        # If it is collision free, add it to tree    
        rrt.append(new_node)

        # Check if we have reached the goal
        if norm(np.array(goal) - np.array(new_node.p)) < minDistGoal:
            # Add last, goal node
            goal_node = Node3D()
            goal_node.p = goal
            goal_node.cost = len(rrt)
            goal_node.costPrev = new_node.cost
            if isCollisionFreeEdge(obstacles, new_node.p, goal_node.p):
                rrt.append(goal_node)
                path = [goal_node.p]
            else: path = []
            end_time = time.time()
            nearGoal = True

            if log_verbose:
                logging.debug('Reached the goal after %.2f seconds:' % (end_time - start_time))

        iters += 1

    if log_verbose:
        logging.debug('Number of iterations passed: %d / %d' %(iters, maxiters))
        logginb.debug('RRT length: ', len(rrt))

    # Path construction from RRT:
    logging.info('Constructing the path...')
    i = len(rrt) - 1
    while True:
        i = rrt[i].costPrev
        path.append(rrt[i].p)
        if i == 0:
            logging.info(print('Reached RRT start node'))
            break
    path = np.array(path)

    # Drawing unoptimized RRT path
    if show_RRT:
        for i in range(path.shape[0]-1):
            ax.plot([path[i,0], path[i+1,0]], [path[i,1], path[i+1,1]], [path[i,2], path[i+1,2]], color = 'g', linewidth=3, zorder=10)
            plt.pause(pause_time)

    ### DRAW SHORTENED PATH ###
    logging.info('Shortening the path...')
    path = shorten_path(path, obstacles, size=80, smoothiters=100)
    path = np.flip(path, axis=0)
    for i in range(path.shape[0]-1):
        ax.plot([path[i,0], path[i+1,0]], [path[i,1], path[i+1,1]], [path[i,2], path[i+1,2]], color = 'orange', linewidth=3, zorder=15)
        plt.pause(pause_time)

    plt.show()