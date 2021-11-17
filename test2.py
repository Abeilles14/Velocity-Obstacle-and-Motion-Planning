# COLLISION AVOIDANCE MECHANISM
import numpy as np
import matplotlib.pyplot as plt
import time
from mpl_toolkits import mplot3d
from enum import Enum
import logging
import pickle
from scipy.interpolate import interp1d
from ground.base import get_context
from bentley_ottmann.planar import segments_intersect
from Geometry3D import *

from shapely.geometry import LineString, MultiPoint, shape
from RRTStar import RRTStar
from arm import Arm
from objects import Object
from velocity_control import interpolate, update_velocity
from arm import Arm

logger = logging.getLogger(__name__)
logging.basicConfig()
logger.setLevel(logging.INFO)

def find_intersections():
    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlabel('X, [m]')
    ax.set_ylabel('Y, [m]')
    ax.set_zlabel('Z, [m]')
    ax.set_xlim([0,12])
    ax.set_ylim([0,12])
    ax.set_zlim([0,12])

    l1 = np.array([[0.0,0.0,0.0], [10.0,8.0,8.0]])
    l2 = np.array([[2.0,0.0,3.0], [6.0,8.0,8.0], [8.0,4.0,4.0]])

    plt.plot(l1[:,0], l1[:,1], l1[:,2], 'o', color='orange')
    plt.plot(l2[:,0], l2[:,1], l2[:,2], 'o', color='blue')

    for i in range(l1.shape[0]-1):
        ax.plot([l1[i,0], l1[i+1,0]], [l1[i,1], l1[i+1,1]], [l1[i,2], l1[i+1,2]], color = 'orange', linewidth=1, zorder=15)
    for i in range(l2.shape[0]-1):
        ax.plot([l2[i,0], l2[i+1,0]], [l2[i,1], l2[i+1,1]], [l2[i,2], l2[i+1,2]], color = 'blue', linewidth=1, zorder=15)

    # check 2d intersections x-y axis
    l1_xy = LineString(l1[:,:2])
    l2_xy = LineString(l2[:,:2])
    intersect_xy = l1_xy.intersection(l2_xy)
    print("GEOM TYPE: {}".format(intersect_xy.geom_type))

    intersect_list = []
    if intersect_xy:       # check if any intersections
        if intersect_xy.geom_type == "MultiPoint":  # multiple intersections
            for point in intersect_xy.geoms:
                intersect_list.append([point.x, point.y])
        elif intersect_xy.geom_type == "Point":  # single intersection
            intersect_list.append([intersect_xy.x, intersect_xy.y])
        elif intersect_xy.geom_type == "LineString":   # infinite intersection
            intersect_list = np.asarray(intersect_xy)
    intersections_xy = np.array(intersect_list)
    print("INTERSECTIONS: {}".format(intersections_xy))
    plt.plot(intersections_xy[:,0], intersections_xy[:,1], 0, 'o', color='cyan')

    # check 2d intersections x-z axis
    l1_xz = LineString(l1[:,0:3:2])
    l2_xz = LineString(l2[:,0:3:2])
    intersect_xz = l1_xz.intersection(l2_xz)
    print("GEOM TYPE: {}".format(intersect_xz.geom_type))

    intersect_list = []
    if intersect_xz:       # check if any intersections
        if intersect_xz.geom_type == "MultiPoint":  # multiple intersections
            for point in intersect_xz.geoms:
                intersect_list.append([point.x, point.y])
        elif intersect_xz.geom_type == "Point":  # single intersection
            intersect_list.append([intersect_xz.x, intersect_xz.y])
        elif intersect_xz.geom_type == "LineString":   # infinite intersection
            intersect_list = np.asarray(intersect_xz)
    intersections_xz = np.array(intersect_list)
    print("INTERSECTIONS: {}".format(intersections_xz))
    plt.plot(intersections_xz[:,0], 0, intersections_xz[:,1], 'o', color='cyan')

    # check 2d intersections y-z axis
    l1_yz = LineString(l1[:,1:])
    l2_yz = LineString(l2[:,1:])
    intersect_yz = l1_yz.intersection(l2_yz)
    print("GEOM TYPE: {}".format(intersect_yz.geom_type))

    intersect_list = []
    if intersect_yz:       # check if any intersections
        if intersect_yz.geom_type == "MultiPoint":  # multiple intersections
            for point in intersect_yz:
                intersect_list.append([point.x, point.y])
        elif intersect_yz.geom_type == "Point":  # single intersection
            intersect_list.append([intersect_yz.x, intersect_yz.y])
        elif intersect_yz.geom_type == "LineString":   # infinite intersection
            intersect_list = np.asarray(intersect_yz.coords)
    intersections_yz = np.array(intersect_list)
    print("INTERSECTIONS: {}".format(intersections_yz))
    for i in range(intersections_yz.shape[0]-1):
        ax.plot([0,0], [intersections_yz[i,0], intersections_yz[i+1,0]], [intersections_yz[i,1], intersections_yz[i+1,1]], color = 'cyan', linewidth=1, zorder=15)
    



    # plt.plot(intersect_list[:,0], , intersect_list[:,2], 'ro', color='cyan')
    plt.show()

def bentley_ottmann_alg():
    context = get_context()
    Point, Segment = context.point_cls, context.segment_cls
    unit_segments = [Segment(Point(0, 0), Point(1, 0)), Segment(Point(0, 0), Point(0, 1))]
    print(segments_intersect(unit_segments))


def geometry3d():
    a = Point(1,1,1)
    b = Point(-1,1,1)
    c = Point(-1,-1,1)
    d = Point(1,-1,1)
    e = Point(1,1,-1)
    f = Point(-1,1,-1)
    g = Point(-1,-1,-1)
    h = Point(1,-1,-1)
    cph0 = Parallelepiped(Point(-1,-1,-1),Vector(2,0,0),Vector(0,2,0),Vector(0,0,2))
    cpg12 = ConvexPolygon((e,c,h))
    cpg13 = ConvexPolygon((e,f,c))
    cpg14 = ConvexPolygon((c,f,g))
    cpg15 = ConvexPolygon((h,c,g))
    cpg16 = ConvexPolygon((h,g,f,e))
    cph1 = ConvexPolyhedron((cpg12,cpg13,cpg14,cpg15,cpg16))
    a1 = Point(1.5,1.5,1.5)
    b1 = Point(-0.5,1.5,1.5)
    c1 = Point(-0.5,-0.5,1.5)
    d1 = Point(1.5,-0.5,1.5)
    e1 = Point(1.5,1.5,-0.5)
    f1 = Point(-0.2,1.5,-0.5)
    g1 = Point(-0.2,-0.5,-0.5)
    h1 = Point(1.5,-0.5,-0.5)
    
    plane = Plane(Point(4,4), Point(8,8), Point(0.0))
    
    cpg6 = ConvexPolygon((a1,d1,h1,e1))
    cpg7 = ConvexPolygon((a1,e1,f1,b1))
    cpg8 = ConvexPolygon((c1,b1,f1,g1))
    cpg9 = ConvexPolygon((c1,g1,h1,d1))
    cpg10 = ConvexPolygon((a1,b1,c1,d1))
    cpg11 = ConvexPolygon((e1,h1,g1,f1))
    cph2 = ConvexPolyhedron((cpg6,cpg7,cpg8,cpg9,cpg10,cpg11))
    cph3 = intersection(cph0,cph2)
    cph4 = intersection(cph1,cph2)
    # pts_inter = intersection()
    r = Renderer()
    # r.add((plane,'r',1),normal_length = 0)
    # r.add((cph0,'r',1),normal_length = 0)
    # r.add((cph1,'r',1),normal_length = 0)
    # r.add((cph2,'g',1),normal_length = 0)
    # r.add((cph3,'b',3),normal_length = 0.5)
    # r.add((cph4,'y',3),normal_length = 0.5)
    r.show()

def geometry3D_test():
    l1p1 = Point(0.0,0.0,0.0)
    l1p2 = Point(10.0,8.0,8.0)
    l1p3 = Point(12.0,8.0,8.0)

    l2p1 = Point(2.0,0.0,3.0)
    l2p2 = Point(6.0,8.0,8.0)
    l2p3 = Point(8.0,4.0,4.0)

    l1 = ConvexPolygon((l1p1,l1p2,l1p3))
    l2 = ConvexPolygon((l2p1,l2p2, l2p3))
    poly_inter = intersection(l1,l2)
    print(poly_inter)
    r = Renderer()
    r.add((l1,'r',1),normal_length = 0)
    r.add((l2,'g',1),normal_length = 0)
    r.add((poly_inter,'b',1),normal_length = 0)
    r.show()

STEP_SIZE = 0.08 #controls speed of paths
THRESHOLD_DIST = 1

def main():

    animate = False

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlabel('X, [m]')
    ax.set_ylabel('Y, [m]')
    ax.set_zlabel('Z, [m]')
    ax.set_xlim([0,12])
    ax.set_ylim([0,12])
    ax.set_zlim([0,12])

    # l1 = np.array([[0.0,0.0,0.0], [10.0,8.0,8.0]])
    # l2 = np.array([[2.0,0.0,3.0], [6.0,8.0,8.0], [10.0,2.0,2.0]])
    l1 = np.array([[0.0,0.0,0.0], [10.0,8.0,8.0]])
    l2 = np.array([[2.0,0.0,3.0], [6.0,8.0,8.0], [10.0,2.0,2.0]])

    # initialize arms
    # start at 1 pts of each lines, end at last pt of each line
    arm1 = Arm(name="PSM1", position=l1[0], destination=l1[l1.shape[0]-1], velocity=STEP_SIZE, home=l1[0])
    arm2 = Arm(name="PSM2", position=l2[0], destination=l2[l2.shape[0]-1], velocity=STEP_SIZE, home=l2[0])

    plt.plot(l1[:,0], l1[:,1], l1[:,2], 'o', color='orange')
    plt.plot(l2[:,0], l2[:,1], l2[:,2], 'o', color='blue')

    # initialize paths
    path1 = interpolate(l1, STEP_SIZE)
    path2 = interpolate(l2, STEP_SIZE)

    for i in range(l1.shape[0]-1):
        ax.plot([l1[i,0], l1[i+1,0]], [l1[i,1], l1[i+1,1]], [l1[i,2], l1[i+1,2]], color = 'orange', linewidth=1, zorder=15)
    for i in range(l2.shape[0]-1):
        ax.plot([l2[i,0], l2[i+1,0]], [l2[i,1], l2[i+1,1]], [l2[i,2], l2[i+1,2]], color = 'blue', linewidth=1, zorder=15)


    arm1.set_position(path1[0])    # start at pt. 0 of path1
    arm2.set_position(path2[50])    # start at pt. 60 of path2

    # check whether any pts in paths are within threshold
    # get start index for path change (current arm pos)
    idx1 = np.where(path1 == arm1.get_position())[0][0]     # get start index
    idx2 = np.where(path2 == arm2.get_position())[0][0]     # get start index
    path_range = min(path1[idx1:,:].shape[0], path2[idx2:,:].shape[0])    # get minimum of both remaining paths
    logger.info("PATH RANGES: {}, {}".format(path1[idx1:,:].shape[0], path2[idx2:,:].shape[0]))

    intersect1 = []
    intersect2 = []
    for i in range(path_range-1):
        idx1 = np.where(path1 == arm1.get_position())[0][0] + i
        idx2 = np.where(path2 == arm2.get_position())[0][0] + i
        # print("IDX: {}, {}".format(idx1, idx2))
        arm_dist = euclidean_distance(path1[idx1], path2[idx2])

        logger.debug("POS: {}, {}".format(path1[idx1], path2[idx2]))
        logger.debug("ARM_DIST: {}".format(arm_dist))
        # plt.plot(path1[idx1,0], path1[idx1,1], path1[idx1,2], 'o', color='red', markersize=1)
        # plt.plot(path2[idx2,0], path2[idx2,1], path2[idx2,2], 'o', color='red', markersize=1)
        
        if (arm_dist <= THRESHOLD_DIST).all():
            logger.debug("COLLISION IMMINENT!")
            intersect1.append([path1[idx1,0], path1[idx1,1], path1[idx1,2]])
            intersect2.append([path1[idx1,0], path1[idx1,1], path1[idx1,2]])
            plt.plot(path1[idx1,0], path1[idx1,1], path1[idx1,2], 'o', color='cyan')
            plt.plot(path2[idx2,0], path2[idx2,1], path2[idx2,2], 'o', color='cyan')

        # plt.pause(0.0005)
    
    # now that we have the intersection zones in both paths, adjust speed and animate  
    intersect_pts1, intersect_pts2 = np.array(intersect1), np.array(intersect2)
    new_path1, new_path2 = avoid_collision(intersect_pts1, intersect_pts2, path1, path2, arm1, arm2)
    run_path(new_path1, new_path2, arm1, arm2)

    logger.info("INTERSECTIONS: {}".format(intersect1))
    # plt.show()

def euclidean_distance(point1, point2):
    distance = np.linalg.norm(point1-point2)
    return distance

def avoid_collision(intersect_pts1, intersect_pts2, path1, path2, arm1, arm2):
    # get start index for path change (current arm pos)
    idx1 = np.where(path1 == arm1.get_position())[0][0]     # get start index
    idx2 = np.where(path2 == arm2.get_position())[0][0]     # get start index

    # if collision detected, adjust path velocities
    if (intersect_pts1.shape[0] != 0) and (intersect_pts2.shape[0] != 0):
        logger.info("COLLISION DETECTED!")
        logger.debug("INTERSECTIONS: {}, SHAPE: {}".format(intersect_pts1, intersect_pts1.shape[0]))
        # temp pre-set velocities:
        # NOTE: start point of new paths is arm current location!! need to iter from 0 when plotting
        new_path1, new_path2 = update_velocity(path1[idx1:,:], path2[idx2:,:], vel1=0.07, vel2=0.09)
        # set new path and last collision point
        logger.info("UPDATED VELOCITY FOR COLLISION AVOIDANCE")
        print("Arm1: {}, Arm2: {}".format(0.07, 0.09))
    else:
        # reset paths velocities if no more intersections
        logger.info("NO COLLISION DETECTED!")
        new_path1, new_path2 = update_velocity(path1[idx1:,:], path2[idx2:,:], vel1=STEP_SIZE, vel2=STEP_SIZE)
        print("Arm1: {}, Arm2: {}".format(STEP_SIZE, STEP_SIZE))
        # arm1_sm.set_path(new_path1, np.empty(3))
        # arm2_sm.set_path(new_path2, np.empty(3))

    return new_path1, new_path2

def run_path(path1, path2, arm1, arm2):
    # check whether any pts in paths are within threshold
    # idx1 = np.where(path1 == arm1.get_position())[0][0]     # get start index
    # idx2 = np.where(path2 == arm2.get_position())[0][0]     # get start index
    
    idx1 = 0
    idx2 = 0
    # path_range = min(path1[idx1:,:].shape[0], path2[idx2:,:].shape[0])    # get minimum of both remaining paths
    # print("PATH RANGES: {}, {}".format(path1[idx1:,:].shape[0], path2[idx2:,:].shape[0]))

    intersect1 = []
    intersect2 = []
    i = 0
    while(idx1 != path1[idx1:,:].shape[0] or idx2 != path2[idx2:,:].shape[0]):
        # idx1 = np.where(path1 == arm1.get_position())[0][0] + i
        # idx2 = np.where(path2 == arm2.get_position())[0][0] + i
        idx1 = i
        idx2 = i

        if idx1 < path1.shape[0]:
            plt.plot(path1[idx1,0], path1[idx1,1], path1[idx1,2], 'o', color='red', markersize=1)
        if idx2 < path2.shape[0]:
            plt.plot(path2[idx2,0], path2[idx2,1], path2[idx2,2], 'o', color='red', markersize=1)
            
        i += 1
        plt.pause(0.0005)

    plt.show()

if __name__ == '__main__':
    main()