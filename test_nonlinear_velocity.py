import logging
import numpy as np
import matplotlib.pyplot as plt
import math

from scipy.optimize import fsolve

logger = logging.getLogger(__name__)
logging.basicConfig()
logger.setLevel(logging.INFO)

STEP_SIZE = 0.08 #controls speed of paths
COLLISION_RANGE = 1

def main():

    animate = False

    fig = plt.figure()
    ax = plt.axes(projection='3d')
    ax.set_xlabel('X, [m]')
    ax.set_ylabel('Y, [m]')
    ax.set_zlabel('Z, [m]')
    ax.set_xlim([0,5])
    ax.set_ylim([0,5])
    ax.set_zlim([0,5])

    # l1 = np.array([[0.0,0.0,0.0], [10.0,8.0,8.0]])
    # l2 = np.array([[2.0,0.0,3.0], [6.0,8.0,8.0], [10.0,2.0,2.0]])
    # l1 = np.array([[0.0,0.0,0.0], [10.0,8.0,8.0]])
    l1 = np.array([[0.0,0.0,0.0], [3.0,2.0,2.0]])
    # l2 = np.array([[ 2.0,0.0,3.0], [6.0,8.0,8.0], [10.0,8.0,8.0]])

    # initialize arms
    # start at 1 pts of each lines, end at last pt of each line
    # arm1 = Arm(name="PSM1", position=l1[0], destination=l1[l1.shape[0]-1], velocity=STEP_SIZE, home=l1[0])
    # arm2 = Arm(name="PSM2", position=l2[0], destination=l2[l2.shape[0]-1], velocity=STEP_SIZE, home=l2[0])

    plt.plot(l1[:,0], l1[:,1], l1[:,2], 'o', color='orange')
    # plt.plot(l2[:,0], l2[:,1], l2[:,2], 'o', color='blue')

     # draw planned paths
    for i in range(l1.shape[0]-1):
        ax.plot([l1[i,0], l1[i+1,0]], [l1[i,1], l1[i+1,1]], [l1[i,2], l1[i+1,2]], color = 'orange', linewidth=1, zorder=15)
    # for i in range(l2.shape[0]-1):
    #     ax.plot([l2[i,0], l2[i+1,0]], [l2[i,1], l2[i+1,1]], [l2[i,2], l2[i+1,2]], color = 'blue', linewidth=1, zorder=15)


    # initialize paths
    path1 = interpolate(l1, STEP_SIZE)
    # path2 = interpolate(l2, STEP_SIZE)

    # arm1.set_position(path1[0])    # start at pt. 0 of path1
    # arm2.set_position(path2[0])    # start at pt. 60 of path2

    # now that we have the intersection zones in both paths, adjust speed and animate
    # intersect1, intersect2 = find_intersection(path1, path2, arm1, arm2)  
    # intersect_pts1, intersect_pts2 = np.array(intersect1), np.array(intersect2)
    # new_path1, new_path2 = avoid_collision(intersect_pts1, intersect_pts2, path1, path2, arm1, arm2)
    # new_path1 = interpolate(path1, 0.08)

    run_path(path1)

    # logger.info("INTERSECTIONS: {}".format(intersect1))
    # plt.show()

def euclidean_distance(point1, point2):
    distance = np.linalg.norm(point1-point2)
    return distance

def run_path(path1):
    # check whether any pts in paths are within threshold
    # idx1 = np.where(path1 == arm1.get_position())[0][0]     # get start index
    # idx2 = np.where(path2 == arm2.get_position())[0][0]     # get start index
    
    idx1 = 0
    # idx2 = 0
    # path_range = min(path1[idx1:,:].shape[0], path2[idx2:,:].shape[0])    # get minimum of both remaining paths
    # print("PATH RANGES: {}, {}".format(path1[idx1:,:].shape[0], path2[idx2:,:].shape[0]))

    intersect1 = []
    # intersect2 = []
    i = 0

    while(idx1 != path1.shape[0]-1):
        # idx1 = np.where(path1 == arm1.get_position())[0][0] + i
        # idx2 = np.where(path2 == arm2.get_position())[0][0] + i
        idx1 = i
        # idx2 = i

        plt.plot(path1[idx1,0], path1[idx1,1], path1[idx1,2], 'o', color='red', markersize=1)
        # if idx2 < path2.shape[0]:
        #     plt.plot(path2[idx2,0], path2[idx2,1], path2[idx2,2], 'o', color='red', markersize=1)

        i += 1
        plt.pause(0.0005)

    plt.show()

def my_lin(lb, ub, step, steps, spacing=1.1):
    span = (ub-lb)
    dx = step / (steps-1)

    return [lb + (i*dx)**spacing*span for i in range(steps)]

def quadratic_interpolation(lb, ub, step):
    li = 0 # last interval (min accel)
    
    # first, solve equation system to find tf, a, b, c
    # c = lb
    # a + b + c = step
    # a*tf**2 + b*tf + c = ub
    # 2*a*tf + b = li

    b = 2*ub - np.sqrt((4*ub**2) - (4*ub*step) + (li**2))
    a = step - b
    c = 0
    tf = (li - b)/(2*(step - b))  # tf = total # of samples
    
    arr = []
    for x in range(round(tf)):
        # quadratic equation:
        y = a*x**2 + b*x + c
        arr.append(y)

    print(y)

    return np.array(arr)

def log_interpolation(lb, ub, step):
    li = 0.01 # last interval (min accel)
    
    # first, solve equation system to find a, t, b, c
    # y = logA(t-B) + c
    # y' = 1/(ln(A)*(t-B))
    # choose A by tuning while A>1, start with 1.2 maybe, more gradual decel small A, fast decel big A
    # C=LN(li*LN(A))/LN(A)+YN
    # B = 1 - A^(Y1-C)
    # tf = a^(lp-c)+b
    print("lb={}, ub={}, step={}".format(lb,ub,step))

    a = 2
    c = (math.log(li*math.log(a))/math.log(a)) + ub
    b = 1 - a**(step - c)
    tf = a**(ub-c)+b  # tf = total # of samples

    print("EQN a={}, c={}, b={}, tf={}".format(a,c,b,tf))
    
    arr = []
    for x in range(round(tf)):
        # quadratic equation:
        y = math.log((x-b), a) + c
        # y = 1/(math.log(a) *(x-b))
        arr.append(y)

    print("ARR: {}".format(arr))
    # x = np.linspace(0,5,tf)
    # fig = plt.figure()
    # ax = fig.add_subplot(1, 1, 1)
    # plt.plot(x,y, 'r')
    # plt.show()

    return np.array(arr)

def natural_interpolation(lb, ub, step):
    li = 0.005 # last interval (min accel)
    
    # first, solve equation system to find a, t, b, c
    # 


    print("lb={}, ub={}, step={}".format(lb,ub,step))

    mult = 0.01866  # 1.866 %
    k = ub*(1+mult)
    t = -1/math.log(1-step/k)
    tf = math.log(1-ub/k)*(-t)

    print("EQN mult={}, k={}, t={}, tf={}".format(mult,k,t,tf))
    
    arr = []
    for x in range(round(tf)):
        # natural equation:
        y = k*(1- math.e **((-x)/t))
        arr.append(y)

    print("ARR: {}".format(arr))
    # x = np.linspace(0,5,tf)
    # fig = plt.figure()
    # ax = fig.add_subplot(1, 1, 1)
    # plt.plot(x,y, 'r')
    # plt.show()

    return np.array(arr)

def interpolate(P, step, show_interplot=False):
    x, y, z = P.T
    xd = np.diff(x)
    yd = np.diff(y)
    zd = np.diff(z)
    dist = np.sqrt(xd**2 + yd**2 + zd**2)
    u_ = np.cumsum(dist)
    u = np.hstack([[0], u_])

    # t = np.linspace(0, u.max(), size) #interpolate using # points (80)
    # t = np.arange(0, u.max()+step, step)  # start val, end val, steps     

    t = natural_interpolation(0, u.max()+step, step)
    print(u.max())
    print(t)

    xn = np.interp(t, u, x)     # TODO: make sure fp and xp len matches
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

def test():
    x, y =  fsolve(equations, (5, 5))
    print(equations((x, y)))
    print(x, y)

def equations(p):
    x, y = p
    return (y - x**2 -7 + 5*x, 4*y - 8*x + 21)
    
if __name__ == '__main__':
    main()