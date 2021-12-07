# COLLISION AVOIDANCE MECHANISM
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

from arm import Arm
from velocity_control import *
from obstacles import Temp_Obstacle
from arm import Arm
from mpl_toolkits.mplot3d import Axes3D

import matplotlib.pyplot as plt

logger = logging.getLogger(__name__)
logging.basicConfig()
logger.setLevel(logging.INFO)


INIT_VEL = 0.05 #controls speed of paths
INC_VEL = 0.06
COLLISION_RANGE = 1

def main():

    # fig = plt.figure()
    # ax = fig.gca(projection='3d')
    # ax.set_xlabel('X')
    # ax.set_xlim3d(-1, 1)
    # ax.set_ylabel('Y')
    # ax.set_ylim3d(-1, 1)
    # ax.set_zlabel('Z')
    # ax.set_zlim3d(0, 1)

    # height = 2.0
    # nphi, nz = 70, 2
    # pos = np.array([0, 0, 0])

    # r = 1  # radius of cylinder
    # phi = np.linspace(0, 360, nphi) / 180.0 * np.pi
    # z = np.linspace(0, height/2, nz)     # height

    # PHI, Z = np.meshgrid(phi, z)
    # CP = r * np.cos(PHI)
    # SP = r * np.sin(PHI)
    # XYZ = np.dstack([CP, SP, Z])
    # verts = np.stack(
    #     [XYZ[:-1, :-1], XYZ[:-1, 1:], XYZ[1:, 1:], XYZ[1:, :-1]], axis=-2).reshape(-1, 4, 3)



    # cmap = plt.cm.rainbow
    # cols = cmap(np.random.random())

    # poly3 = Poly3DCollection(verts, facecolor=cols, edgecolor="none")
    # poly3.set_alpha(0.8)
    # ax.add_collection3d(poly3)

    
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.set_aspect("auto")

    # draw sphere
    u, v = np.mgrid[0: 2*np.pi: 20j, 0: np.pi: 10j]
    x = np.cos(u)*np.sin(v)
    y = np.sin(u)*np.sin(v)
    z = np.cos(v)

    print(x)
    ax.plot_wireframe(x, y, z, color="r")
    # ax.scatter(x,y,z,"o")

    plt.show()

    # poly3 = Poly3DCollection(verts, facecolor='blue', edgecolor="none")
    # ax.add_collection3d(poly3)
    


if __name__ == '__main__':
    main()