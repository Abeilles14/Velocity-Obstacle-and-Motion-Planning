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

def main():
    fig = plt.figure(figsize=(10,10))
    ax = plt.axes(projection='3d')
    ax.set_xlabel('X, [m]')
    ax.set_ylabel('Y, [m]')
    ax.set_zlabel('Z, [m]')
    ax.set_xlim([-2.5, 2.5])
    ax.set_ylim([-2.5, 2.5])
    ax.set_zlim([0.0, 3.0])

    x = np.arange(0, 1000)
    f = np.arange(0, 1000)
    g = np.sin(np.arange(0, 10, 0.01) * 2) * 1000

    plt.plot(x, f, '-')
    plt.plot(x, g, '-')

    # find_intersection(f,g)
    idx = np.argwhere(np.diff(np.sign(f - g))).flatten()
    plt.plot(x[idx], f[idx], 'ro')
    plt.show


def find_intersection(path1, path2):
    idx = np.argwhere(np.diff(np.sign(path1 - path2))).flatten()
    plt.plot(path1[idx], path2[idx], 'ro')
    plt.show

if __name__ == '__main__':
    main()
