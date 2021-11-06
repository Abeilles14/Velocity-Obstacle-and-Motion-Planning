# ENVIRONMENT OBSTACLES
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

# Add Obstacles
class Parallelepiped:
    def __init__(self):
        self.dimensions = [0,0,0]
        self.pose = [0,0,0]
        self.verts = self.vertixes()
        
    def vertixes(self):
        dx = self.dimensions[0]
        dy = self.dimensions[1]
        dz = self.dimensions[2]
        C = np.array(self.pose)

        Z = np.array([[-dx/2, -dy/2, -dz/2],
                      [dx/2, -dy/2, -dz/2 ],
                      [dx/2, dy/2, -dz/2],
                      [-dx/2, dy/2, -dz/2],
                      [-dx/2, -dy/2, dz/2],
                      [dx/2, -dy/2, dz/2 ],
                      [dx/2, dy/2, dz/2],
                      [-dx/2, dy/2, dz/2]])
        Z += C

        # list of sides' polygons of figure
        verts = [ [Z[0], Z[1], Z[2], Z[3]],
                  [Z[4], Z[5], Z[6], Z[7]], 
                  [Z[0], Z[1], Z[5], Z[4]], 
                  [Z[2], Z[3], Z[7], Z[6]], 
                  [Z[1], Z[2], Z[6], Z[5]],
                  [Z[4], Z[7], Z[3], Z[0]] ]

        return verts

    def draw(self, ax):
        ax.add_collection3d(Poly3DCollection(self.vertixes(), facecolors='k', linewidths=1, edgecolors='k', alpha=.25))

