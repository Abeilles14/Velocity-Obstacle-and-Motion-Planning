# ENVIRONMENT OBSTACLES
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection
import math

class Static_Obstacle:
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
        colors = (135 / 255, 92 / 255, 41 / 255)
        ax.add_collection3d(Poly3DCollection(self.vertixes(), facecolors=colors, linewidths=1, edgecolors='k', alpha=.2))

# class Temp_Obstacle:
#     def __init__(self, rad, pos, stack, sector):
#         self.radius = rad
#         self.position = pos
#         self.stackcount = stack
#         self.sectorcount = sector

#     def vertixes(self):
#         verts = []
#         edges = []
#         sectorstep = 2 * math.pi / self.sectorcount
#         stackstep = math.pi / self.stackcount
#         for num in range(self.stackcount+1):
#             stackangle = math.pi / 2 - num * stackstep
#             for num2 in range(self.sectorcount+1):
#                 sectorangle = num2 * sectorstep
#                 x = self.radius * math.cos(stackangle) * math.cos(sectorangle)
#                 y = self.radius * math.cos(stackangle) * math.sin(sectorangle)
#                 z = self.radius * math.sin(stackangle)
#                 verts.append([x,y,z])

#         for num in range(self.stackcount):
#             cstack = num * (self.sectorcount + 1)
#             nstack = cstack + self.sectorcount + 1
#             for num2 in range(self.sectorcount):
#                 if num != 0:
#                     edges.append([cstack, nstack, cstack + 1])
#                 if num != self.stackcount - 1:
#                     edges.append([cstack + 1, nstack, nstack + 1])

#                 cstack += 1
#                 nstack += 1

#         return verts,edges
    
#     def draw(self, ax):
#         colors = (135 / 255, 92 / 255, 41 / 255)
#         ax.add_collection3d(Poly3DCollection(self.vertixes(), facecolors=colors, linewidths=1, edgecolors='k', alpha=.2))
