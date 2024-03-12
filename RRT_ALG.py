#rrt alg maybe

import matplotlib.pyplot as plt
import numpy as np
import random
import time

from math               import pi, sin, cos, atan2, sqrt, ceil
from scipy.spatial      import KDTree
from shapely.geometry   import Point, LineString, Polygon, MultiPolygon
from shapely.prepared   import prep


######################################################################
#
#   Parameters
#
#   Define the step size.  Also set the maximum number of nodes.
#
DSTEP = .5

# Maximum number of nodes.
NMAX = 1500
######################################################################
#
#   World Definitions
#
#   List of obstacles/objects as well as the start/goal.
#
(xmin, xmax) = (0, 20)
(ymin, ymax) = (0, 20)

#how do we want to define walls
walls = ()

# Define the start/goal states (x, y, theta)
(xstart, ystart) = (13, 5)
(xgoal,  ygoal)  = ( 1, 5)

######################################################################
#   Node Definition
class Node:
    def __init__(self, x, y):
        # Define a parent (cleared for now).
        self.parent = None

        # Define/remember the state/coordinates (x,y).
        self.x = x
        self.y = y

    ############
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return ("<Point %5.2f,%5.2f>" % (self.x, self.y))

    # Compute/create an intermediate node.  This can be useful if you
    # need to check the local planner by testing intermediate nodes.
    def intermediate(self, other, alpha):
        return Node(self.x + alpha * (other.x - self.x),
                    self.y + alpha * (other.y - self.y))

    # Return a tuple of coordinates, used to compute Euclidean distance.
    def coordinates(self):
        return (self.x, self.y)

    # Compute the relative distance to another node.
    def distance(self, other):
        return sqrt((self.x - other.x)**2 + (self.y - other.y)**2)

    ################
    # Collision functions:
    # Check whether in free space.
    def inFreespace(self):
        if (self.x <= xmin or self.x >= xmax or
            self.y <= ymin or self.y >= ymax):
            return False
        return walls.disjoint(Point(self.coordinates()))

    # Check the local planner - whether this connects to another node.
    def connectsTo(self, other):
        line = LineString([self.coordinates(), other.coordinates()])
        return walls.disjoint(line)


######################################################################
#
#   EST Functions
#
def est(startnode, goalnode, visual):
    # Start the tree with the startnode (set no parent just in case).
    startnode.parent = None
    tree = [startnode]

    # Function to attach a new node to an existing node: attach the
    # parent, add to the tree, and show in the figure.
    def addtotree(oldnode, newnode):
        newnode.parent = oldnode
        tree.append(newnode)
        visual.drawEdge(oldnode, newnode, color='g', linewidth=1)
        visual.show()

    def distance_ghost_1(self, other):
        return sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def distance_ghost_2(self, other):
        return sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    # Loop - keep growing the tree.
    while True:
        # Determine the local density by the number of nodes nearby.
        # KDTree uses the coordinates to compute the Euclidean distance.
        # It returns a NumPy array, same length as nodes in the tree.
        X = np.array([node.coordinates() for node in tree])
        kdtree  = KDTree(X)
        numnear = kdtree.query_ball_point(X, r=1.5*DSTEP, return_length=True)

        # Directly determine the distances to the goal node.
        distances = np.array([node.distance(goalnode) for node in tree])

        # Select the node from which to grow, which minimizes some metric.
        scale = 5
        grownode = tree[np.argmin(numnear)]
    
        xknown = ()  #list of all known x and y coord in view of lidar
        yknown = ()
    
        def distance_goal(known, goal):
            return sqrt((xknown - goal.x)**2 + (yknown - goal.y)**2)
    
        while True:
            failed_connections = 0
        
        # Determine the target state.
        
            if distance_ghost_1 >= distance_ghost_2:
            #tell ghost cost of turning or going that direction... maybe with a theta idk
                turn_factor = ()
            
            else:
            #tell ghost cost of turning other direction high
                turn_factor = ()

        #lowkey this target node more so based of of est_triangles from hw 4.. maybe do est?    
            
            min_list = distance_goal + failed_connections + turn_factor
            targetnode = np.argmin(min_list)    
        
            
        # Directly determine the distances to the target node.
            distances = np.array([node.distance(targetnode) for node in tree])
            index     = np.argmin(distances)
            nearnode  = tree[index]
            d         = distances[index]

        # Determine the next node.
            heading = atan2(targetnode.y - nearnode.y, targetnode.x - nearnode.x)
            stepsize = DSTEP
            if nearnode.distance(targetnode) < DSTEP: stepsize = nearnode.distance(targetnode)
            nextnode_coords = (nearnode.x + stepsize * cos(heading), nearnode.y + stepsize * sin(heading))
            nextnode = Node(x=nextnode_coords[0], y=nextnode_coords[1])


            if nextnode.inFreespace() and grownode.connectsTo(nextnode):
                addtotree(grownode, nextnode)
                break
            
        # Once grown, also check whether to connect to goal.
        if nextnode.distance(goalnode) <= DSTEP:
            if nextnode.connectsTo(goalnode): 
                addtotree(nextnode, goalnode)
                break
        else:
            failed_connections += 1              
            
        


    




