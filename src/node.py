import numpy as np
from buildmap import WIDTH, HEIGHT, MAZE_SIZE, RESOLUTION
from math               import pi, sin, cos, atan2, sqrt, ceil

######################################################################
#
#   Node Definition
#
class Node:
    def __init__(self, x, y, env):
        # Define a parent (cleared for now).
        self.parent = None

        # Define/remember the state/coordinates (x,y).
        self.x = x
        self.y = y

        self.env = env

    ############
    # Utilities:
    # In case we want to print the node.
    def __repr__(self):
        return ("<Point %5.2f,%5.2f>" % (self.x, self.y))

    # Compute/create an intermediate node.  This can be useful if you
    # need to check the local planner by testing intermediate nodes.
    def intermediate(self, other, alpha):
        return Node(self.x + alpha * (other.x - self.x),
                    self.y + alpha * (other.y - self.y),
                    self.env)

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
        if (self.x <= 0 or self.x >= WIDTH or
            self.y <= 0 or self.y >= HEIGHT):
            return False
        grid_point = self.env.pixel_to_grid((self.x, self.y))
        return (self.env.walls[round(grid_point[0]), round(grid_point[1])] == 0)

    # Check the local planner - whether this connects to another node.
    def connectsTo(self, other):
        for i in range(100):
            if not (self.intermediate(other, i/100).inFreespace()):
                return False
        return True

