import numpy as np
from buildmap import WIDTH, HEIGHT, MAZE_SIZE, RESOLUTION
from math     import pi, sin, cos, atan2, sqrt, ceil
from utils    import grid_to_pixel, pixel_to_grid
from constants import SCAN_RESOLUTION

######################################################################
#
#   Node Definition
#
class Node:
    def __init__(self, x, y, env, learn = False, pacman = True, fuck_factor = 0):
        # Define a parent (cleared for now).
        self.parent = None

        # Define/remember the state/coordinates (x,y).
        self.x = x
        self.y = y

        self.env = env
        self.learn = learn
        self.pacman = pacman

        if learn:
            if pacman:
                self.walls = env.pacman_learned_walls
            else:
                self.walls = env.ghosts_learned_walls
        else:
            self.walls = env.walls
        
        self.fuck_factor = 0
        self.depth = 0

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
                    self.env,
                    self.learn,
                    self.pacman)

    # Return a tuple of coordinates, used to compute Euclidean distance.
    def coordinates(self):
        return (self.x, self.y)

    # Compute the relative distance to another node.
    def distance(self, other):
        return sqrt((self.x - other.x)**2 + (self.y - other.y)**2) + self.fuck_factor

    ################
    # Collision functions:
    # Check whether in free space.
    def inFreespace(self):
        if (self.x <= 0 or self.x >= WIDTH or
            self.y <= 0 or self.y >= HEIGHT):
            return False
        
        for dx in [-2, 0, 2]:
            for dy in [-2, 0, 2]:
                grid_point = pixel_to_grid((self.x + dx, self.y + dy))
                try:
                    if (self.walls[round(grid_point[0]), round(grid_point[1])] == 1):
                        return False
                except IndexError:
                    pass
        return True

    # Check the local planner - whether this connects to another node.
    def connectsTo(self, other):
        for i in range(SCAN_RESOLUTION * 3):
            if not (self.intermediate(other, i/(SCAN_RESOLUTION * 3)).inFreespace()):
                return False
        return True

