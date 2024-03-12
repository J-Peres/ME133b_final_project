import numpy as np
from env import Environment
from node import Node
from scipy.spatial      import KDTree
from math               import pi, sin, cos, atan2, sqrt, ceil
import random
from constants import C_GOAL, C_GHOST, C_LOR, WIDTH, HEIGHT, COLORS
from utils import euclidean
from constants import SCAN_RESOLUTION

class Ghost:
    def __init__(self, pos: tuple[int, int], goal: tuple[int, int], env: Environment, speed: int = .1):
        """Initializes the ghost object.

        Args:
            pos (tuple[int, int]): initial position of the pacman
            speed (int, optional): speed of the pacman. Defaults to 1.
        """

        self.pos = pos
        self.speed = speed
        self.direction = None
        self.env = env

        self.start = pos
        self.goal = goal

        self.index = 0

        self.intermediate_points = SCAN_RESOLUTION
    
    def update_pos(self):
        try:
            start = Node(self.pos[0], self.pos[1], self.env, True)
            end = Node(self.est_path[self.index + 1][0], self.est_path[self.index + 1][1], self.env, True)
        except IndexError:
            self.index = 0
            self.env.generate_other_map_img(display=True)
            return

        # if we can no longer go this way, restart
        if not start.connectsTo(end):
            self.est()
            self.index = 0
            return

        curr_path = np.array(end.coordinates()) - np.array(start.coordinates())
        if np.linalg.norm(curr_path) > self.speed:
            new_pos = tuple(self.speed * curr_path / np.linalg.norm(curr_path) + np.array(self.pos))
        else:
            new_pos = end.coordinates()
            self.index += 1

        self.pos = new_pos

    def est(self):
        startnode = Node(self.pos[0], self.pos[1], self.env, True)
        goalnode = Node(self.goal[0], self.goal[1], self.env, True)

        # Start the tree with the startnode (set no parent just in case).
        startnode.parent = None
        tree = [startnode]

        # Function to attach a new node to an existing node: attach the
        # parent, add to the tree, and show in the figure.
        def addtotree(oldnode, newnode):
            newnode.parent = oldnode
            tree.append(newnode)
            # visual.drawEdge(oldnode, newnode, color='g', linewidth=1)
            # visual.show()

        # Loop - keep growing the tree.
        while True:
            # Determine the local density by the number of nodes nearby.
            # KDTree uses the coordinates to compute the Euclidean distance.
            # It returns a NumPy array, same length as nodes in the tree.
            X = np.array([node.coordinates() for node in tree])
            kdtree  = KDTree(X)
            numnear = kdtree.query_ball_point(X, r=1.5*self.speed, return_length=True)

            # Directly determine the distances to the goal node.
            distances = np.array([node.distance(goalnode) for node in tree])

            min_neighbors = min(numnear)
            choices = [i for i in range(len(numnear)) if numnear[i] == min_neighbors]
            grownode = tree[random.choice(choices)]

            # Check the incoming heading, potentially to bias the next node.
            if grownode.parent is None:
                heading = 0
            else:
                heading = atan2(grownode.y - grownode.parent.y,
                                grownode.x - grownode.parent.x)

            # Find something nearby: keep looping until the tree grows.
            while True:
                angle = np.random.normal(heading, np.pi/2) # np.pi / 8 makes worse, need to be able to turn 90 deg

                nextnode = Node(grownode.x + self.speed*np.cos(angle), grownode.y + self.speed*np.sin(angle), self.env, True)

                # Try to connect.
                if nextnode.inFreespace() and (nextnode.connectsTo(grownode)):
                    addtotree(grownode, nextnode)
                    break

            # Once grown, also check whether to connect to goal.
            if nextnode.connectsTo(goalnode):# and nextnode.distance(goalnode) <= self.speed: # nextnode.distance(goalnode) <= self.speed and 
                addtotree(nextnode, goalnode)
                break

            # Check whether we should abort - too many nodes.
            if (len(tree) >= 15000):
                print("Aborted with the tree having %d nodes" % len(tree))
                return None

        # Build the path.
        path = [goalnode]
        while path[0].parent is not None:
            path.insert(0, path[0].parent)
        self.PostProcess(path)

        # return path
        pos_path = []
        for node in path:
            pos_path.append((node.x, node.y))

        self.est_path = pos_path

    # Post process the path.
    def PostProcess(self, path):
        i = 0
        while (i < len(path)-2):
            if path[i].connectsTo(path[i+2]):
                path.pop(i+1)
            else:
                i = i+1
    
    def update_goal(self, new_goal):
        self.goal = new_goal