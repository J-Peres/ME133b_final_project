import numpy as np
from env import Environment
from node import Node
from scipy.spatial      import KDTree
from math               import pi, sin, cos, atan2, sqrt, ceil
import random

class Pacman:
    def __init__(self, pos: tuple[int, int], goal: tuple[int, int], speed: int = .1):
        """Initializes the pacman object.

        Args:
            pos (tuple[int, int]): initial position of the pacman
            speed (int, optional): speed of the pacman. Defaults to 1.
        """

        self.pos = pos
        self.speed = speed
        self.direction = None

        self.start = pos
        self.goal = goal

        self.index = 0
        self.move = 0

        self.intermediate_points = 50
    
    def update_pos(self, env):
        # start = env.grid_to_pixel(env.true_path[self.index])
        # end = env.grid_to_pixel(env.true_path[self.index + 1])

        try:
            start = self.est_path[self.index]
            end = self.est_path[self.index + 1]
        except IndexError:
            return None
 
        new_pos = self.move * (np.array(end) - np.array(start)) / self.intermediate_points + np.array(start)
        self.pos = (int(new_pos[0]), int(new_pos[1]))

        if self.move >= self.intermediate_points:
            self.move = 0
            self.index += 1
        else:
            self.move += 1

        return self.pos

    def est(self, env):
        startnode = Node(self.pos[0], self.pos[1], env)
        goalnode = Node(self.goal[0], self.goal[1], env)

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
            # print('HI')
            # Determine the local density by the number of nodes nearby.
            # KDTree uses the coordinates to compute the Euclidean distance.
            # It returns a NumPy array, same length as nodes in the tree.
            X = np.array([node.coordinates() for node in tree])
            kdtree  = KDTree(X)
            numnear = kdtree.query_ball_point(X, r=1.5*self.speed, return_length=True)

            # Directly determine the distances to the goal node.
            distances = np.array([node.distance(goalnode) for node in tree])

            # Select the node from which to grow, which minimizes some metric.

            # -------PART A---------
            min_neighbors = min(numnear)
            choices = [i for i in range(len(numnear)) if numnear[i] == min_neighbors]
            grownode = tree[random.choice(choices)]
            # -------PART A---------

            # -------PART C---------
            # scale = 5
            # new_metric = np.array([numnear[i] + scale * distances[i] for i in range(len(X))])
            # index     = np.argmin(new_metric)
            # grownode  = tree[index]
            # -------PART C---------

            # Check the incoming heading, potentially to bias the next node.
            if grownode.parent is None:
                heading = 0
            else:
                heading = atan2(grownode.y - grownode.parent.y,
                                grownode.x - grownode.parent.x)

            # Find something nearby: keep looping until the tree grows.
            while True:
                # Pick the next node randomly.
                # -------PART A---------
                angle = random.uniform(-np.pi, np.pi)

                # -------PART B---------
                # angle = np.random.normal(heading, np.pi/2)
                
                # ---------ALL----------
                nextnode = Node(grownode.x + self.speed*np.cos(angle), grownode.y + self.speed*np.sin(angle), env)

                # Try to connect.
                if nextnode.inFreespace() and (nextnode.connectsTo(grownode)):
                    addtotree(grownode, nextnode)
                    break
                
                # print("INNER HI")

            # Once grown, also check whether to connect to goal.
            if nextnode.distance(goalnode) <= self.speed and nextnode.connectsTo(goalnode):
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