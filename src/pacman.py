import numpy as np
from env import Environment
from node import Node
from scipy.spatial      import KDTree
from math               import pi, sin, cos, atan2, sqrt, ceil
import random
from constants import C_GOAL, C_GHOST, C_LOR, WIDTH, HEIGHT, COLORS
from utils import euclidean
from constants import SCAN_RESOLUTION

class Pacman:
    def __init__(self, pos: tuple[int, int], goal: tuple[int, int], env: Environment, speed: int = .1):
        """Initializes the pacman object.

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

        self.costs = np.ones((WIDTH, HEIGHT)) * 1e9
        self.path = [pos]
        self.target_pos = None
    
    def update_pos(self, path):
        if path is None and self.target_pos is not None:
            # Get angle to target_pos
            angle = atan2(self.target_pos[1] - self.pos[1], self.target_pos[0] - self.pos[0])
            # Move pacman
            new_pos = (int(self.pos[0] + self.speed * cos(angle)), int(self.pos[1] + self.speed * sin(angle)))
            
            # color = tuple(self.env.map_img_arr[new_pos[0], new_pos[1]])
            # if color != COLORS['black']:
            
            self.pos = new_pos
            self.path.append(new_pos)

        if path is not None:
            try:
                start = Node(self.pos[0], self.pos[1], self.env, True)
                end = Node(path[self.index + 1][0], path[self.index + 1][1], self.env, True)
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
    
    def update_costs(self, probs, changes):
        if probs is not None and changes is not None:
            for (i, j) in changes:
                node = Node(i, j, self.env)
                if probs[i, j] > 0.5 and node.inFreespace():
                    self.costs[i, j] = C_GOAL * euclidean((i, j), self.goal) + C_GHOST * 0 + C_LOR * probs[i, j]
        
    def update_target_pos(self):
        self.target_pos = np.unravel_index(np.argmin(self.costs, axis=None), self.costs.shape)
        
        # curr_node   = Node(self.pos[0], self.pos[1], self.env)
        # target_node = Node(target_pos[0], target_pos[1], self.env)
        
        # if target_node.inFreespace() and (curr_node.connectsTo(target_node)):
        #     self.target_pos = target_pos
        
        # Print target_pos
        print(f'target_pos: {self.target_pos}')
        
        # # Print current pos
        # print(f'current pos: {self.pos}')
        
        # print cost at target_pos
        print(f'cost at target_pos: {self.costs[self.target_pos]}')
        # print cost at (258, 227)
        # print(f'cost at (258, 227): {self.costs[258, 227]}\n')