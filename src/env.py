import math 
import pygame as pg
import numpy as np
from mazelib import Maze
import time
from mazelib.generate.Prims import Prims
from mazelib.solve.BacktrackingSolver import BacktrackingSolver as BackTracker
import matplotlib.pyplot as plt
from matplotlib import colors
from mazelib.solve.BacktrackingSolver import BacktrackingSolver
from buildmap import RESOLUTION

COLORS = {
    'black': (0, 0, 0),
    'white': (255, 255, 255),
    'red': (255, 0, 0),
    'green': (0, 255, 0),
    'blue': (0, 0, 255)
}

class Environment:
    """The environment for the map and point cloud generated from a laser 
    scan.
    """
    
    def __init__(self, dims, seed=1, map_size=20):
        """Initializes the map environment.

        Args:
            dims (tuple[int, int]): (width, height) of the map window
            seed (int, optional): seed for the random number generator. Defaults to 1.
            map_size (int, optional): size of the map. Defaults to 20.
        """
        
        # Initialize variables
        self.seed = seed
        self.map_size = map_size
        self.walls = None
        self.start = None
        self.goal = None
        self.path = []
        self.point_cloud = []
        self.map_file = f"maps\map_{seed}_{map_size}.png"
        
        # Generate the maze and map image
        self.generate_maze()
        self.generate_map_img()
        
        # Initialize pygame
        pg.init()
        pg.display.set_caption("map")
        self.map = pg.display.set_mode(dims, pg.RESIZABLE)
        self.original_map = pg.display.set_mode(dims, pg.RESIZABLE)
        
        # Load map image
        self.map_img = pg.image.load(self.map_file)
        self.map_img_arr = pg.surfarray.array3d(self.map_img)
        
        # Draw the map
        self.map.blit(self.map_img, (0, 0))
        
        # Draw goal
        pg.draw.circle(self.map, COLORS['green'], self.goal, 25)
    
    def grid_to_pixel(self, pos: tuple[int, int]) -> tuple[int, int]:
        """Converts grid coordinates to pixel coordinates."""
        
        return ((pos[1] + 0.5) / RESOLUTION, (pos[0] + 0.5) / RESOLUTION)

    def pixel_to_grid(self, pos: tuple[int, int]) -> tuple[int, int]:
        """Converts pixel coordinates to grid coordinates."""
        
        return (pos[1] * RESOLUTION - 0.5, pos[0] * RESOLUTION - 0.5)
        
        # ((pos[1] + 0.5) / RESOLUTION, (pos[0] + 0.5) / RESOLUTION)
        
    def generate_maze(self):
        """Generates a maze and sets the walls, start, and goal."""
        
        # Set the seed
        Maze.set_seed(self.seed)

        # Create a maze object
        m = Maze()
        m.generator = Prims(self.map_size // 2, self.map_size // 2)
        m.generate()

        # Set the walls
        self.walls = m.grid
        rows  = np.size(self.walls, axis=0)
        cols  = np.size(self.walls, axis=1)
        
        # Generates start and end, if want them on outer wall its true
        m.generate_entrances(True, True)
        start = m.start
        goal = m.end

        # Make sure the start and end not on outer wall
        if start[0] == 0:
            start = (start[0] + 1, start[1])
        elif start[0] == rows - 1:
            start = (start[0] - 1, start[1])

        if start[1] == 0:
            start = (start[0], start[1] + 1)
        elif start[1] == cols - 1:
            start = (start[0], start[1] - 1)

        if goal[0] == 0:
            goal = (goal[0] + 1, goal[1])
        elif goal[0] == rows - 1:
            goal = (goal[0] - 1, goal[1])

        if goal[1] == 0:
            goal = (goal[0], goal[1] + 1)
        elif goal[1] == cols - 1:
            goal = (goal[0], goal[1] - 1)

        self.start = self.grid_to_pixel(start)
        self.goal = self.grid_to_pixel(goal)

        #IF NEEDED, GENERALLY SHOULDNT BE USED
        # Solve the maze
        m.solver = BackTracker()
        m.solve()

        # Get the path
        self.true_path = m.solutions[0]
    
    def generate_map_img(self, display=False):
        """Generates a map image and saves it to a file."""
        
        # Create an image of the maze
        colormap = colors.ListedColormap(["white", "black"])
        plt.figure(figsize=(8, 8))
        plt.imshow(self.walls, cmap=colormap)
        plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
        plt.axis('off')
        plt.savefig(self.map_file)
        plt.show() if display else None
    
    def calc_point_pos(self, distance: float, angle: float, robot_pos: tuple[int, int]) -> tuple[int, int]:
        """Calculates the position of a point away from robot.

        Args:
            distance (float): distace of point from robot
            angle (float): angle of point from robot
            robot_pos (tuple[int, int]): (x, y) position of robot in map
            
        Returns:
            tuple[int, int]: (x, y) position of point in map
        """
        
        x =  distance * math.cos(angle) + robot_pos[0]
        y = -distance * math.sin(angle) + robot_pos[1]
        return (round(x), round(y))
    
    def process_data(self, data: list[list[float, float, tuple[int, int]]]):
        """Processes the laser scan data and stores it in the point cloud.

        Args:
            data (list[list[float, float, tuple[int, int], bool]]): list of laser scan data
                data[i] = [distance, angle, robot_pos, is_obstacle]
        """
        # Reset point cloud
        for point in self.point_cloud:
            self.map.set_at(point, self.map_img_arr[point[0], point[1]])
        self.point_cloud = []
        
        for distance, angle, robot_pos, is_obstacle in data:
            # Calculate the position of the point
            point = self.calc_point_pos(distance, angle, robot_pos)
            
            if point not in self.point_cloud and is_obstacle:
                self.point_cloud.append(point)
            
            if robot_pos not in self.path:
                self.path.append(robot_pos)
                if len(self.path) > 1:
                    pg.draw.line(self.map, COLORS['blue'], self.path[-2], self.path[-1], 3)
    
    def grid_to_pixel(self, pos: tuple[int, int]) -> tuple[int, int]:
        """Converts grid coordinates to pixel coordinates."""
        
        return ((pos[1] + 0.5) / RESOLUTION, (pos[0] + 0.5) / RESOLUTION)
            
    def show(self, probs: np.ndarray = None, changes: np.ndarray = None):
        """Shows the map image with the point cloud."""
        
        # self.map.blit(self.map_img, (0, 0))
        # Draw the probs
        if probs is not None and changes is not None:
            for (i, j) in changes:
            # for i in range(probs.shape[0]):
            #     for j in range(probs.shape[1]):
                if probs[i, j] > 0:
                    color = np.array([255, 255, 255]) * (1 - probs[i, j])
                    self.map.set_at((j, i), color)

        # Draw point cloud
        for point in self.point_cloud:
            self.map.set_at(point, COLORS['red'])
            # pg.draw.circle(self.map, COLORS['red'], point, 1)

        # Update the display
        pg.display.flip()