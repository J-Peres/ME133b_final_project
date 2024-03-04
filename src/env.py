import math 
import pygame as pg
import numpy as np
from mazelib import Maze
from mazelib.generate.Prims import Prims
import matplotlib.pyplot as plt
from matplotlib import colors


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
            start =     (start[0] - 1, start[1])

        if start[1] == 0:
            start = (start[0], start[1] + 1)
        elif start[1] == cols - 1:
            start = (start[0], start[1] - 1)
        
        self.start = start
        self.goal = goal
        
    def generate_map_img(self):
        """Generates a map image and saves it to a file."""
        
        # Create an image of the maze
        colormap = colors.ListedColormap(["white", "black"])
        plt.figure(figsize=(8, 8))
        plt.imshow(self.walls, cmap=colormap)
        plt.subplots_adjust(left=0, right=1, top=1, bottom=0)
        plt.axis('off')
        plt.savefig(self.map_file)
        plt.show()
        
    
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
            data (list[list[float, float, tuple[int, int]]]): list of laser scan data
                data[i] = [distance, angle, robot_pos]
        """
        for distance, angle, robot_pos in data:
            # Calculate the position of the point
            point = self.calc_point_pos(distance, angle, robot_pos)
            
            if point not in self.point_cloud:
                self.point_cloud.append(point)
    
    def show(self, 
             robot_pos: tuple[int, int], 
             goal: tuple[int, int], 
             probs: np.ndarray, 
             path: list[tuple[int, int]]):
        """Shows the map image with the point cloud."""
        
        # Draw the map
        self.map.blit(self.map_img, (0, 0))
        
        # Draw the point cloud
        for point in self.point_cloud:
            pg.draw.circle(self.map, (255, 0, 0), point, 1)
            
        # Draw the robot
        pg.draw.circle(self.map, (0, 0, 255), robot_pos, 5)
        
        # Draw the goal
        pg.draw.circle(self.map, (0, 255, 0), goal, 25)

        # # Draw the probs
        # if probs is not None:
        #     for i in range(probs.shape[0]):
        #         for j in range(probs.shape[1]):
        #             color = tuple(self.map_img_arr[i, j])
        #             if color == (0, 0, 0):
        #                 continue
                    
        #             if probs[i, j] > 0:
        #                 pg.draw.circle(self.map, (0, 0, 128 * probs[i, j]), (i, j), 1)
            
        # Update the display
        pg.display.flip()