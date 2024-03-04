import math 
import pygame
import numpy as np
from mazelib import Maze
from mazelib.generate.Prims import Prims
import matplotlib.pyplot as plt
from matplotlib import colors

class Buildenv:
    def __init__(self, MapDimensions, seed=1, map_size=20):
        self.seed = seed
        self.map_size = map_size
        self.generate_maze()
        self.generate_map_img()
        
        pygame.init()
        self.pointcloud=[]
        self.externalMap=pygame.image.load("map.png")
        self.maph, self.mapw=MapDimensions
        self.MapwindowsName="RRT path Planing"
        pygame.display.set_caption(self.MapwindowsName)
        self.map=pygame.display.set_mode((self.mapw, self.maph), pygame.RESIZABLE)
        
        
        # colors
        self.black=(0,0,0)
        self.grey=(70,70,70)
        self.blue=(0,0,255)
        self.green=(0,255,0)
        self.red=(255,0,0)
        self.white=(255,255,255)
    
    def AD2pos(self, distance, angle, robotposition):
        x=distance * math.cos(angle)+robotposition[0]
        y=-distance * math.sin(angle)+robotposition[1]
        return (int(x), int(y))
    
    def datastorage(self,data):
        print(len(self.pointcloud))
        for element in data:
            point=self.AD2pos(element[0], element[1], element[2])
            if point not in self.pointcloud:
                self.pointcloud.append(point)
    
    def show_sonsorData(self):
        self.infomap=self.map.copy()
        for point in self.pointcloud:
            self.infomap.set_at((int(point[0]), int(point[1])), (255,0,0))
        

    def generate_maze(self):
        Maze.set_seed(self.seed)

        m = Maze()
        m.generator = Prims(self.map_size // 2, self.map_size // 2)
        m.generate()

        walls = m.grid
        rows  = np.size(walls, axis=0)
        cols  = np.size(walls, axis=1)

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
            
        return walls, start, goal
    
    def generate_map_img(self):
        self.walls, self.start, self.goal = self.generate_maze()

        data = np.array(self.walls)
        colormap = colors.ListedColormap(["white", "black"])
        plt.imshow(data, cmap = colormap)
        plt.axis('off')
        plt.savefig("map.png")
        plt.show()
