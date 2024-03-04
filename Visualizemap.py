import numpy as np
from mazelib import Maze
from mazelib.generate.Prims import Prims
import matplotlib.pyplot as plt
from matplotlib import colors

MAZE_SQUARE_SIZE = 20

def generate_maze(seed, size=MAZE_SQUARE_SIZE):
    Maze.set_seed(seed)

    m = Maze()
    m.generator = Prims(size // 2, size // 2)
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

walls, start, goal = generate_maze(42, MAZE_SQUARE_SIZE)

data = np.array(walls)
colormap = colors.ListedColormap(["white", "black"])
plt.imshow(data, cmap = colormap)
plt.axis('off')
plt.show()