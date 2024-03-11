COLORS = {
    'black': (0, 0, 0),
    'white': (255, 255, 255),
    'red': (255, 0, 0),
    'green': (0, 255, 0),
    'blue': (0, 0, 255)
}

WIDTH, HEIGHT = 800, 800
MAZE_SIZE = 20

RESOLUTION =  (MAZE_SIZE + 1) / WIDTH
LFREE     = -0.1     # Set the log odds ratio of detecting freespace
LOCCUPIED =  0.1     # Set the log odds ratio of detecting occupancy

LMAX = None

C_GOAL  = 1
C_GHOST = 2
C_LOR   = 30
