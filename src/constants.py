COLORS = {
    'black': (0, 0, 0),
    'white': (255, 255, 255),
    'red': (255, 0, 0),
    'green': (0, 255, 0),
    'blue': (0, 0, 255)
}

WIDTH, HEIGHT = 840, 840
MAZE_SIZE = 20

RESOLUTION =  (MAZE_SIZE + 1) / WIDTH
LFREE     = -0.1     # Set the log odds ratio of detecting freespace
LOCCUPIED =  0.1     # Set the log odds ratio of detecting occupancy
GRID_SIZE = int(1 / RESOLUTION)

LMAX = None

C_GOAL  = .1
C_GHOST = 0
C_LOR   = 10

SEED = 42
RMIN = 10
RMAX = 100

SCAN_RESOLUTION = 30
HEADING_RESOLUTION = 120