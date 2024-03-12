COLORS = {
    'black': (0, 0, 0),
    'white': (255, 255, 255),
    'red': (255, 0, 0),
    'green': (0, 255, 0),
    'blue': (0, 0, 255),
    'yellow': (255, 255, 0)
}

WIDTH, HEIGHT = 800, 800
MAZE_SIZE = 20

RESOLUTION =  (MAZE_SIZE + 1) / WIDTH
LFREE     = -0.1     # Set the log odds ratio of detecting freespace
LOCCUPIED =  0.1     # Set the log odds ratio of detecting occupancy

LMAX = None

C_GOAL  = .001
C_GHOST = 0
C_LOR   = 10

SEED = 42
RMIN = 10
RMAX = 100
SCAN_RESOLUTION = 30
HEADING_RESOLUTION = 120