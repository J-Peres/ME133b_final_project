import numpy as np
from mazelib import Maze
from mazelib.generate.Prims import Prims

from visualization import Visualization, Robot

#
#   Global Definitions
#

MAZE_SQUARE_SIZE = 20
WIDTH  = MAZE_SQUARE_SIZE
HEIGHT = MAZE_SQUARE_SIZE

RESOLUTION = 1
ORIGIN_X   = -10              # Origin = location of lower-left corner
ORIGIN_Y   = -10

LFREE     = -0.1         # Set the log odds ratio of detecting freespace
LOCCUPIED = 0.1         # Set the log odds ratio of detecting occupancy

class Map:
    def __init__(self):
        # Create the log-odds-ratio grid.
        self.logoddsratio = np.zeros((HEIGHT, WIDTH))
    
    def sendMap(self):
        # Convert the log odds ratio into a probability (0...1).
        # Remember: self.logsoddsratio is a 3460x240 NumPy array,
        # where the values range from -infinity to +infinity.  The
        # probability should also be a 360x240 NumPy array, but with
        # values ranging from 0 to 1, being the probability of a wall.
       
        probability = 1 - 1/(1+np.exp(self.logoddsratio))

        # Perpare the message and send.  Note this converts the
        # probability into percent, sending integers from 0 to 100.
        data = (100 * probability).astype(int).flatten().tolist()

        return data

    ##################################################################
    # Utilities:
    # Set the log odds ratio value
    def set(self, u, v, value):
        # Update only if legal.
        if (u>=0) and (u<WIDTH) and (v>=0) and (v<HEIGHT):
            self.logoddsratio[v,u] = value
        else:
            print("Out of bounds (%d, %d)" % (u,v))

    # Adjust the log odds ratio value
    def adjust(self, u, v, delta):
        # Update only if legal.
        if (u>=0) and (u<WIDTH) and (v>=0) and (v<HEIGHT):
            self.logoddsratio[v,u] += delta
        else:
            print("Out of bounds (%d, %d)" % (u,v))

    # Return a list of all intermediate (integer) pixel coordinates
    # from (start) to (end) coordinates (which could be non-integer).
    # In classic Python fashion, this excludes the end coordinates.
    def bresenham(self, start, end):
        # Extract the coordinates
        (xs, ys) = start
        (xe, ye) = end

        # Move along ray (excluding endpoint).
        if (np.abs(xe-xs) >= np.abs(ye-ys)):
            return[(u, int(ys + (ye-ys)/(xe-xs) * (u+0.5-xs)))
                   for u in range(int(xs), int(xe), int(np.sign(xe-xs)))]
        else:
            return[(int(xs + (xe-xs)/(ye-ys) * (v+0.5-ys)), v)
                   for v in range(int(ys), int(ye), int(np.sign(ye-ys)))]


    ##################################################################
    # Laserscan CB.  Process the scans.
    def laserCB(self, data: list[list]):
        # data[i] = [range, theta, (xc, yc)]
        
        # Extract the laser scanner's position and orientation.
        xc = tfmsg.transform.translation.x
        yc = tfmsg.transform.translation.y

        # Grab the rays: each ray's range and angle relative to the
        # turtlebot's position and orientation.
        rmin     = msg.range_min        # Sensor minimum range to be valid
        rmax     = msg.range_max        # Sensor maximum range to be valid
        ranges   = msg.ranges           # List of ranges for each angle

        thetamin = msg.angle_min        # Min angle (0.0)
        thetamax = msg.angle_max        # Max angle (2pi)
        thetainc = msg.angle_increment  # Delta between angles (2pi/360)
        thetas   = np.arange(thetamin, thetamax, thetainc)

        # Process each ray.     
        for i, r in enumerate(ranges):
            if rmin < r:
                l_occupied = LOCCUPIED
                if r > rmax:
                    r = rmax
                    l_occupied = 0
                    
                x_r, y_r = xc + r * np.cos(thetas[i]), yc + r * np.sin(thetas[i])
                
                xs = (xc - ORIGIN_X) * 1/RESOLUTION
                ys = (yc - ORIGIN_Y) * 1/RESOLUTION

                xe = (x_r - ORIGIN_X) * 1/RESOLUTION
                ye = (y_r - ORIGIN_Y) * 1/RESOLUTION
                for (u, v) in self.bresenham((xs, ys), (xe, ye)):
                    self.adjust(u, v, LFREE)

                u_r = int(xe)
                v_r = int(ye)
                self.adjust(u_r, v_r, l_occupied)

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
    

def main():
    walls, start, goal = generate_maze(42, MAZE_SQUARE_SIZE)
    
    print(walls)
    
    robot = Robot(walls, row=start[0], col=start[1])

    # Initialize the figure.
    visual = Visualization(walls, robot, goal)

    bel = (1.0 - walls) / np.sum(1.0 - walls)

    # Loop continually.
    while True:
        # Show the current belief.  Also show the actual position.
        visual.Show(bel, markRobot=True)
        
        # Get the command key to determine the direction.
        while True:
            key = input("Cmd (q=quit, i=up, m=down, j=left, k=right) ?")
            if   (key == 'q'):  return
            elif (key == 'i'):  (drow, dcol) = (-1,  0) ; break
            elif (key == 'm'):  (drow, dcol) = ( 1,  0) ; break
            elif (key == 'j'):  (drow, dcol) = ( 0, -1) ; break
            elif (key == 'k'):  (drow, dcol) = ( 0,  1) ; break
            
        
        # Move the robot in the simulation.
        robot.Command(drow, dcol)

if __name__== "__main__":
    main()