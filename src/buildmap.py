import numpy as np
import time

WIDTH, HEIGHT = 800, 800
MAZE_SIZE = 20

RESOLUTION =  (MAZE_SIZE + 1) / WIDTH
LFREE     = -0.1     # Set the log odds ratio of detecting freespace
LOCCUPIED =  0.1     # Set the log odds ratio of detecting occupancy

class Map:
    def __init__(self):
        # Create the log-odds-ratio grid.
        self.logoddsratio = np.zeros((HEIGHT, WIDTH))

        self.old_prob = None

    def get_probs(self):
        """Get probabilities from the logoddsratio."""
        
        # Convert the log odds ratio into probabilities (0...1).
        probs = 1 - 1/(1+np.exp(self.logoddsratio))

        probs[probs > 1] = 1

        if self.old_prob is None:
            self.old_prob = probs
            changes = [(i, j) for i in range(HEIGHT) for j in range(WIDTH)]
        else:
            changes = np.argwhere(np.abs(probs - self.old_prob) > 0.01)
            self.old_prob = probs
        
        return probs, changes
    
    def set(self, u: int, v: int, value: float):
        """Set the log odds ratio value."""
        
        # Update only if legal.
        if (u>=0) and (u<WIDTH) and (v>=0) and (v<HEIGHT):
            self.logoddsratio[v,u] = value
        else:
            print("Out of bounds (%d, %d)" % (u,v))
        
    def adjust(self, u: int, v: int, delta: float):
        """Adjust the log odds ratio value."""
        
        # Update only if legal.
        if (u>=0) and (u<WIDTH) and (v>=0) and (v<HEIGHT):
            self.logoddsratio[v,u] += delta
        else:
            print("Out of bounds (%d, %d)" % (u,v))
        
    def bresenham(self, start: tuple[int, int], end: tuple[int, int]):
        """Return a list of all intermediate (integer) pixel coordinates from (start) to (end) coordinates (which could be non-integer)."""
        
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
    
    def laserCB(self, data: list[list[float, float, tuple[int, int]]], rmin: float, rmax: float):
        """

        Args:
            list[list[float, float, tuple[int, int], bool]]: list of laser scan data
                data[i] = [r, theta, laser_pos, is_obstacle]
        """
        
        # If no scanned data, return
        if not data:
            return
        
        xc, yc = data[0][2]

        # Convert the laser position to pixel coordinates
        xs = xc
        ys = yc
        
        for r, theta, _, is_obstacle in data:
            if rmin < r:
                l_occ = LOCCUPIED
                if not is_obstacle:
                    l_occ = 0
                
                # Calculate the endpoint of the laser
                x_r = xc + r * np.cos(theta)
                y_r = yc - r * np.sin(theta)
                
                # Convert the endpoint to pixel coordinates
                xe = x_r
                ye = y_r
                
                # Set points from laser to endpoint as free
                for (u, v) in self.bresenham((xs, ys), (xe, ye)):
                    self.adjust(u, v, LFREE)
                
                # Set the endpoint as occupied
                self.set(int(xe), int(ye), l_occ)