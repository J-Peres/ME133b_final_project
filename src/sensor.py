import pygame as pg
import math
import numpy as np

class LaserSensor:
    """The laser sensor class."""
    
    def __init__(self, map: pg.Surface, uncertainty: tuple[float, float], dims: tuple[int, int], rmin: float, rmax: float, scan_resolution: int = 100, heading_resolution: int = 60):
        """Initializes the laser sensor.
        
        Args:
            map (pg.Surface): map image
            uncertainty (tuple[float, float]): standard deviation of distance and angle
            dims (tuple[int, int]): (width, height) of the map window
            rmin (float, optional): minimum range of the laser.
            rmax (float, optional): maximum range of the laser.
        """
        
        self.map = map
        self.pos = (0, 0)
        self.w, self.h = dims
        
        # NOTE: setting rmin to 0 temporarily until fixed
        self.rmin = 0
        
        self.rmax = rmax
        self.sigma = np.array(uncertainty)
        self.scan_resolution = scan_resolution
        self.heading_resolution = heading_resolution
        
        self.sensed_obstacles = []
    
    def euclidean(self, p: tuple[int, int]):
        """Calculates the euclidean distance between the sensor and a point."""
        
        return math.sqrt((p[0] - self.pos[0])**2 + (p[1] - self.pos[1])**2)
    
    def add_uncertainty(self, distance: float, angle: float, sigma) -> list[float, float]:
        """Adds uncertainty to the distance and angle measurements.
        
        Args:
            distance (float): distance measurement
            angle (float): angle measurement
            sigma (tuple[float, float]): standard deviation of distance and angle
        
        Returns:
            list[float, float]: distance and angle with added uncertainty
        """
        
        mean = np.array([distance, angle])
        covariance = np.diag(sigma**2)
        distance, angle = np.random.multivariate_normal(mean, covariance)
        distance = max(distance, 0)
        angle = max(angle, 0)
        return [distance, angle]
    
    def scan(self):
        """Scans the environment for obstacles and returns the data.
        
        Returns:
            list[list[float, float, tuple[int, int]]]: list of laser scan data
                data[i] = [distance, angle, robot_pos]
                False if no obstacles are found
        """
        
        data = []
        
        # Iterate through all headings
        for theta in np.linspace(0, 2*math.pi, self.heading_resolution, False):
            target = (self.pos[0] + self.rmax * math.cos(theta), 
                      self.pos[1] - self.rmax * math.sin(theta))
            
            # Iterate through all points along the ray
            for i in range(self.scan_resolution):
                u = i/self.scan_resolution
                x = round(target[0] * u + self.pos[0] * (1-u))
                y = round(target[1] * u + self.pos[1] * (1-u))
                
                # Check if the point is within the map
                if 0 < x < self.w and 0 < y < self.h:
                    # Check if the point is an obstacle
                    color = tuple(self.map[x, y])
                    if color == (0, 0, 0):
                        # Add the point to the data
                        distance = self.euclidean((x, y))
                        if distance > self.rmin:    # Ignore points before rmin
                            output = self.add_uncertainty(distance, theta, self.sigma)
                            output.append(self.pos)
                            data.append(output)
                            break
            
        if len(data) > 0:
            return data
        else:
            return False