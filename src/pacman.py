import numpy as np
from env import Environment

class Pacman:

    def __init__(self, pos: tuple[int, int], speed: int = .1):
        """Initializes the pacman object.

        Args:
            pos (tuple[int, int]): initial position of the pacman
            speed (int, optional): speed of the pacman. Defaults to 1.
        """

        self.pos = pos
        self.speed = speed
        self.direction = None

        self.index = 0
        self.move = 0
    
    def update_pos(self, env):
        start = env.grid_to_pixel(env.true_path[self.index])
        end = env.grid_to_pixel(env.true_path[self.index + 1])
 
        new_pos = self.move * (np.array(end) - np.array(start)) / 100 + np.array(start)
        self.pos = (int(new_pos[0]), int(new_pos[1]))

        if self.move >= 100:
            self.move = 0
            self.index += 1
        else:
            self.move += 1

        return self.pos