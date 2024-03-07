import numpy as np



class Pacman:

    def __init__(self, pos: tuple[int, int], speed: int = 1):
        """Initializes the pacman object.

        Args:
            pos (tuple[int, int]): initial position of the pacman
            speed (int, optional): speed of the pacman. Defaults to 1.
        """

        self.pos = pos
        self.speed = speed
        self.direction = None
    
    def get_next_pos(self):
        self.pos = (self.pos[0] + self.speed, self.pos[1])
        return self.pos