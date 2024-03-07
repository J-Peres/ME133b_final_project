import sensor, env, buildmap
from buildmap import WIDTH, HEIGHT, MAZE_SIZE, RESOLUTION
from pacman import Pacman

import pygame as pg
import numpy as np

SEED = 42
RMIN = 10
RMAX = 100


def main():
    env_  = env.Environment((WIDTH, HEIGHT), seed=SEED, map_size=MAZE_SIZE)
    laser_ = sensor.LaserSensor(env_.map_img_arr, (0, 0), (WIDTH, HEIGHT), RMIN, RMAX)
    map_  = buildmap.Map()
    
    start, goal = env_.start, env_.goal
    robot_pos = start
    goal = goal
    running = True
    
    probs = np.zeros((HEIGHT, WIDTH))
    pacman = Pacman(robot_pos)
    
    count = 0
    while running:
        sensor_on = True
        for event in pg.event.get():
            if event.type == pg.QUIT:
                print("Quitting...")
                running = False
        #     if pg.mouse.get_focused():
        #         sensor_on = True
        #     elif not pg.mouse.get_focused():
        #         sensor_on = False
        
        if sensor_on:
            laser_.pos = pacman.get_next_pos() #pg.mouse.get_pos()
            sensor_data = laser_.scan()
            env_.process_data(sensor_data) if sensor_data else None
            
            # map_.laserCB(sensor_data, RMIN, RMAX)
            # probs = map_.get_probs()
        
        env_.show(probs)
        count += 1
        
        pg.display.update()
    
    pg.quit()
    
if __name__ == "__main__":
    main()