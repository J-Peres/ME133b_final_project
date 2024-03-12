import sensor, env, buildmap
from buildmap import WIDTH, HEIGHT, MAZE_SIZE, RMIN, RMAX, SEED, SCAN_RESOLUTION, HEADING_RESOLUTION
from pacman import Pacman

import pygame as pg
import numpy as np
import time

path = 'est'
# path = 'true'
# path = None

def main():
    env_  = env.Environment((WIDTH, HEIGHT), seed=SEED, map_size=MAZE_SIZE)
    laser_ = sensor.LaserSensor(env_.map_img_arr, (0, 0), (WIDTH, HEIGHT), RMIN, RMAX, scan_resolution=SCAN_RESOLUTION, heading_resolution=HEADING_RESOLUTION)
    map_  = buildmap.Map()
    
    start, goal = env_.start, env_.goal
    robot_pos = start
    goal = goal
    running = True

    probs = np.zeros((WIDTH, HEIGHT))
    changes = None
    
    pacman = Pacman(robot_pos, goal, env_, 15)

    if path == 'est':
        pacman.est()

    count = 0
    while running:
        sensor_on = True
        for event in pg.event.get():
            if event.type == pg.QUIT:
                print("Quitting...")
                running = False
            if pg.mouse.get_focused():
                # print mouse if press space
                if event.type == pg.KEYDOWN:
                    if event.key == pg.K_SPACE:
                        print(f'mouse: {pg.mouse.get_pos()}')
                        print(pacman.costs[pg.mouse.get_pos()[0], pg.mouse.get_pos()[1]])

        # if count > 100:
        #     sensor_on = False

        if sensor_on:
            if path == 'est':
                pacman.update_pos(pacman.est_path)
            elif path == 'true':
                pacman.update_pos(env_.true_path) 
            elif path is None:
                if changes is not None:
                    pacman.update_costs(probs, changes)
                    pacman.update_target_pos()
                pacman.update_pos(None) 

            laser_.pos = pacman.pos

            if laser_.pos is None:
                sensor_on = False

            sensor_data = laser_.scan()
            env_.process_data(sensor_data) if sensor_data else None
            
            map_.laserCB(sensor_data, RMIN, RMAX)
            probs, changes = map_.get_probs()
        
            # Print max cummuative probs
            # if count % 10 == 0:
            #     print(f'max cummulative probs: {cummulative_probs.max()}')
        
        if path == 'est':
            env_.show(None, None, pacman.pos)
        else:
            env_.show(probs, changes)
        count += 1
        pg.display.update()

    pg.quit()
    
if __name__ == "__main__":
    main()