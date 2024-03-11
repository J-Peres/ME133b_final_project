import sensor, env, buildmap
from buildmap import WIDTH, HEIGHT, MAZE_SIZE, RMIN, RMAX, SEED
from pacman import Pacman

import pygame as pg
import numpy as np

# path = 'est'
# path = 'true'
path = None

def main():
    env_  = env.Environment((WIDTH, HEIGHT), seed=SEED, map_size=MAZE_SIZE)
    laser_ = sensor.LaserSensor(env_.map_img_arr, (0, 0), (WIDTH, HEIGHT), RMIN, RMAX, scan_resolution=15, heading_resolution=120)
    map_  = buildmap.Map()
    
    start, goal = env_.start, env_.goal
    robot_pos = start
    goal = goal
    running = True
    
    probs = np.zeros((HEIGHT, WIDTH))
    changes = None
    
    pacman = Pacman(robot_pos, goal, env_, 10)

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
                
        # if count > 2:
        #     sensor_on = False   
        
        if sensor_on:
            if path == 'est':
                laser_.pos = pacman.update_pos(pacman.est_path)
            elif path == 'true':
                laser_.pos = pacman.update_pos(env_.true_path) 
            elif path is None:
                pacman.update_costs(probs, changes)
                if probs.max() > 0.5:
                    pacman.calc_target_pos()
                laser_.pos = pacman.update_pos(None) 

            if laser_.pos is None:
                print('GOAL REACHED!')
                sensor_on = False
            
            sensor_data = laser_.scan()
            env_.process_data(sensor_data) if sensor_data else None
            
            map_.laserCB(sensor_data, RMIN, RMAX)
            probs, changes = map_.get_probs()
        
        
        env_.show(probs, changes)
        count += 1
        pg.display.update() 

    pg.quit()
    
if __name__ == "__main__":
    main()