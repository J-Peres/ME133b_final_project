import sensor, env, buildmap
from constants import *
from utils import within_thresh
from entity import Entity

import pygame as pg
import numpy as np
import time

path = 'est'
# path = 'true'
# path = None

def main():
    env_  = env.Environment((WIDTH, HEIGHT), seed=SEED, map_size=MAZE_SIZE, loops=7)
    laser_ = sensor.LaserSensor(env_.map_img_arr, (0, 0), (WIDTH, HEIGHT), RMIN, RMAX, scan_resolution=SCAN_RESOLUTION, heading_resolution=HEADING_RESOLUTION)
    
    ghost_lasers = []
    for i in range(NUM_GHOSTS):
        ghost_lasers.append(sensor.LaserSensor(env_.map_img_arr, (0, 0), (WIDTH, HEIGHT), RMIN, RMAX, scan_resolution=SCAN_RESOLUTION, heading_resolution=HEADING_RESOLUTION))
    
    map_ = buildmap.Map()
    
    start, goal = env_.start, env_.goal
    robot_pos = start
    goal = goal
    running = True

    probs = np.zeros((WIDTH, HEIGHT))
    changes = None
    
    pacman = Entity(robot_pos, goal, env_, 15, pacman=True)
    
    ghosts: list[Entity] = []
    for i in range(NUM_GHOSTS):
        ghosts.append(Entity(env_.ghost_pos[i], tuple(pacman.pos), env_, 15, pacman=False))
        
    if path == 'est':
        pacman.est()
        for ghost in ghosts:
            ghost.est()

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

        for ghost in ghosts:
            if within_thresh(ghost.pos, pacman.pos, CAUGHT_DISTANCE):
                print('Pacman caught by ghost')
                sensor_on = False
        
        if within_thresh(pacman.pos, goal, CAUGHT_DISTANCE):
            print('Pacman reached goal')
            sensor_on = False
    
        # Ping locations
        if count % PING_PERIOD == 0:
            # Reinitialize ghosts (for some reason just setting goal to pacman.pos doesn't work)
            ghosts: list[Entity] = []
            for i in range(NUM_GHOSTS):
                ghosts.append(Entity(env_.ghost_pos[i], tuple(pacman.pos), env_, 15, pacman=False))
            if path == 'est':
                pacman.est()
                for ghost in ghosts:
                    ghost.est()
            
            # Set ghost positions for pacman
            for i, ghost in enumerate(ghosts):
                pacman.ghost_pos[i] = tuple(ghost.pos)  # using tuple to ensure actual ghost position is not changed
            
            # print('updated ghost goals')

        if sensor_on:
            if path == 'est':
                pacman.update_pos(pacman.est_path)
                for i, ghost in enumerate(ghosts):
                    ghost.update_pos(ghost.est_path)
                    env_.ghost_pos[i] = ghost.pos
            elif path == 'true':
                pacman.update_pos(env_.true_path) 
            elif path is None:
                if changes is not None:
                    pacman.update_costs(probs, changes)
                    pacman.update_target_pos()
                pacman.update_pos(None) 

            laser_.pos = pacman.pos
            for i, ghost_laser in enumerate(ghost_lasers):
                ghost_laser.pos = ghosts[i].pos

            if laser_.pos is None:
                sensor_on = False

            sensor_data = laser_.scan()
            ghost_data = [ghost_laser.scan() for ghost_laser in ghost_lasers]

            env_.process_data(sensor_data) if sensor_data else None
            for i, data in enumerate(ghost_data):
                env_.process_data(data, player=i+1) if data else None
            
            map_.laserCB(sensor_data, RMIN, RMAX)
            for i, data in enumerate(ghost_data):
                map_.laserCB(data, RMIN, RMAX)

            probs, changes = map_.get_probs()
        
        
        if path == 'est':
            env_.show(probs, changes, pacman.pos)
            # env_.show(None, None, pacman.pos)
            for i, ghost in enumerate(ghosts):
                env_.show(None, None, ghost.pos, player=i+1)
        else:
            env_.show(probs, changes)

        count += 1
        pg.display.update()

        if count % 3 == 0:
            env_.show_learned_maps()

    pg.quit()
    
if __name__ == "__main__":
    main()
