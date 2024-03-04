import sensor,env
import pygame
import math 

width, height = 640, 480
environment=env.Buildenv((height, width))

environment.originalMap=environment.map.copy()

laser=sensor.LaserSensor(200, environment.originalMap, uncertainty=(0, 0), width=width, height=height)
environment.map.fill((0,0,0))
environment.infomap=environment.map.copy()


running=True


while running:
    sensorON=False
    for envent in pygame.event.get():
        if envent.type==pygame.QUIT:
            running=False
        if pygame.mouse.get_focused():
            sensorON=True
        elif not pygame.mouse.get_focused():
            sensorON=False
    if sensorON:
        position=pygame.mouse.get_pos()
        laser.position=position
        sensor_data=laser.sense_obstacles()
        environment.datastorage(sensor_data)
        environment.show_sonsorData()
    # environment.map.blit(environment.externalMap, (0,0))
    # environment.map.blit(environment.infomap, (0,0))
    pygame.display.update()
