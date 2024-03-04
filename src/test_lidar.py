import sensor, env
import pygame as pg

WIDTH, HEIGHT = 800, 800

def main():
    env_ = env.Environment((WIDTH, HEIGHT))
    laser = sensor.LaserSensor(pg.surfarray.array3d(env_.map_img), (0, 0), (WIDTH, HEIGHT))
    
    running = True
    
    while running:
        sensor_on = False
        for event in pg.event.get():
            if event.type == pg.QUIT:
                print("Quitting...")
                running = False
            if pg.mouse.get_focused():
                sensor_on = True
            elif not pg.mouse.get_focused():
                sensor_on = False
                
        if sensor_on:
            mouse_pos = pg.mouse.get_pos()
            laser.pos = mouse_pos
            sensor_data = laser.scan()
            env_.process_data(sensor_data) if sensor_data else None
        
        env_.show()
        
        pg.display.update()
    
    pg.quit()

if __name__ == "__main__":
    main()