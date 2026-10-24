import pygame
import math
import sys
import os
from collections import deque

# 1. Path Fix
parent_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))
if parent_dir not in sys.path:
    sys.path.append(parent_dir)

from software_emulation.differential_drive_emulator import DifferentialDriveEmulator
from software_emulation.rplidar_emulator import RPLidar
from software_emulation.wall import Wall

# --- Constants & Colors ---
WIDTH, HEIGHT = 800, 600
SCALE = 50.0  
FPS = 60

BLACK = (20, 20, 20)
WHITE = (240, 240, 240)
CYAN  = (0, 255, 255)
RED   = (255, 50, 50)
GREEN = (50, 255, 50)

def to_screen(x, y):
    return int(WIDTH / 2.0 + (x * SCALE)), int(HEIGHT / 2.0 - (y * SCALE))

class Robot:
    def __init__(self, environment_walls, x=0.0, y=0.0):
        self.x = x
        self.y = y
        self.theta = 0.0
        
        self.motors = DifferentialDriveEmulator()
        self.lidar = RPLidar(environment_walls)

    def update(self, dt):
        v, w = self.motors.step()
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt
        self.theta += w * dt

def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Differential Drive & LiDAR Sim")
    clock = pygame.time.Clock()

    walls = [
        Wall((-5, -5), (5, -5)), 
        Wall((5, -5),  (5, 5)),
        Wall((5, 5),   (-5, 5)), 
        Wall((-5, 5),  (-5, -5)),
        Wall((-2, -2), (-2, 2)), 
        Wall((1, 3),   (3, 0))
    ]

    robot = Robot(walls, x=0.0, y=-3.0)
    
    # Create a rolling buffer to accumulate the last 500 lidar points
    # 40 points per cabin * ~12.5 cabins for a 360 degree sweep = 500 points
    point_cloud = deque(maxlen=500)
    
    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0 

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        keys = pygame.key.get_pressed()
        req_v = 0.0
        req_w = 0.0
        if keys[pygame.K_UP]:    req_v =  1.5
        if keys[pygame.K_DOWN]:  req_v = -1.5
        if keys[pygame.K_LEFT]:  req_w =  3.0
        if keys[pygame.K_RIGHT]: req_w = -3.0

        robot.motors.set_targets(req_v, req_w)
        robot.update(dt)

        # --- LiDAR Cabin Extraction & Interpolation ---
        theta_deg = math.degrees(robot.theta)
        
        # Pull exactly one cabin (40 points) from the emulator
        start_angle, cabin_data = robot.lidar.get_next_cabin(robot.x, robot.y, theta_deg)
        
        # Interpolate the angles for each distance in the cabin
        for i, dist in enumerate(cabin_data):
            if dist < 12.0:
                # The local ray angle (start angle + 0.72 degrees per point)
                ray_angle_deg = start_angle + (i * 0.72)
                
                # Convert to a global angle by adding the robot's current heading
                global_angle_rad = robot.theta + math.radians(ray_angle_deg)
                
                # Calculate the X, Y coordinate of the physical hit
                hx = robot.x + (dist * math.cos(global_angle_rad))
                hy = robot.y + (dist * math.sin(global_angle_rad))
                
                # Accumulate the point
                point_cloud.append((hx, hy))

        # --- Rendering ---
        screen.fill(BLACK)
        
        for wall in walls:
            p1 = to_screen(wall.start[0], wall.start[1])
            p2 = to_screen(wall.end[0], wall.end[1])
            pygame.draw.line(screen, WHITE, p1, p2, 3)

        rob_pos = to_screen(robot.x, robot.y)
        
        # Draw the accumulated point cloud
        for hx, hy in point_cloud:
            pygame.draw.circle(screen, RED, to_screen(hx, hy), 2)

        # Draw Robot
        pygame.draw.circle(screen, GREEN, rob_pos, int(0.2 * SCALE))
        heading_x = robot.x + 0.3 * math.cos(robot.theta)
        heading_y = robot.y + 0.3 * math.sin(robot.theta)
        pygame.draw.line(screen, WHITE, rob_pos, to_screen(heading_x, heading_y), 3)

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()