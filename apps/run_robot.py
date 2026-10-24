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

# --- IMPORT YOUR CUSTOM MPC ---
from MPC import ModelPredictiveController

# --- Constants & Colors ---
WIDTH, HEIGHT = 800, 600
SCALE = 50.0  
FPS = 60

BLACK   = (20, 20, 20)
WHITE   = (240, 240, 240)
CYAN    = (0, 255, 255)
GREEN   = (50, 255, 50)
YELLOW  = (255, 255, 0)
MAGENTA = (255, 0, 255) 

def to_screen(x, y):
    return int(WIDTH / 2.0 + (x * SCALE)), int(HEIGHT / 2.0 - (y * SCALE))

def to_world(sx, sy):
    return (sx - WIDTH / 2.0) / SCALE, (HEIGHT / 2.0 - sy) / SCALE

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
    pygame.display.set_caption("My First Custom MPC")
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
    
    # Initialize your imported custom MPC
    mpc = ModelPredictiveController(dt=0.1, time_horizon=10)
    
    target_x, target_y = 0.0, 0.0
    point_cloud = deque(maxlen=500)
    predicted_path = [] # Empty for now since mpc.py doesn't return the path coordinates
    
    running = True
    while running:
        dt = clock.tick(FPS) / 1000.0 

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = pygame.mouse.get_pos()
                target_x, target_y = to_world(mx, my)

        # --- FRAME TRANSFORMATION ---
        # Your MPC assumes the robot is at (0,0) pointing straight ahead (theta=0).
        # We must convert the global target (mouse click) into the robot's local frame.
        dx = target_x - robot.x
        dy = target_y - robot.y
        
        # 2D Rotation Matrix to rotate the target into the robot's perspective
        local_target_x = dx * math.cos(-robot.theta) - dy * math.sin(-robot.theta)
        local_target_y = dx * math.sin(-robot.theta) + dy * math.cos(-robot.theta)

        # --- CALL YOUR MPC ---
        req_v, req_w = mpc.optimizer(local_target_x, local_target_y)

        robot.motors.set_targets(req_v, req_w)
        robot.update(dt)

        # --- LiDAR Extraction ---
        theta_deg = math.degrees(robot.theta)
        start_angle, cabin_data = robot.lidar.get_next_cabin(robot.x, robot.y, theta_deg)
        
        for i, dist in enumerate(cabin_data):
            if dist < 12.0:
                ray_angle_deg = start_angle + (i * 0.72)
                global_angle_rad = robot.theta + math.radians(ray_angle_deg)
                
                hx = robot.x + (dist * math.cos(global_angle_rad))
                hy = robot.y + (dist * math.sin(global_angle_rad))
                point_cloud.append((hx, hy))

        # --- Rendering ---
        screen.fill(BLACK)
        
        for wall in walls:
            p1 = to_screen(wall.start[0], wall.start[1])
            p2 = to_screen(wall.end[0], wall.end[1])
            pygame.draw.line(screen, WHITE, p1, p2, 3)

        pygame.draw.circle(screen, YELLOW, to_screen(target_x, target_y), 6)
        
        for hx, hy in point_cloud:
            pygame.draw.circle(screen, CYAN, to_screen(hx, hy), 2)

        rob_pos = to_screen(robot.x, robot.y)
        pygame.draw.circle(screen, GREEN, rob_pos, int(0.2 * SCALE))
        heading_x = robot.x + 0.3 * math.cos(robot.theta)
        heading_y = robot.y + 0.3 * math.sin(robot.theta)
        pygame.draw.line(screen, WHITE, rob_pos, to_screen(heading_x, heading_y), 3)

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()