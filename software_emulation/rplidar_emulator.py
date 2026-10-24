from software_emulation.wall import Wall
import math

class RPLidar:
    """Emulates an RPLIDAR C1 in dense express mode."""

    def __init__(self, environment_walls):
        self.scan_rate = 10 # Hz
        self.points_per_cabin = 40 
        self.resolution = 0.72 # degrees
        self.current_angle = 0.0
        self.walls = environment_walls
        self.robot_x = 0
        self.robot_y = 0
        self.robot_theta = 0
        self.local_walls = []

    def get_next_cabin(self, robot_x, robot_y, robot_theta) -> tuple[float, list[float]]:

        start_angle = self.current_angle
        cabin_data = []

        self.transform_walls_to_robot_frame(robot_x, robot_y, robot_theta)

        for i in range(500):
            angle_ray = start_angle + i * self.resolution
            cabin_data.append(self.get_ray_intercept_distance(angle_ray))

        self.current_angle = 0
        
        return start_angle, cabin_data

    def get_ray_intercept_distance(self, angle_ray) -> float:

        angle_rad = math.radians(angle_ray)

        ray_x = math.cos(angle_rad)
        ray_y = math.sin(angle_rad)

        closest_t = 12.0

        for wall in self.local_walls:
            x1, y1 = wall.start[0], wall.start[1]
            x2, y2 = wall.end[0], wall.end[1]

            wall_x = x2 - x1
            wall_y = y2 - y1

            D = (ray_x * wall_y) - (ray_y * wall_x)

            if D != 0:
                t = (x1 * wall_y - y1 * wall_x) / D
                u = (x1 * ray_y - y1 * ray_x) / D

                if (0 <= t <= closest_t) and (0.0 <= u <= 1):
                    closest_t = t

        return closest_t
            
        
    def transform_walls_to_robot_frame(self, robot_x, robot_y, robot_theta):

        current_local_walls = []
        robot_angle_rad = math.radians(robot_theta)

        for wall in self.walls:

            temp_x1 = (wall.start[0] - robot_x) 
            temp_y1 = (wall.start[1] - robot_y) 
            temp_x2 = (wall.end[0] - robot_x) 
            temp_y2 = (wall.end[1] - robot_y) 

            new_x1 = temp_x1 * math.cos(robot_angle_rad) + temp_y1 * math.sin(robot_angle_rad)
            new_y1 = -temp_x1 * math.sin(robot_angle_rad) + temp_y1 * math.cos(robot_angle_rad)          

            new_x2 = temp_x2 * math.cos(robot_angle_rad) + temp_y2 * math.sin(robot_angle_rad)
            new_y2 = -temp_x2 * math.sin(robot_angle_rad) + temp_y2 * math.cos(robot_angle_rad)       
    
            temp_wall = Wall((new_x1, new_y1), (new_x2, new_y2))
            
            current_local_walls.append(temp_wall)

        self.local_walls = current_local_walls



        

wall1 = Wall((10,10), (-10,10))
wall2 = Wall((-10,10), (-10,-10))
wall3 = Wall((-10,-10), (10,-10))
wall4 = Wall((10,-10), (10,10))

env_walls = [wall1, wall2, wall3, wall4]

lidar1 = RPLidar(env_walls)