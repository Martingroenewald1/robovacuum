import math
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Import your classes from your file (assuming it's named lidar_sim.py)
from rplidar_emulator import RPLidar, Wall

# 1. Setup the Environment and Sensor
env_walls = [
    Wall((5,5), (-5,5)),
    Wall((-5,5), (-5,-5)),
    Wall((-5,-5), (5,-5)),
    Wall((5,-5), (5,5)),
    Wall((1,-1), (1,1)),
    Wall((0,0), (0,1)),
    Wall((1,0), (-1,0))
]
lidar = RPLidar(env_walls)

# 2. Setup Robot State
robot_x = 0.0
robot_y = 0.0
robot_theta = 0.0

# We will keep a rolling buffer of the last 500 points (one full 360 rotation)
history_x = [0.0] * 500
history_y = [0.0] * 500
buffer_index = 0

# 3. Setup the Matplotlib Plot
fig, ax = plt.subplots(figsize=(8, 8))
ax.set_xlim(-15, 15)
ax.set_ylim(-15, 15)
ax.set_title("Live LiDAR C1 Emulator (Full 360° Sweep)")
ax.grid(True)

# Draw the walls on the plot
for wall in env_walls:
    ax.plot([wall.start[0], wall.end[0]], [wall.start[1], wall.end[1]], 'b-', linewidth=2)

# Create empty scatter plot objects for the points and the robot
scatter_points, = ax.plot([], [], 'r.', markersize=4)
robot_marker, = ax.plot([], [], 'go', markersize=8)

def update(frame):
    global robot_x, robot_y, robot_theta, buffer_index
    
    # Let's make the robot slowly drive in a circle to see the physics working
    robot_theta = (robot_theta + 2.0) % 360.0
    robot_x = 3.0 * math.cos(math.radians(robot_theta))
    robot_y = 3.0 * math.sin(math.radians(robot_theta))

    # Grab a full 360-degree sweep (13 cabins = 520 points) all at once
    for _ in range(13):
        start_angle, cabin_data = lidar.get_next_cabin(robot_x, robot_y, robot_theta)

        # Convert the 40 distances into global X, Y coordinates
        for i in range(40):
            point_angle = start_angle + (i * lidar.resolution)
            global_ray_angle = (robot_theta + point_angle) % 360.0
            angle_rad = math.radians(global_ray_angle)
            
            dist = cabin_data[i]
            
            # If it hit nothing (returned 12.0), hide the point
            if dist < 12.0:
                hit_x = robot_x + (dist * math.cos(angle_rad))
                hit_y = robot_y + (dist * math.sin(angle_rad))
            else:
                hit_x = float('nan')
                hit_y = float('nan')

            history_x[buffer_index] = hit_x
            history_y[buffer_index] = hit_y
            buffer_index = (buffer_index + 1) % 500

    # Update the plot with the new data
    scatter_points.set_data(history_x, history_y)
    robot_marker.set_data([robot_x], [robot_y])
    
    return scatter_points, robot_marker

# 4. Run the Animation Loop
# interval=100 means it calls update() every 100ms (10Hz scan rate)
ani = animation.FuncAnimation(fig, update, frames=200, interval=100, blit=True)

plt.show()