from software_emulation.differential_drive_emulator import DifferentialDriveEmulator
import math
import numpy 
from scipy.optimize import minimize


class ModelPredictiveController:

    def __init__(self, dt = 0.1, time_horizon = 10):
        self.current_x = 0
        self.current_y = 0
        self.current_heading = 0
        self.diff_drive = DifferentialDriveEmulator()
        self.dt = dt
        self.Qd = 10
        self.Rv = 10
        self.Rw = 10
        self.Rh = 10
        self.Rl = 10
        self.time_horizon = time_horizon
        self.robot_radius_sq = 0.1
        
        self.point_cloud = None

    def predict_next_state(self, current_x, current_y, current_heading, vel, ang_vel):
        
        x_1 = current_x + vel * math.cos(current_heading) * self.dt
        y_1 = current_y + vel * math.sin(current_heading) * self.dt
        theta_1 = current_heading + ang_vel * self.dt

        states = numpy.array([x_1, y_1, theta_1])

        return states

    def cost_function(self, control_sequence, target_x, target_y):

        ghost_x = 0
        ghost_y = 0
        ghost_heading = 0

        total_cost = 0

        pc_array = numpy.array(self.point_cloud)

        for step in range(self.time_horizon):

            vel = control_sequence[step*2]
            ang_vel = control_sequence[step*2+1]

            states = self.predict_next_state(ghost_x, ghost_y, ghost_heading, vel, ang_vel)
            ghost_x, ghost_y, ghost_heading = states

            total_cost += self.Qd * ((target_x - ghost_x)**2 + (target_y - ghost_y)**2)

            total_cost += self.Rv * (vel**2) + self.Rw*(ang_vel**2)

            total_cost += self.Rh * (math.atan2(target_y, target_x) - ghost_heading)**2 

            

            if len(pc_array) > 0:
                dx = self.point_cloud[:, 0] - ghost_x
                dy = self.point_cloud[:, 1] - ghost_y

                sq_distances = dx**2 + dy**2

                min_sq_distance = numpy.min(sq_distances)

                if min_sq_distance < self.robot_radius_sq:
                    total_cost += 100000
                else:
                    total_cost += (0.5 / math.sqrt(min_sq_distance))*self.Rl



        return total_cost

    def optimizer(self, target_x, target_y):
        
        initial_guess = numpy.ones(self.time_horizon * 2) * 0.1

        motor_bounds = [(0.0, 1), (-1.5, 1.5)] * self.time_horizon

        result = minimize(
            fun=self.cost_function,      
            x0=initial_guess,            
            args=(target_x, target_y),   
            method='SLSQP',              
            bounds=motor_bounds,         
            options={'maxiter': 10}     
        )

        best_sequence = result.x 

        best_vel = best_sequence[0]
        best_ang_vel = best_sequence[1]

        return best_vel, best_ang_vel

    def set_lidar_points(self, lidar_points):
        self.point_cloud = numpy.array(lidar_points)
