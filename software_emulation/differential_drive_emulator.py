import random

class DifferentialDriveEmulator:
    
    def __init__(self):
        self.requested_v = 0.0
        self.requested_w = 0.0
        self.actual_v = 0.0
        self.actual_w = 0.0
        self.track_width = 100
        self.wheel_radius = 0.065
        self.w_motor_R = 0.0
        self.w_motor_L = 0.0
        self.smoothing_alpha = 1
        self.std_dev_v = 0.01
        self.std_dev_w = 0.01

    def set_targets(self, requested_v, requested_w):
        self.requested_v = requested_v
        self.requested_w = requested_w

    def step(self) -> tuple[float, float]:

        left_motor_v_requested = self.requested_v - (self.requested_w * self.track_width/2)
        right_motor_v_requested = self.requested_v + (self.requested_w * self.track_width/2)

        left_motor_w_requested = left_motor_v_requested / self.wheel_radius
        right_motor_w_requested = right_motor_v_requested / self.wheel_radius

        self.w_motor_R = self.w_motor_R + self.smoothing_alpha*(right_motor_w_requested - self.w_motor_R)
        self.w_motor_L = self.w_motor_L + self.smoothing_alpha*(left_motor_w_requested - self.w_motor_L)

        right_actual_v = self.w_motor_R * self.wheel_radius
        left_actual_v = self.w_motor_L * self.wheel_radius

        self.actual_v = (right_actual_v + left_actual_v) / 2.0
        self.actual_w = (right_actual_v - left_actual_v) / self.track_width

        measured_v = random.gauss(self.actual_v, self.std_dev_v)
        measured_w = random.gauss(self.actual_w, self.std_dev_w)

        return measured_v, measured_w
        


        