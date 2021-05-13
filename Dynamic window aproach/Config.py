import math


class Config:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_steering_angle = math.radians(8)  # [rad]
        self.max_accel = 0.2  # [m/ss]
        self.max_steering_angle_rate = math.radians(5)  # [rad/s]
        self.v_resolution = 0.01  # [m/s]
        self.steering_angle_resolution = math.radians(0.1)  # [rad]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
