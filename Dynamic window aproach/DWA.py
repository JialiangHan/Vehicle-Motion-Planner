"""
this is a python code for dynamic window approach.

model used here is bicycle model
"""
import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.abspath(__file__))
                + "/../Support functions")

import Vehicle_model
import Node_3D, Map, Config
import distance
import Node


class dynamic_window_approach:
    def __init__(self, start: Node_3D, goal: Node_3D, map: Map, config: Config.Config):
        self.start = start
        self.goal = goal
        self.map = map
        self.config = config
        self.path = []

    def run(self):
        # state is initial state, should equal to self.start
        state = np.array([self.start.x, self.start.y, self.start.theta, 0, 0])
        trajectory = np.array(state)
        while True:
            dw = self.get_dynamic_window(state)
            u, predicted_trajectory = self.get_trajectory(state, dw)
            motion = Vehicle_model.KinematicModel(state[0], state[1], state[2], u[0])
            state = motion.update_state(u[0], u[1], self.config.dt)
            self.path = np.vstack((trajectory, state))
            # plotting
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(state[0], state[1], "xr")
            plt.plot(self.goal.x, self.goal.y, "xb")
            self.map.Plot()
            Vehicle_model.plot_car(state[0], state[1], state[2])
            Vehicle_model.plot_arrow(state[0], state[1], state[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)
            # goal check
            distance_to_goal = math.hypot(state[0] - self.goal.x, state[1] - self.goal.y)
            delta_angle = state[2] - self.goal.theta
            if distance_to_goal < Vehicle_model.VEHICLE_RADIUS and delta_angle < math.radians(5):
                print("Goal!!")
                break

    def get_trajectory(self, state, dw):
        best_u = [0, 0]
        best_trajectory = np.array(state)
        min_cost = float("inf")
        for velocity in np.arange(dw[0], dw[1], self.config.v_resolution):
            for steering_angle in np.arange(dw[2], dw[3], self.config.steering_angle_resolution):
                trajectory = self.predict_trajectory(state, velocity, steering_angle)
                cost = self.calculate_total_cost(trajectory)
                if cost < min_cost:
                    min_cost = cost
                    best_u = [velocity, steering_angle]
                    best_trajectory = trajectory
        return best_u, best_trajectory

    def predict_trajectory(self, state, velocity, steering_angle):
        trajectory = np.array(state)
        time = 0
        while time < self.config.predict_time:
            motion = Vehicle_model.KinematicModel(state[0], state[1], state[2], velocity)
            state = motion.update_state(velocity, steering_angle, self.config.dt)
            trajectory = np.vstack((trajectory, state))
            time = time + self.config.dt
        return trajectory

    def get_dynamic_window(self, state: list) -> list:
        # state = [x,y,heading angle, velocity, steering angle]
        # use bicycle model, dynamic window should be [velocity, steering angle]
        Vs = [self.config.min_speed, self.config.max_speed,
              -self.config.max_steering_angle, self.config.max_steering_angle]

        Vd = [state[3] - self.config.max_accel * self.config.dt,
              state[3] + self.config.max_accel * self.config.dt,
              state[4] - self.config.max_steering_angle_rate * self.config.dt,
              state[4] + self.config.max_steering_angle_rate * self.config.dt]

        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]), max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw

    def calculate_total_cost(self, trajectory):
        to_goal_cost = self.to_goal_cost(trajectory)
        obs_cost = self.obs_cost(trajectory)
        velocity_cost = self.velocity_cost(trajectory)
        total_cost = self.config.obstacle_cost_gain * obs_cost + self.config.speed_cost_gain * velocity_cost + \
                     self.config.to_goal_cost_gain * to_goal_cost
        return total_cost

    def to_goal_cost(self, trajectory):
        """
        calculate angle diff between current position and goal
        """
        angle_diff = self.goal.theta - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(angle_diff), math.cos(angle_diff)))
        return cost

    def obs_cost(self, trajectory):
        """
        calculate obstacle cost, distance to obstacle, inf: collision
        """
        min_dist = float("inf")
        for node in trajectory:
            dist, _ = distance.distance_node_to_polygons(Node.Node(node[0], node[1]), self.map.obstacle_list)
            if dist <= Vehicle_model.VEHICLE_RADIUS:
                dist = float("inf")
                return dist
            elif dist < min_dist:
                min_dist = dist
        return 1 / dist

    def velocity_cost(self, trajectory):
        # make vehicle move fast as it can
        cost = self.config.max_speed - trajectory[-1, 3]
        return cost

    def plot_path(self):
        plt.plot(self.path[:, 0], self.path[:, 1], "-r")
        plt.pause(0.0001)
