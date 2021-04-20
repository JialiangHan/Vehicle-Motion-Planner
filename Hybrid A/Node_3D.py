"""
this class is for 3D(x,y,theta)
"""
import Angle
import distance
import math

WEIGHT_TURNING = 1.05
WEIGHT_CHANGE_OF_DIRECTION = 2
WEIGHT_REVERSE = 2
R = 6  # [m] turning radius
steering_angle = 6.75  # [deg]
d_theta = math.radians(steering_angle)
dx = R * math.sin(d_theta)
dy = R * (1 - math.cos(d_theta))
delta_theta = [0, d_theta, -d_theta]
delta_x = [R * d_theta, dx, dx]
delta_y = [0, dy, -dy]


class Node_3D:
    def __init__(self, x: float, y: float, theta: float, cost_so_far=None, heuristic=None, predecessor=None,
                 movement_index=None):
        self.x = x
        self.y = y
        self.theta = theta
        if cost_so_far is not None:
            self.cost_so_far = cost_so_far
        else:
            self.cost_so_far = 0
        self.heuristic = heuristic
        if predecessor is None:
            self.Predecessor = self
        else:
            self.Predecessor = predecessor
        self.in_open_list = False
        self.in_close_list = False
        self.discover = False
        self.index = -1
        self.f_value = 0
        self.movement_index = movement_index

    def get_f_value(self):
        self.f_value = self.cost_so_far + self.heuristic

    def is_in_map(self, map) -> bool:
        if map.size[0] <= self.x < map.size[1] and map.size[2] <= self.y < map.size[3]:
            return True
        else:
            return False

    def update_cost_so_far(self):
        if self.movement_index < 3:  # forward
            if self.movement_index != self.Predecessor.movement_index:  # turning
                if self.Predecessor.movement_index > 2:  # change of direction: reverse to forward
                    movement_cost = delta_x[0] * WEIGHT_CHANGE_OF_DIRECTION * WEIGHT_TURNING
                else:
                    movement_cost = delta_x[0] * WEIGHT_TURNING
            else:
                movement_cost = delta_x[0]
        else:  # reverse
            if self.movement_index != self.Predecessor.movement_index:  # turning
                if self.Predecessor.movement_index < 3:  # change of direction: reverse to forward
                    movement_cost = delta_x[0] * WEIGHT_CHANGE_OF_DIRECTION * WEIGHT_TURNING * WEIGHT_REVERSE
                else:
                    movement_cost = delta_x[0] * WEIGHT_TURNING * WEIGHT_REVERSE
            else:
                movement_cost = delta_x[0] * WEIGHT_REVERSE
        self.cost_so_far = self.cost_so_far + movement_cost
        self.discover = True

    def __gt__(self, other):
        return self.f_value > other.f_value

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y and self.theta == other.theta:
            return True
        else:
            return False

    def create_successor(self):
        successors = []
        for i in range(6):
            if i < 3:  # forward
                x = self.x + delta_x[i] * math.cos(self.theta) - delta_y * math.sin(self.theta)
                y = self.y + delta_x[i] * math.sin(self.theta) + delta_y * math.cos(self.theta)
                theta = Angle.convert_into_2pi(self.theta + delta_theta[i])
            else:  # backward
                x = self.x - delta_x[i - 3] * math.cos(self.theta) - delta_y[i - 3] * math.sin(self.theta)
                y = self.y - delta_x[i - 3] * math.sin(self.theta) + delta_y[i - 3] * math.cos(self.theta)
                theta = Angle.convert_into_2pi(self.theta - delta_theta[i - 3])
            successors.append(Node_3D(x, y, theta, self.cost_so_far, None, self, i))
        return successors

    def __str__(self):
        return "x: " + str(self.x)[:4] + ", y: " + str(self.y)[:4] + ", theta: " + str(self.theta)[:4]
