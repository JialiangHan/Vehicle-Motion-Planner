"""
this is a simple bicycle model

author: Jialiang Han
"""
import math
import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial.transform import Rotation as Rot

# basic vehicle geometry data
WHEEL_BASE = 2709e-3  # meter
WIDTH_OF_CAR = 1820e-3  # meter
DISTANCE_REAR_TO_FRONT = 4261e-3  # meter
DISTANCE_REAR_TO_END = 1551e-3  # meter
MAX_STEER = 33.5  # deg

DISTANCE_REAR_TO_CENTER = (DISTANCE_REAR_TO_FRONT - DISTANCE_REAR_TO_END) / 2  # meter
VEHICLE_RADIUS = math.sqrt((DISTANCE_REAR_TO_END + DISTANCE_REAR_TO_FRONT) / 2 ** 2 + WIDTH_OF_CAR / 2 ** 2)

# vehicle rectangle refer to rear axis center[Left upper, left lower,right lower,right upper,left upper]
VEHICLE_RECTANGLE_X = [DISTANCE_REAR_TO_FRONT, DISTANCE_REAR_TO_FRONT, -DISTANCE_REAR_TO_END, -DISTANCE_REAR_TO_END,
                       DISTANCE_REAR_TO_FRONT]
VEHICLE_RECTANGLE_Y = [WIDTH_OF_CAR / 2, -WIDTH_OF_CAR / 2, -WIDTH_OF_CAR / 2, WIDTH_OF_CAR / 2, WIDTH_OF_CAR / 2]


def vehicle_collision_check(x_list, y_list, yaw_list, ox, oy, kd_tree) -> bool:
    """
    x_list,y_list, yaw_list are path which are exzimined for collision
    ox,oy , kd_tree are map/obstacle info
    """
    for x, y, yaw in zip(x_list, y_list, yaw_list):
        # get vehicle center coordinate
        vehicle_center_x = x + DISTANCE_REAR_TO_CENTER * math.cos(yaw)
        vehicle_center_y = y + DISTANCE_REAR_TO_CENTER * math.sin(yaw)
        # get all points index within vehicle_radius
        indices = kd_tree.query_ball_point([vehicle_center_x, vehicle_center_y], VEHICLE_RADIUS *1.1)
        if not indices:
            continue
        else:
            if not rectangle_collision_check(x, y, yaw, [ox[i] for i in indices], [oy[i] for i in indices]):
                return False  # collision
    return True  # no collision


def rectangle_collision_check(x, y, yaw, ox, oy) -> bool:
    # transfer obstacles to vehicle frame
    rot = Rot.from_euler('z', yaw).as_matrix()[0:2, 0:2]
    for ix, iy in zip(ox, oy):
        delta_x = ix - x
        delta_y = iy - y
        rotated_xy = np.stack([delta_x, delta_y]).T @ rot
        new_x, new_y = rotated_xy[0], rotated_xy[1]
        if -DISTANCE_REAR_TO_END * 1.1 < new_x < DISTANCE_REAR_TO_FRONT * 1.1 and -1.1 * WIDTH_OF_CAR / 2 < new_y < 1.1 * WIDTH_OF_CAR / 2:
            return False  # collision
    return True  # no collision


def move(x: float, y: float, yaw: float, distance: float, steer: float) -> float:
    # here distance=velocity * delta time, for delta time we assume this is a constant
    # unit for angle is all degree, for length, unit are meter
    x = x + distance * math.cos(yaw)
    y = y + distance * math.sin(yaw)
    yaw = yaw + distance * math.tan(steer) / WHEEL_BASE
    return x, y, yaw


def plot_arrow(x, y, yaw, length=1.0, width=0.5, fc="r", ec="k"):
    """Plot arrow."""
    if not isinstance(x, float):
        for (i_x, i_y, i_yaw) in zip(x, y, yaw):
            plot_arrow(i_x, i_y, i_yaw)
    else:
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  fc=fc, ec=ec, head_width=width, head_length=width, alpha=0.4)


def plot_car(x, y, yaw):
    car_color = '-k'
    c, s = math.cos(yaw), math.sin(yaw)
    rot = Rot.from_euler('z', -yaw).as_matrix()[0:2, 0:2]
    car_outline_x, car_outline_y = [], []
    for rx, ry in zip(VEHICLE_RECTANGLE_X, VEHICLE_RECTANGLE_Y):
        converted_xy = np.stack([rx, ry]).T @ rot
        car_outline_x.append(converted_xy[0] + x)
        car_outline_y.append(converted_xy[1] + y)
    arrow_x, arrow_y, arrow_yaw = c * 1.5 + x, s * 1.5 + y, yaw
    plot_arrow(arrow_x, arrow_y, arrow_yaw)

    plt.plot(car_outline_x, car_outline_y, car_color)
    plt.axis('equal')

def main():
    x, y, yaw = 0., 1, math.pi / 4

    plot_car(x, y, yaw)
    plt.show()


if __name__ == '__main__':
    main()
