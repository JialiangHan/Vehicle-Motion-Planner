import math

import matplotlib.pyplot as plt

import Map
import Node_3D
from Hybrid_A_star import Hybrid_A_star

# size: list, a list which show max range of map, [min_x, max_x, min_y, max_y],all units are meter
# n: number of obstacle
grid_size = 20
grid_resolution = 1
size = [0, grid_size, 0, grid_size]
n = 1
# generate map
map = Map.Map(size, n)
initial = Node_3D.Node_3D(size[0], size[2], math.radians(0))
goal = Node_3D.Node_3D(size[1] - 1, size[1] - 1, math.radians(45))
angle_resolution = math.radians(5)
map.Plot()
# plt.show()
hybrid_a_star = Hybrid_A_star(initial, goal, map, grid_resolution, angle_resolution)
hybrid_a_star.run()
path_x, path_y, path_theta = hybrid_a_star.get_path()
# hybrid_a_star.plot_path()
hybrid_a_star.plot_vehicle()
plt.show()
