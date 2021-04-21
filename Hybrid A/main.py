import math

import matplotlib.pyplot as plt

import Map
import Node_3D
from Hybrid_A_star import Hybrid_A_star

# size: list, a list which show max range of map, [min_x, max_x, min_y, max_y],all units are meter
# n: number of obstacle
grid_size = 20
# grid_resolution = 0.5
size = [0, grid_size, 0, grid_size]
n = 1
# generate map
map = Map.Map(size, n)
initial = Node_3D.Node_3D(size[0], size[2], math.radians(0))
goal = Node_3D.Node_3D(size[1] - 1, size[1] - 1, math.radians(45))
angle_resolution = math.radians(5)
# map.Plot()
# plt.show()
grid_resolution=[0.25,0.5,0.75,1,1.25,1.5,1.75,2]
for i in range(len(grid_resolution)):
    hybrid_a_star = Hybrid_A_star(initial, goal, map, grid_resolution[i], angle_resolution)
    hybrid_a_star.run()
    hybrid_a_star.plot_path()
# hybrid_a_star.plot_vehicle()
plt.show()
