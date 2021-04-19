import matplotlib.pyplot as plt

import Map
import Node_2D
from Hybrid_A_star import Hybrid_A_star

# size: list, a list which show max range of map, [min_x, max_x, min_y, max_y],all units are meter
# n: number of obstacle
grid_size = 3
grid_resolution = 0.5
size = [0, grid_size, 0, grid_size]
n = 1
# generate map
map = Map.Map(size, n)
initial = Node_2D.Node_2D(size[0], size[2])
goal = Node_2D.Node_2D(size[1] - 1, size[3] - 1)

hybrid_a_star = Hybrid_A_star(initial, goal, map,grid_resolution)
hybrid_a_star.run()
path_x, path_y = hybrid_a_star.get_path()
hybrid_a_star.plot_path()
hybrid_a_star.plot_vehicle()
plt.show()


