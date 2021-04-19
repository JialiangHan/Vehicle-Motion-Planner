import matplotlib.pyplot as plt

import Map
import Node_2D
import Grid_map
from A_star import A_star

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
grid_map = Grid_map.Grid_map(map,grid_resolution)
# grid_map.plot_grid_map()
# plt.show()
a_star = A_star(initial, goal, map,grid_resolution)
a_star.run()
path_x, path_y = a_star.get_path()
a_star.plot()
plt.show()
# offline calculate constrained heuristic
#
# generate a grid map with obstacle
#
# use hybrid_A_star()
#
# path smoothing
#
# plot
