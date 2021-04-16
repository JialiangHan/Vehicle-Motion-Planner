import Map
import Node_2D
import Grid_map
import matplotlib.pyplot as plt

# size: list, a list which show max range of map, [min_x, max_x, min_y, max_y],all units are meter
# n: number of obstacle
grid_size = 20
size = [0, grid_size, 0, grid_size]
n = 3
# generate map
map = Map.Map(size, n)
initial = Node_2D.Node_2D(size[0] + 5, size[2] + 5)
goal = Node_2D.Node_2D(size[1] - 5, size[3] - 5)

grid_map = Grid_map.Grid_map(map)
grid_map.plot_grid_map()

# offline calculate constrained heuristic
#
# generate a grid map with obstacle
#
# use hybrid_A_star()
#
# path smoothing
#
# plot
