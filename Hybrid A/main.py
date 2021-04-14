import Map
from .Node_2D import *

# size: list, a list which show max range of map, [min_x, max_x, min_y, max_y],all units are meter
# n: number of obstacle
size = [0, 100, 0, 100]
n = 1
# generate map
map = Map.Map(size, n)
initial = Node_2D(size[0]+5, size[2]+5)
goal = Node_2D(size[1]-5, size[3]-5)

offline calculate constrained heuristic

generate a grid map with obstacle

use hybrid_A_star()

path smoothing

plot

