import math
import matplotlib.pyplot as plt
import Map
import Node_3D
from Hybrid_A_star import Hybrid_A_star
from path_smoother import Path_smoother
from path_evaluation import path_evaluation

def main():
    # size: list, a list which show max range of map, [min_x, max_x, min_y, max_y],all units are meter
    # n: number of obstacle
    grid_size = 20
    size = [0, grid_size, 0, grid_size]
    n = 1
    # generate map
    map = Map.Map(size, n)

    initial = Node_3D.Node_3D(size[0], size[2], math.radians(0))
    goal = Node_3D.Node_3D(size[1] - 1, size[1] - 1, math.radians(45))
    angle_resolution = math.radians(5)
    max_iterations = 100
    grid_resolution = [1]
    plt.subplot(221)
    for i in range(len(grid_resolution)):
        hybrid_a_star = Hybrid_A_star(initial, goal, map, grid_resolution[i], angle_resolution)
        hybrid_a_star.run()
        hybrid_a_star.plot_path()
        # path_evaluation(hybrid_a_star.path, hybrid_a_star.map)
        smooth = Path_smoother(hybrid_a_star.path, hybrid_a_star.map, max_iterations)
        smooth.plot()
    # path_evaluation(smooth.smoothed_path,map)
    plt.show()


if __name__ == '__main__':
    main()
