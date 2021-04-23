import math
import matplotlib.pyplot as plt
import Map
import Node_3D
import distance
from Hybrid_A_star import Hybrid_A_star
from path_smoother import Path_smoother
import numpy as np
import Node


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
    max_iterations = 20
    grid_resolution = [1]
    for i in range(len(grid_resolution)):
        hybrid_a_star = Hybrid_A_star(initial, goal, map, grid_resolution[i], angle_resolution)
        hybrid_a_star.run()
        hybrid_a_star.plot_path()
        smooth = Path_smoother(hybrid_a_star.path, hybrid_a_star.map, max_iterations)
        smooth.plot()
        # following part are path evalutation: plot distance to obstalce, curvature
        distance = []
        for i in range(len(smooth.smoothed_path[0])):
            node = np.array([[smooth.smoothed_path[0][i], smooth.smoothed_path[1][i]]])
            node = Node.Node(node[0][0], node[0][1])
            d, _ = distance.distance_node_to_polygons(node, smooth.map.obstacle_list)
            distance.append(d)
        curvature = []
        for i in range(1, len(smooth.smoothed_path[0]) - 2):
            xi = np.array([[smooth.smoothed_path[0][i], smooth.smoothed_path[1][i]]])
            xp = np.array([[smooth.smoothed_path[0][i - 1], smooth.smoothed_path[1][i - 1]]])
            xs = np.array([[smooth.smoothed_path[0][i + 1], smooth.smoothed_path[1][i + 1]]])
            Dxi = xi - xp
            Dxs = xs - xi
            Dxi_length = np.linalg.norm(Dxi)
            Dxs_length = np.linalg.norm(Dxs)
            a = np.dot(Dxs, Dxi.T) / Dxi_length / Dxs_length
            if a > 1:
                a = round(a[0][0])
            Delta_phi = math.acos(a)
            curvature.append( Delta_phi / Dxi_length)

    plt.show()


if __name__ == '__main__':
    main()
