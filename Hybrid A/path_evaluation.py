import matplotlib.pyplot as plt
import math
import numpy as np

import Map
import Node
import distance


def path_evaluation(path, map:Map.Map):
    # following part are path evaluation: plot distance to obstacle, curvature
    distance_to_obstacle = path_distance_to_obstacle(path, map)
    curvature = path_curvature(path)
    plt.subplot(222)
    plt.plot(path[0], distance_to_obstacle, label="distance to obstacle")
    plt.legend(loc="upper left")
    plt.grid(True)
    plt.subplot(223)
    plt.plot(path[0], curvature, label="curvature")
    plt.legend(loc="upper left")
    plt.grid(True)
    plt.show()


def path_distance_to_obstacle(path, map):
    distance_to_obstacle = []
    for i in range(len(path[0])):
        node = np.array([[path[0][i], path[1][i]]])
        node = Node.Node(node[0][0], node[0][1])
        d, _ = distance.distance_node_to_polygons(node, map.obstacle_list)
        distance_to_obstacle.append(d)
    return distance_to_obstacle


def path_curvature(path):
    curvature = []
    first_curvature = 0
    last_curvature = 0
    curvature.append(first_curvature)
    for i in range(1, len(path[0]) - 1):
        xi = np.array([[path[0][i], path[1][i]]])
        xp = np.array([[path[0][i - 1], path[1][i - 1]]])
        xs = np.array([[path[0][i + 1], path[1][i + 1]]])
        Dxi = xi - xp
        Dxs = xs - xi
        Dxi_length = np.linalg.norm(Dxi)
        Dxs_length = np.linalg.norm(Dxs)
        a = np.dot(Dxs, Dxi.T) / Dxi_length / Dxs_length
        if a > 1:
            a = round(a[0][0])
        Delta_phi = math.acos(a)
        curvature.append(Delta_phi / Dxi_length)
    curvature.append(last_curvature)
    return curvature
