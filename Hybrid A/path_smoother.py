"""
this is a path smoother class for path got from hybrid A star.py
function should include:
objective function P = Pobs+Pcurvature+Psmooth+Pvoronoi
gradient descent
"""
import distance
import Node
import matplotlib.pyplot as plt
import numpy as np
import math

Dmax = 2  # [m]
Weight_obstacle = 0.25
Weight_curvature = 0.25
Weight_smoothness = 0.25
Weight_voronoi = 0.25


class Path_smoother:
    def __init__(self, path, map):
        self.path = path
        self.map = map
        self.smoothed_path = self.path
        self.gradient_descent()

    def gradient_descent(self):
        max_iterations = 10
        alpha = 0.1
        iteration = 0
        gradient = np.array([0, 0])
        while iteration < max_iterations:
            for i in range(1, len(self.path[0]) - 2):
                xi = np.array([self.path[0][i], self.path[1][i]])
                gradient = gradient - self.obstacle_term(xi)
                # gradient=gradient-self.curvature_term()
                # gradient=gradient-self.smooth_term()
                # gradient=gradient-self.voronoi_term()
                xi = xi + alpha * gradient
                self.smoothed_path[0][i] = xi[0]
                self.smoothed_path[1][i] = xi[1]
                self.smoothed_path[2][i] = math.atan2(xi[1] - self.smoothed_path[1][i - 1],
                                                      xi[0] - self.smoothed_path[0][i - 1])
            iteration = iteration + 1

    def obstacle_term(self, node: np.ndarray):
        # find distance to closest obstacle
        node = Node.Node(node[0], node[1])
        distance_to_closest_obstacle, closet_node_on_obstacle = distance.distance_node_to_polygons(node,
                                                                                                   self.map.obstacle_list)
        dx = node.x - closet_node_on_obstacle.x
        dy = node.y - closet_node_on_obstacle.y
        obstacle_vector = np.array([dx, dy])
        if distance_to_closest_obstacle <= Dmax:
            gradient = Weight_obstacle * 2 * (
                    distance_to_closest_obstacle - Dmax) * obstacle_vector / distance_to_closest_obstacle
        else:
            gradient = np.array([0, 0])
        return gradient

    # def curvature_term(self):
    #
    # def smooth_term(self):
    #
    # def voronoi_term(self):
    def plot(self):
        path_x, path_y, path_theta = self.smoothed_path[0], self.smoothed_path[1], self.smoothed_path[2]
        plt.plot(path_x, path_y, label="smoothed")
        plt.legend(loc="upper left")
        plt.grid()
