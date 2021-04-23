"""
this is a path smoother class for path got from hybrid A star.py
function should include:
objective function P = Pobs+Pcurvature+Psmooth+Pvoronoi
gradient descent
"""
import Vector
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
Curvature_max = 6


class Path_smoother:
    def __init__(self, path, map, max_iteration):
        self.path = path
        self.map = map
        self.smoothed_path = self.path
        self.max_iterations = max_iteration
        self.gradient_descent()

    def gradient_descent(self):
        alpha = 0.1
        iteration = 0
        total_weight = Weight_obstacle + Weight_curvature  # +Weight_smoothness+Weight_voronoi
        path=self.path
        smooth_path = {}
        while iteration < self.max_iterations:
            for i in range(1, len(path[0]) - 2):
                gradient = 0
                xi = np.array([[path[0][i], path[1][i]]])
                xp = np.array([[path[0][i - 1], path[1][i - 1]]])
                xs = np.array([[path[0][i + 1], path[1][i + 1]]])
                gradient = gradient - self.obstacle_term(xi)
                gradient = gradient - self.curvature_term(xp, xi, xs)
                # gradient=gradient-self.smooth_term()
                # gradient=gradient-self.voronoi_term()
                xi = xi + alpha * gradient / total_weight
                path[0][i] = xi[0][0]
                path[1][i] = xi[0][1]
                path[2][i] = math.atan2(xi[0][1] - xp[0][1], xi[0][0] - xp[0][0])
            smooth_path[iteration]=path
            iteration = iteration + 1
        self.smoothed_path=path
        # for key,value in smooth_path.items():
        #     plt.plot(value[0],value[1],label=key)
        #     plt.legend(loc="upper left")
        #     plt.grid(True)
        # plt.show()


    def obstacle_term(self, node: np.ndarray):
        # find distance to closest obstacle
        node = Node.Node(node[0][0], node[0][1])
        distance_to_closest_obstacle, closet_node_on_obstacle = \
            distance.distance_node_to_polygons(node, self.map.obstacle_list)
        dx = node.x - closet_node_on_obstacle.x
        dy = node.y - closet_node_on_obstacle.y
        obstacle_vector = np.array([[dx, dy]])
        if distance_to_closest_obstacle <= Dmax:
            gradient = Weight_obstacle * 2 * (
                    distance_to_closest_obstacle - Dmax) * obstacle_vector / distance_to_closest_obstacle
        else:
            gradient = np.array([[0, 0]])
        return gradient

    @staticmethod
    def curvature_term(xp, xi, xs):
        Dxi = xi - xp
        Dxs = xs - xi
        Dxi_length = np.linalg.norm(Dxi)
        Dxs_length = np.linalg.norm(Dxs)
        a = np.dot( Dxs,Dxi.T) / Dxi_length / Dxs_length
        if a>1:
            a=round(a[0][0])
        Delta_phi = math.acos(a)
        curvature_xi = Delta_phi / Dxi_length
        if curvature_xi <= Curvature_max:
            gradient = 0
        else:
            p1 = Vector.orthogonal_complements(xi, -xp) / np.linalg.norm(xi) / np.linalg.norm(xp)
            p2 = Vector.orthogonal_complements(-xp, xi) / np.linalg.norm(xi) / np.linalg.norm(xp)
            inverse = -1 / Dxi_length
            P_delta_phi_P_cos_delta_phi = -1 / math.sqrt(1 - math.cos(Delta_phi) ** 2)
            first_term = inverse * P_delta_phi_P_cos_delta_phi
            last_term = Delta_phi / Dxi_length ** 2 * np.array([1, 1])
            ki = first_term * (-p1 - p2) - last_term
            # kp = first_term * p2 - last_term
            # ks = first_term * p1
            gradient = ki
        return gradient

    # def smooth_term(self):
    #
    # def voronoi_term(self):
    def plot(self):
        path_x, path_y, path_theta = self.smoothed_path[0], self.smoothed_path[1], self.smoothed_path[2]
        plt.plot(path_x, path_y, label=self.max_iterations)
        plt.legend(loc="upper left")
        plt.grid(True)
