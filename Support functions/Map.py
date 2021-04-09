"""
this file contains a map class which contains some obstacle
"""
import random

import Edge
import Node
import Polygon
from Computational_Geometry import Convex_hull
from Plot import plot_Polygon


class Map:
    def __init__(self, size: list, n: int):
        """
        size:list, a list which show max range of map,[min_x,max_x,min_y,max_y]
        n: number of obstacle
        """
        self.size = size
        self.number_of_obstacle = n
        self.obstacle_list = []
        self.generate_obstacle()
        self.boundary = []
        self.generate_boundary()

    def generate_boundary(self):
        node_list = []
        edge_list = []
        for i in range(2):
            node_list.append(Node.Node(self.size[i], self.size[2]))
            node_list.append(Node.Node(self.size[i], self.size[3]))
        convexhull = Convex_hull(node_list)
        convexhull.run()
        for i in range(len(convexhull.hull) - 1):
            edge_list.append(Edge.Edge(convexhull.hull[i], convexhull.hull[i + 1]))
        self.boundary = Polygon.Polygon(edge_list)

    def generate_obstacle(self):
        number_of_nodes = 5  # number of vertex
        node_list = []
        edge_list = []
        delta_x = (self.size[1] - self.size[0]) / self.number_of_obstacle
        for i in range(self.number_of_obstacle):
            for j in range(number_of_nodes):
                # random.seed(j)
                x = random.uniform(self.size[0] + 1 + i * delta_x, self.size[0] - 1 + (i + 1) * delta_x)
                y = random.uniform(self.size[2] + 1, self.size[3] - 1)
                node_list.append(Node.Node(x, y))
            convexhull = Convex_hull(node_list)
            convexhull.run()
            for j in range(len(convexhull.hull) - 1):
                edge_list.append(Edge.Edge(convexhull.hull[j], convexhull.hull[j + 1]))
            polygon = Polygon.Polygon(edge_list)
            self.obstacle_list.append(polygon)
            node_list = []
            edge_list = []

    def Plot(self):
        plot_Polygon(self.boundary, False)
        for obstacle in self.obstacle_list:
            plot_Polygon(obstacle, True)
