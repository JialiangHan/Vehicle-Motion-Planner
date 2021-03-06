import matplotlib.patches as patches
import matplotlib.pyplot as plt
import numpy as np

import Edge
import Map
import Node
import Polygon
import geometry
from Computational_Geometry import Convex_hull


class Grid_map:
    def __init__(self, map: Map, resolution):
        self.map = map
        self.resolution = resolution
        self.width = 0
        self.height = 0
        self.calculate_grid_map_size()
        self.grid_map = np.zeros(self.width * self.height)
        self.generate_grid_map(map)

    def calculate_grid_map_size(self):
        remainder_x = self.map.size[1] // self.resolution
        quotient_x = int(self.map.size[1] // self.resolution)
        if remainder_x != 0.0:
            self.width = quotient_x + 1
        else:
            self.width = quotient_x
        remainder_y = self.map.size[3] // self.resolution
        quotient_y = int(self.map.size[3] // self.resolution)
        if remainder_y != 0.0:
            self.height = quotient_y + 1
        else:
            self.height = quotient_y

    def generate_grid_map(self, map):
        """
            generate a list of Node_2D for map, key here is the index
            0: free space
            1: obstacle
            return an array object[1Xn]
            """
        for obstacle in map.obstacle_list:
            x_min = self.floor_resolution(min(obstacle.vertices, key=lambda vertex: vertex.node.x).node.x)
            y_min = self.floor_resolution(min(obstacle.vertices, key=lambda vertex: vertex.node.y).node.y)
            x_max = self.ceil_resolution(max(obstacle.vertices, key=lambda vertex: vertex.node.x).node.x)
            y_max = self.ceil_resolution(max(obstacle.vertices, key=lambda vertex: vertex.node.y).node.y)
            x, y = x_min, y_min
            while x <= x_max:
                while y <= y_max:
                    cell = self.create_cell(x, y)
                    index = self.get_index(x, y)
                    if geometry.intersect_polygons(cell, obstacle):
                        self.grid_map[index] = 1
                    elif geometry.polygon_in_polygon(cell, obstacle) == "inside":
                        self.grid_map[index] = 1
                    elif geometry.polygon_in_polygon(obstacle, cell) == "inside":
                        self.grid_map[index] = 1
                    else:
                        self.grid_map[index] = 0
                    y = round(y + self.resolution, 3)
                x = round(x + self.resolution, 3)
                y = y_min

    def get_index(self, x, y) -> int:
        index = x // self.resolution + (y // self.resolution) * self.width
        return int(index)

    def plot_grid_map(self) -> None:
        fig = plt.figure()
        currentAxis = fig.add_subplot(111)
        currentAxis.axis(self.map.size)
        plt.xticks(np.arange(self.map.size[0], self.map.size[1], step=self.resolution))
        plt.yticks(np.arange(self.map.size[2], self.map.size[3], step=self.resolution))
        plt.grid()
        self.map.Plot(currentAxis)
        for i in range(len(self.grid_map)):
            if self.grid_map[i] == 1:
                x = i % self.width * self.resolution
                y = i // self.width * self.resolution
                rect = patches.Rectangle((x, y), self.resolution, self.resolution)
                currentAxis.add_patch(rect)
        # plt.show()

    def create_cell(self, x, y) -> Polygon:
        """
        create a small cell:polygon
        """
        node_list = []
        edge_list = []
        delta_x = [0, 0, self.resolution, self.resolution]
        delta_y = [0, self.resolution, 0, self.resolution]
        for i in range(len(delta_y)):
            new_x = x + delta_x[i]
            new_y = y + delta_y[i]
            node_list.append(Node.Node(new_x, new_y))
        convexhull = Convex_hull(node_list)
        convexhull.run()
        for j in range(len(convexhull.hull) - 1):
            edge_list.append(Edge.Edge(convexhull.hull[j], convexhull.hull[j + 1]))
        cell = Polygon.Polygon(edge_list)
        return cell

    def floor_resolution(self, x):
        quotient = int(x // self.resolution)
        return round(quotient * self.resolution, 3)

    def ceil_resolution(self, x):
        quotient = int(x // self.resolution)
        return round((quotient + 1) * self.resolution, 3)

    def is_on_grid(self, node):
        """
        this function return True if node in on the grid map
        """
        if node.x % self.resolution == 0.0 and node.y % self.resolution == 0.0:
            return True
        else:
            return False

    def put_on_grid(self, node):
        """
        this function convert a node not on the grid to a node on grid
        """
        if self.is_on_grid(node):
            return node
        else:
            x = self.floor_resolution(node.x)
            y = self.floor_resolution(node.y)
            return Node.Node(x, y)
