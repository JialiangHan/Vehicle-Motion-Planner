import Map
import numpy as np
import Node, Edge, Polygon
from Computational_Geometry import Convex_hull
import geometry, math
import matplotlib.pyplot as plt
import matplotlib.patches as patches


class Grid_map:
    def __init__(self, map: Map, resolution):
        self.map = map
        self.resolution = resolution
        self.width = map.size[1]
        self.grid_map = np.zeros(map.size[1] * map.size[3])
        self.generate_grid_map(map)

    def generate_grid_map(self, map):
        """
            generate a list of Node_2D for map, key here is the index
            0: free space
            1: obstacle
            return an array object[1Xn]
            """
        for obstacle in map.obstacle_list:
            x_min = math.floor(min(obstacle.vertices, key=lambda vertex: vertex.node.x).node.x)
            y_min = math.floor(min(obstacle.vertices, key=lambda vertex: vertex.node.y).node.y)
            x_max = math.ceil(max(obstacle.vertices, key=lambda vertex: vertex.node.x).node.x)
            y_max = math.ceil(max(obstacle.vertices, key=lambda vertex: vertex.node.y).node.y)
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
                    y = y + self.resolution
                x = x + self.resolution
                y = y_min

    def get_index(self, x, y) -> int:
        index = x / self.resolution + y / self.resolution * self.width / self.resolution
        return index

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
                x = i % (self.width / self.resolution) * self.resolution
                y = i // (self.width / self.resolution) * self.resolution
                rect = patches.Rectangle((x, y), self.resolution, self.resolution)
                currentAxis.add_patch(rect)
        # plt.show()

    def create_cell(self, x: int, y: int) -> Polygon:
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
