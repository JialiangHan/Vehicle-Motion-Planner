import Map
import numpy as np
import Node, Edge, Polygon
from Computational_Geometry import Convex_hull
import geometry


def generate_grid_map(map: Map):
    """
    generate a list of Node_2D for map, key here is the index
    0: free space
    1: obstacle
    """
    max_number_grid = map.size[1] * map.size[3]
    grid_map = np.zeros(max_number_grid)
    for obstacle in map.obstacle_list:
        x_min = round(min(obstacle.vertices, key=lambda vertex: vertex.node.x))
        y_min = round(min(obstacle.vertices, key=lambda vertex: vertex.node.y))
        x_max = round(max(obstacle.vertices, key=lambda vertex: vertex.node.x))
        y_max = round(max(obstacle.vertices, key=lambda vertex: vertex.node.y))
        x, y = x_min, y_min
        while x <= x_max:
            while y <= y_max:
                cell = create_cell(x, y)
                if geometry.intersect_polygons(cell, obstacle):
                    grid_map[x + y * map.size[1]] = 1
                y = y + 1
            x = x + 1
            y = y_min
    return grid_map


def create_cell(x: int, y: int) -> Polygon:
    """
    create a small cell:polygon
    """
    node_list = []
    edge_list = []
    delta_x = [0, 0, 1, 1]
    delta_y = [0, 1, 0, 1]
    for i in range(len(delta_y)):
        x = x + delta_x[i]
        y = y + delta_y[i]
        node_list.append(Node(x, y))
    convexhull = Convex_hull(node_list)
    convexhull.run()
    for j in range(len(convexhull.hull) - 1):
        edge_list.append(Edge.Edge(convexhull.hull[j], convexhull.hull[j + 1]))
    cell = Polygon.Polygon(edge_list)
    return cell
