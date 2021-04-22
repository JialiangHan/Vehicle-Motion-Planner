import numpy as np

import Edge
import Node
import Polygon
import Vertex


def cross_product(list1: list, list2: list) -> int:
    result = list1[0] * list2[1] - list1[1] * list2[0]
    return result


def intersect(edge1: Edge.Edge, edge2: Edge.Edge) -> bool:
    """
    determine if two edges are intersect with each other:
    True: intersect
    False: not intersect
    """
    x_max_1 = max(edge1.start.x, edge1.end.x)
    x_min_1 = min(edge1.start.x, edge1.end.x)
    y_max_1 = max(edge1.start.y, edge1.end.y)
    y_min_1 = min(edge1.start.y, edge1.end.y)
    x_max_2 = max(edge2.start.x, edge2.end.x)
    x_min_2 = min(edge2.start.x, edge2.end.x)
    y_max_2 = max(edge2.start.y, edge2.end.y)
    y_min_2 = min(edge2.start.y, edge2.end.y)
    if x_max_2 < x_min_1 or y_max_2 < y_min_1:
        return False
    elif x_max_1 < x_min_2 or y_max_1 < y_min_2:
        return False
    else:
        CA = [edge2.start.x - edge1.start.x, edge2.start.y - edge1.start.y]
        CD = [edge1.end.x - edge1.start.x, edge1.end.y - edge1.start.y]
        CB = [edge2.end.x - edge1.start.x, edge2.end.y - edge1.start.y]
        AB = [edge2.end.x - edge2.start.x, edge2.end.y - edge2.start.y]
        AC = [edge1.start.x - edge2.start.x, edge1.start.y - edge2.start.y]
        AD = [edge1.end.x - edge2.start.x, edge1.end.y - edge2.start.y]
        result1 = cross_product(CA, CD) * cross_product(CB, CD)
        result2 = cross_product(AC, AB) * cross_product(AD, AB)
        if result1 < 0 and result2 < 0:
            return True
        else:
            return False


def intersection(edge1: Edge.Edge, edge2: Edge.Edge) -> Node.Node:
    """
    this function return intersection node of two edges
    input type: two edges
    output type: a node
    """
    if intersect(edge1, edge2):
        a = np.array([[edge1.A, edge1.B], [edge2.A, edge2.B]])
        b = np.array([[-edge1.C], [-edge2.C]])
        result = np.linalg.solve(a, b)
        result = Node.Node(result[0][0], result[1][0])
        return result


def left_or_right(vertex, edge):
    """
    output is left,right, location of an edge compared to vertex
    """
    result = []
    list_node = [edge.start, edge.end]
    for i in range(len(list_node)):
        if vertex.node == list_node[i]:
            if vertex.node.x > list_node[1 - i].x:
                result.append("left")
            else:
                result.append("right")
    return result


def node_in_edge(node: Node.Node, edge: Edge.Edge) -> list:
    if node == edge.start:
        return [True, edge.end]
    elif node == edge.end:
        return [True, edge.start]
    else:
        return [False, None]


def edge_in_polygon(edge: Edge.Edge, polygon: Polygon.Polygon) -> bool:
    if polygon is None:
        return False
    else:
        if edge in polygon.edge_list:
            return True
        else:
            return False


def node_in_polygon(node: Node.Node, polygon: Polygon.Polygon) -> bool:
    """
    射线法, 点在内部,则射线与多边形的交点为奇数
    点在外部,交点数为偶数
    this function determine if a node is inside or on the boundary of a polygon
    :param node:
    :param polygon:
    :return: False: node outside polygon
    True: node inside polygon
    """
    # create 射线
    line = Edge.Edge(node, Node.Node(10000, node.y))
    count = 0
    for edge in polygon.edge_list:
        if intersect(edge, line):
            count += 1
    if count % 2 == 0:
        return False
    else:
        return True


def vertex_in_obstacle(vertex: Vertex.Vertex, obstacle_list: list) -> Polygon.Polygon or None:
    """
    this function determine if vertex is in obstacle.vertices
    :param vertex:
    :param obstacle_list:
    :return:
    """
    for obstacle in obstacle_list:
        if vertex in obstacle.vertices:
            return obstacle
    return None


def intersect_polygons(polygon1: Polygon, polygon2: Polygon) -> bool:
    """
    this function return True or False: if two polygons are intersected
    True:intersect
    False: not intersect
    """
    for edge in polygon1.edge_list:
        if intersect_edge_polygon(edge, polygon2):
            return True
    return False


def polygon_in_polygon(polygon1: Polygon, polygon2: Polygon) -> str:
    """
    this function return True or False: if polygon1 is inside polygon2
    True: inside
    False: not inside
    """
    n = len(polygon1.vertices)
    m = len(polygon2.vertices)
    number_of_inside = 0
    number_of_outside = 0
    for vertex in polygon1.vertices:
        if node_in_polygon(vertex.node, polygon2):
            number_of_inside += 1
        else:
            number_of_outside += 1
    if number_of_outside == n:
        return "outside"
    else:
        return "inside"


def intersect_edge_polygon(edge: Edge, polygon: Polygon) -> bool:
    """
    this function return True or False: if edge and polygon are intersected
    True:intersect
    False: not intersect
    """
    for edge1 in polygon.edge_list:
        if intersect(edge, edge1):
            return True
    return False
