# this file contains all distance functions:
# distance between nodes
# distance from node to segment
# distance from node to polygon
from math import sqrt

import Edge
import Node
import Polygon


def dist(a: Node.Node, b: Node.Node) -> float:
    # a, b are nodes from geometry
    # Euclidean distance
    delta_x = a.x - b.x
    delta_y = a.y - b.y
    distance = sqrt(delta_y ** 2 + delta_x ** 2)
    return distance


def distance_node_to_polygon(node: Node.Node, polygon: Polygon.Polygon) -> float:
    # distance between node and polygon,
    # determine if edge of polygon are visible to node
    # then calculate distance
    # return min distance
    # not consider obstacle
    distance = float("inf")
    for edge in polygon.edge_list:
        if distance > distance_node_to_segment(node, edge):
            distance = distance_node_to_segment(node, edge)
    return distance


def distance_node_to_segment(node: Node.Node, segment: Edge.Edge) -> float:
    # distance between node and segment
    A_start, B_start, C_start = perpendicular(segment.start, segment)
    A_end, B_end, C_end = perpendicular(segment.end, segment)
    if line_value(A_start, B_start, C_start, node) * line_value(A_end, B_end, C_end, node) < 0:
        distance = abs(line_value(segment.A, segment.B, segment.C, node) / sqrt(segment.A ** 2 + segment.B ** 2))
    else:
        distance = min(dist(node, segment.start), dist(node, segment.end))
    return distance


def line_value(A: float, B: float, C: float, node: Node.Node) -> float:
    # determine if a node is on which side of line
    result = A * node.x + B * node.y + C
    return result


def perpendicular(node: Node.Node, segment: Edge.Edge) -> tuple:
    # this function return a line that perpendicular to segment and pass the node
    # need convert kx+b=y to ax+by+c=0 for all line, segment. done 202103292131
    if segment.A == 0:
        A = 1
        B = 0
        C = -node.x
    elif segment.B == 0:
        A = 0
        B = 1
        C = -node.y
    else:
        A = -segment.B / segment.A
        B = 1
        C = -A * node.x - node.y
    return A, B, C
