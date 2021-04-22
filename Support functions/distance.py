# this file contains all distance functions:
# distance between nodes
# distance from node to segment
# distance from node to polygon
from math import sqrt

import Edge
import Node
import Polygon
import geometry


def dist(a: Node.Node, b: Node.Node) -> float:
    # a, b are nodes from geometry
    # Euclidean distance
    delta_x = a.x - b.x
    delta_y = a.y - b.y
    distance = sqrt(delta_y ** 2 + delta_x ** 2)
    return distance


def distance_node_to_polygons(node: Node.Node, polygon_list: list):
    temp = {}
    for polygon in polygon_list:
        distance, closest_node = distance_node_to_polygon(node, polygon)
        temp[closest_node] = distance
    closest_node = min(temp, key=temp.get)
    distance = temp[closest_node]
    return distance, closest_node


def distance_node_to_polygon(node: Node.Node, polygon: Polygon.Polygon) -> float:
    # distance between node and polygon,
    temp = {}
    for edge in polygon.edge_list:
        distance = distance_node_to_segment(node, edge)
        closest_node = closest_node_on_segment_to_node(node, edge)
        temp[closest_node] = distance
    closest_node = min(temp, key=temp.get)
    distance = temp[closest_node]
    return distance, closest_node


def distance_node_to_segment(node: Node.Node, segment: Edge.Edge):
    # distance between node and segment, return min distance and closest point on segment to the node
    A_start, B_start, C_start = perpendicular(segment.start, segment)
    A_end, B_end, C_end = perpendicular(segment.end, segment)
    if line_value(A_start, B_start, C_start, node) * line_value(A_end, B_end, C_end, node) < 0:
        distance = abs(line_value(segment.A, segment.B, segment.C, node) / sqrt(segment.A ** 2 + segment.B ** 2))
        node_A, node_B, node_C = perpendicular(node, segment)
        x = 10000
        y = (-node_A * x - node_C) / node_B
        end = Node.Node(x, y)
        node_segment = Edge.Edge(node, end)
        if geometry.intersect(segment, node_segment):
            closest_node = geometry.intersection(segment, node_segment)
    else:
        distance = min(dist(node, segment.start), dist(node, segment.end))
    return distance


def closest_node_on_segment_to_node(node, segment) -> Node.Node:
    A_start, B_start, C_start = perpendicular(segment.start, segment)
    A_end, B_end, C_end = perpendicular(segment.end, segment)
    if line_value(A_start, B_start, C_start, node) * line_value(A_end, B_end, C_end, node) < 0:
        node_A, node_B, node_C = perpendicular(node, segment)
        x = 10000
        y = (-node_A * x - node_C) / node_B
        end = Node.Node(x, y)
        node_segment = Edge.Edge(node, end)
        if geometry.intersect(segment, node_segment):
            closest_node = geometry.intersection(segment, node_segment)
        else:
            x = -10000
            y = (-node_A * x - node_C) / node_B
            end = Node.Node(x, y)
            node_segment = Edge.Edge(node, end)
            closest_node = geometry.intersection(segment, node_segment)
    else:
        closest_node = segment.start
        distance_to_start = dist(node, segment.start)
        distance_to_end = dist(node, segment.end)
        if distance_to_start > distance_to_end:
            closest_node = segment.end
    return closest_node


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
