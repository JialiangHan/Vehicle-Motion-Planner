"""
this A star is only for calculating unconstrained heuristic for hybrid A
"""

from .Node_2D import *
import Map
import heapq


class A_star():
    def __init__(self, start: Node_2D, goal: Node_2D, map: Map):
        self.start = start
        self.goal = goal
        self.map = map
        self.grip_map=[]
        self.open_list = heapq.heapify([])
        self.close_list = []


