import math
import time
from matplotlib import pyplot as plt
import numpy as np


class Astar:
    def __init__(self):
        self.openlist = set()
        self.closelist = set()
        self.path = []

    def run(self, start, goal):
        self.openlist.add(start)
        start.heuristic(goal)
        while True:
            current = self.min_state()
            if current == -1:
                break
            current.successor(goal)
            self.openlist.remove(current)
            self.closelist.add(current)

            if current.isgoal:
                break
            else:
                for child in current.children:
                    if child.x == current.x and child.y == current.y:  # if child and current are in same cell
                        g = current.g + child.cost(current)
                        tiebreaker = 0.1
                        if child.f > current.f + tiebreaker:
                            continue
                        if (not self.exist(child, self.openlist)) or g < child.g:
                            child.parent = current
                            child.g = g
                            child.heuristic(goal)
                            if not self.exist(child, self.openlist):
                                self.openlist.add(child)
                    elif not self.exist(child, self.closelist):
                        g = current.g + child.cost(current)
                        if (not self.exist(child, self.openlist)) or g < child.g:
                            child.parent = current
                            child.g = g
                            child.heuristic(goal)
                            if not self.exist(child, self.openlist):
                                self.openlist.add(child)

        self.get_backpointer_list(goal, start)

    def get_backpointer_list(self, current, start):
        self.path = [current]
        while True:
            s = current.parent
            self.path.append(s)
            if s == start:
                break
            else:
                current = current.parent

    def min_state(self):
        if not self.openlist:
            return -1
        else:
            return min(self.openlist, key=lambda x: x.g + x.h)

    def exist(self, current, list):
        for n in list:
            if n.x == current.x and n.y == current.y and n.theta == current.theta:
                return True
