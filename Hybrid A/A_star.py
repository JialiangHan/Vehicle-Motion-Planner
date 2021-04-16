"""
this A star is only for calculating unconstrained heuristic for hybrid A
"""

from Node_2D import *
from Grid_map import *
import heapq


class A_star:
    def __init__(self, start: Node_2D, goal: Node_2D, map: Map):
        self.start = start
        self.goal = goal
        self.map = map
        self.grid_map = Grid_map(map)
        self.open_list = []
        self.close_list = []

    def run(self):
        self.start.update_heuristic(self.goal)
        self.start.update_cost_so_far()
        self.start.get_f_value()
        heapq.heappush(self.open_list, self.start)
        while self.open_list:
            current_node = heapq.heappop(self.open_list)
            self.close_list.append(current_node)
            if current_node == self.goal:
                break
            successor = current_node.create_successor()
            for succ in successor:
                if succ.is_on_grid(self.map):
                    if succ not in self.close_list:
                        succ.set_index(self.map.size[1])
                        if self.grid_map.grid_map[succ.index] != 1:
                            succ.update_cost_so_far()
                        else:
                            succ.cost_so_far = 10000
                        succ.update_heuristic(self.goal)
                        succ.get_f_value()
                        if succ not in self.open_list:
                            heapq.heappush(self.open_list, succ)
                        elif current_node.cost_so_far + current_node.cost(succ) < succ.cost_so_far:
                            succ.Predecessor = current_node
                            succ.update_cost_so_far()
                            succ.get_f_value()

    def get_path(self):
        path_x, path_y = [self.goal.x], [self.goal.y]
        current_node = self.goal
        while 1:
            current_node = current_node.Predecessor
            path_x.append(current_node.x)
            path_y.append(current_node.y)
        return path_x, path_y
