"""
this A star is only for calculating unconstrained heuristic for hybrid A
"""

from Node_2D import *
from Grid_map import *
import Plot


class A_star:
    def __init__(self, start: Node_2D, goal: Node_2D, map: Map, resolution):
        self.start = start
        self.goal = goal
        self.map = map
        self.grid_map = Grid_map(map, resolution)
        self.open_list = dict()
        self.close_list = dict()

    def run(self):
        self.start = Node_2D(self.grid_map.put_on_grid(self.start).x, self.grid_map.put_on_grid(self.start).y)
        self.goal = Node_2D(self.grid_map.put_on_grid(self.goal).x, self.grid_map.put_on_grid(self.goal).y)
        self.start.update_heuristic(self.goal)
        self.start.update_cost_so_far()
        self.start.get_f_value()
        index = self.grid_map.get_index(self.start.x, self.start.y)
        self.open_list[index] = self.start
        while len(self.open_list.keys()) > 0:
            current_index = min(self.open_list, key=lambda x: self.open_list[x].f_value)
            current_node = self.open_list[current_index]
            del self.open_list[current_index]
            self.close_list[current_index] = current_node
            if current_node == self.goal:
                self.goal = current_node
                break
            successor = self.create_successor(current_node)
            for succ in successor:
                if succ.is_in_map(self.map):
                    succ_index = self.grid_map.get_index(succ.x, succ.y)
                    if succ_index not in self.close_list:
                        if self.grid_map.grid_map[succ_index] != 1:
                            succ.update_cost_so_far()
                        else:
                            succ.cost_so_far = 10000
                        succ.update_heuristic(self.goal)
                        succ.get_f_value()
                        if succ_index not in self.open_list:
                            self.open_list[succ_index] = succ
                        else:
                            if succ.cost_so_far < self.open_list[succ_index].cost_so_far:
                                self.open_list[succ_index] = succ

    def get_path(self):
        path_x, path_y = [self.goal.x], [self.goal.y]
        current_node = self.goal
        while 1:
            current_node = current_node.Predecessor
            path_x.append(current_node.x)
            path_y.append(current_node.y)
            if current_node == self.start:
                break
        return path_x, path_y

    def create_successor(self, current_node):
        direction = 8
        dx = [-self.grid_map.resolution, -self.grid_map.resolution, 0, self.grid_map.resolution,
              self.grid_map.resolution, self.grid_map.resolution, 0, -self.grid_map.resolution]
        dy = [0, self.grid_map.resolution, self.grid_map.resolution, self.grid_map.resolution, 0,
              -self.grid_map.resolution, -self.grid_map.resolution, -self.grid_map.resolution]
        successor = []
        for i in range(direction):
            x_successor = current_node.x + dx[i]
            y_successor = current_node.y + dy[i]
            successor.append(Node_2D(x_successor, y_successor, current_node.cost_so_far, 0, current_node))
        return successor

    def plot(self):
        self.grid_map.plot_grid_map()
        path_x, path_y = self.get_path()
        for x, y in zip(path_x, path_y):
            cell = self.grid_map.create_cell(x, y)
            if path_x.index(x) == 0:
                color = "r"
            elif path_x.index(x) == len(path_x)-1:
                color = "y"
            else:
                color = 'g'
            Plot.plot_Polygon(cell, True, None, color)
        plt.show()
