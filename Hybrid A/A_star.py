"""
this A star is only for calculating unconstrained heuristic for hybrid A
"""

from Node_2D import *
from Grid_map import *
import Plot


class A_star:
    def __init__(self, start: Node_2D, goal: Node_2D, map: Map):
        self.start = start
        self.goal = goal
        self.map = map
        self.grid_map = Grid_map(map)
        self.open_list = dict()
        self.close_list = dict()

    def run(self):
        width = self.map.size[1]
        self.start.update_heuristic(self.goal)
        self.start.update_cost_so_far()
        self.start.get_f_value()
        self.start.set_index(width)
        self.open_list[self.start.index] = self.start
        while len(self.open_list.keys()) > 0:
            current_index = min(self.open_list, key=lambda x: self.open_list[x].f_value)
            current_node = self.open_list[current_index]
            del self.open_list[current_index]
            self.close_list[current_index] = current_node
            if current_node == self.goal:
                self.goal = current_node
                break
            successor = current_node.create_successor()
            for succ in successor:
                if succ.is_on_grid(self.map):
                    succ.set_index(self.map.size[1])
                    if succ.index not in self.close_list:
                        if self.grid_map.grid_map[succ.index] != 1:
                            succ.update_cost_so_far()
                        else:
                            succ.cost_so_far = 10000
                        succ.update_heuristic(self.goal)
                        succ.get_f_value()
                        if succ.index not in self.open_list:
                            self.open_list[succ.index] = succ
                        else:
                            if succ.cost_so_far < self.open_list[succ.index].cost_so_far:
                                self.open_list[succ.index] = succ

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

    def plot(self):
        self.grid_map.plot_grid_map()
        path_x,path_y=self.get_path()
        for x,y in zip(path_x,path_y):
            cell=self.grid_map.create_cell(x,y)
            Plot.plot_Polygon(cell,True,None ,"g")
        plt.show()