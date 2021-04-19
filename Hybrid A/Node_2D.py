"""
this class is for 2D a star to find out unconstrained heuristic
"""
import distance


class Node_2D:
    def __init__(self, x: int, y: int, cost_so_far=None, heuristic=None, predecessor=None):
        self.x = x
        self.y = y
        if cost_so_far is not None:
            self.cost_so_far = cost_so_far
        else:
            self.cost_so_far = 0
        self.heuristic = heuristic
        if predecessor is None:
            self.Predecessor = self
        else:
            self.Predecessor = predecessor
        self.in_open_list = False
        self.in_close_list = False
        self.discover = False
        self.index = -1
        self.f_value = 0

    def get_f_value(self):
        self.f_value = self.cost_so_far + self.heuristic

    def cost(self, other):
        return distance.dist(self, other)

    def update_heuristic(self, goal):
        self.heuristic = self.cost(goal)

    def is_in_map(self, map) -> bool:
        if map.size[0] <= self.x < map.size[1] and map.size[2] <= self.y < map.size[3]:
            return True
        else:
            return False

    def update_cost_so_far(self):
        self.cost_so_far = self.cost_so_far + self.cost(self.Predecessor)
        self.discover = True

    def __gt__(self, other):
        return self.f_value > other.f_value

    def __eq__(self, other):
        if self.x == other.x and self.y == other.y:
            return True
        else:
            return False

    def __str__(self):
        return "x: " + str(self.x)[:4] + ", y: " + str(self.y)[:4]
