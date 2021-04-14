"""
this class is for 2D a star to find out unconstrained heuristic
"""
import distance


class Node_2D:
    def __init__(self, x: float, y: float, cost_so_far=None, heuristic=None, predecessor=None):
        self.x = x
        self.y = y
        self.cost_so_far = cost_so_far
        self.heuristic = heuristic
        self.Predecessor = predecessor
        self.in_open_list = False
        self.in_close_list = False
        self.discover = False
        self.index = -1

    def set_index(self, map_width):
        self.index = self.x + self.y * map_width

    def get_f_value(self):
        return self.cost_so_far + self.heuristic

    def cost(self, other):
        return distance.dist(self, other)

    def update_heuristic(self, goal):
        return self.cost(goal)

    def is_on_grid(self, map) -> bool:
        if map.size[0] < self.x < map.size[1] and map.size[2] < self.y < map.size[3]:
            return True
        else:
            return False

    def update_cost_so_far(self):
        self.cost_so_far = self.cost_so_far + self.cost(self.Predecessor)
        self.discover = True

    def create_successor(self):
        direction = 8
        dx = [-1, -1, 0, 1, 1, 1, 0, -1]
        dy = [0, 1, 1, 1, 0, -1, -1, -1]
        successor = []
        for i in range(direction):
            x_successor = self.x + dx[i]
            y_successor = self.y + dy[i]
            successor.append(Node_2D(x_successor, y_successor, self.cost_so_far, 0, self))
        return successor
