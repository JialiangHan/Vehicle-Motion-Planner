"""
this file contains some function for computational geometry
function list:
convex hull
"""

from geometry import cross_product


class Convex_hull:
    """
    class convex hull: compute convex hull for a node list
    input:a node list without ordered
    output: hull edge list in clockwise order
    """

    def __init__(self, node_list):
        self.node_list = node_list
        self.n = len(self.node_list)
        self.sort()
        self.upper_hull = []
        self.lower_hull = []
        self.hull = []

    def sort(self):
        # sort node list according to x coordinate, when x coordinate then use y coordinate
        self.node_list.sort(key=lambda x: x.x)

    def run(self):
        self.get_upper_hull()
        self.get_lower_hull()
        self.get_hull()

    def get_upper_hull(self):
        self.upper_hull.append(self.node_list[0])
        self.upper_hull.append(self.node_list[1])
        for i in range(2, self.n):
            self.upper_hull.append(self.node_list[i])
            while len(self.upper_hull) > 2 and not self.turn_right(self.upper_hull[-3:]):
                self.upper_hull.pop(-2)

    def get_lower_hull(self):
        self.lower_hull.append(self.node_list[-1])
        self.lower_hull.append(self.node_list[-2])
        for i in range(3, self.n + 1):
            self.lower_hull.append(self.node_list[-i])
            while len(self.lower_hull) > 2 and not self.turn_right(self.lower_hull[-3:]):
                self.lower_hull.pop(-2)

    def get_hull(self):
        self.lower_hull.pop(0)
        self.hull = self.upper_hull + self.lower_hull

    @staticmethod
    def turn_right(node):
        """

        :param node:
        :return: True(turn right) or False(not turn right)
        """
        list = []
        for i in range(len(node) - 1):
            delta_x = node[i + 1].x - node[i].x
            delta_y = node[i + 1].y - node[i].y
            list.append([delta_x, delta_y])
        if cross_product(list[0], list[1]) > 0:
            return True
        else:
            return False
