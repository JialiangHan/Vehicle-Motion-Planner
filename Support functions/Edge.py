import Node
# import distance


class Edge:
    def __init__(self, start: Node.Node, end: Node.Node):
        # we want to make sure p is always the left start point
        if start.x < end.x:
            self.start = start
            self.end = end
        elif start.x > end.x:
            self.start = end
            self.end = start
        else:
            if start.y < end.y:
                self.start = start
                self.end = end
            else:
                self.start = end
                self.end = start
        # self.length = distance.dist(start, end)
        # line function: Ax+By+C=0
        self.A = 0
        self.B = 0
        self.C = 0
        self.calculate()

    def calculate(self):
        if self.start.x == self.end.x:
            self.A = 1
            self.B = 0
            self.C = -self.start.x
        elif self.start.y == self.end.y:
            self.A = 0
            self.B = 1
            self.C = -self.start.y
        else:
            self.A = -(self.start.y - self.end.y) / (self.start.x - self.end.x)
            self.B = 1
            self.C = -self.A * self.start.x - self.B * self.start.y

    def on_edge(self, node: Node.Node) -> bool:
        if self.start.x < node.x < self.end.x:
            if self.A * node.x + self.B * node.y + self.C == 0:
                return True
            else:
                return False
        return False

    def aboveLine(self, node: Node.Node) -> bool:
        """
        Return true if node lies above line segment 'self'.
        http://stackoverflow.com/enduestions/3838319/how-can-i-check-if-a-node-is-below-a-line-or-not
        :param node:
        :return:
        """
        v1x = self.end.x - self.start.x  # Vector 1.x
        v1y = self.end.y - self.start.y  # Vector 1.y
        v2x = self.end.x - node.x  # Vector 2.x
        v2y = self.end.y - node.y  # Vector 2.y
        xp = v1x * v2y - v1y * v2x  # Cross product
        # when its larger than zero, return false
        # so we assume that if it lies on the line that it is "above"
        if xp > 0:
            return False
        else:
            return True

    def belowOther(self, other) -> bool:
        if self.aboveLine(other.start) and self.aboveLine(other.end):
            return True
        if not other.aboveLine(self.start) and not other.aboveLine(self.end):
            return True
        return False

    def __gt__(self, other):
        return not self.belowOther(other)

    def cmp(self, other):
        if other.start == self.start and other.end == self.end:
            return True
        else:
            return False

    def __repr__(self):
        return '<Segment start:%s end:%s>' % (self.start.__str__(), self.end.__str__())
