import math
import random

import matplotlib.pyplot as plt


class node:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __str__(self):
        return "x: " + str(self.x) + ", y: " + str(self.y)


class Event:
    def __init__(self, x, p, a):


class line:
    def __init__(self, a, b):
        self.start = a
        self.end = b
        self.k = 0
        self.b = 0
        self.slope()

    def __str__(self):
        return "start: " + str(self.start) + ", end: " + str(self.end) + ", k: " + str(self.k) + ", b: " + str(self.b)

    def __eq__(self, other):
        return (self.start == other.start and self.end == other.end) or (
                self.start == other.end and self.end == other.start)

    def slope(self):
        if self.start.x == self.end.x:
            self.k = float("inf")
            self.b = float("-inf")
        else:
            self.k = (self.end.y - self.start.y) / (self.end.x - self.start.x)
            self.b = self.start.y - self.k * self.start.x


class Voronoi:
    def __init__(self, sites, xmin, xmax, ymin, ymax, decimal):
        self.decimal = decimal
        self.sites = sites
        self.bisector = []
        self.edges = []
        self.vertices = []
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax

    def crossproduct(self, a, b):
        # a, b are vector(list), this operation is for vector
        return a[0] * b[1] - a[1] * b[0]

    def if_intersect(self, a, b):
        # determian whether line a and line b intersect?
        # two step, use line a as base, then use line b as base
        # create two vector,
        va = [a.end.x - a.start.x, a.end.y - a.start.y]
        vb = [b.end.x - b.start.x, b.end.y - b.start.y]
        v1 = [b.start.x - a.start.x, b.start.y - a.start.y]
        v2 = [b.end.x - a.start.x, b.end.y - a.start.y]
        v3 = [a.start.x - b.start.x, a.start.y - b.start.y]
        v4 = [a.end.x - b.start.x, a.end.y - b.start.y]
        if self.crossproduct(vb, v3) * self.crossproduct(vb, v4) < 0 and self.crossproduct(va, v1) * self.crossproduct(
                va,
                v2) < 0:
            return True
        else:
            return False

    def dist(self, a, b):
        # this is an eulidean distance function, a,b are nodes
        deltax = b.x - a.x
        deltay = b.y - a.y
        result = math.sqrt(deltax ** 2 + deltay ** 2)
        result = round(result, self.decimal)
        return result

    def intersection(self, a, b):
        # this function find intersection point of two lines,line a and line b
        if a.start.x == a.end.x or b.start.x == b.end.x:
            if a.start.x == a.end.x:
                x = a.start.x
                k2 = (b.end.y - b.start.y) / (b.end.x - b.start.x)
                b2 = b.start.y + k2 * b.start.x
                y = k2 * x + b2
            else:
                x = b.start.x
                k1 = (a.end.y - a.start.y) / (a.end.x - a.start.x)
                b1 = a.start.y + k1 * a.start.x
                y = k1 * x + b1
        else:
            k1 = (a.end.y - a.start.y) / (a.end.x - a.start.x)
            b1 = a.start.y + k1 * a.start.x
            k2 = (b.end.y - b.start.y) / (b.end.x - b.start.x)
            b2 = b.start.y + k2 * b.start.x
            x = (b2 - b1) / (k1 - k2)
            y = k1 * x + b1
        return node(x, y)

    # def find_vertices(self):
    #     #find all vertices
    #
    def find_edges(self):
        # find all edges
        candidates = []
        l = len(self.bisector)
        for i in range(l):
            intersection = []
            for j in range(l):
                if i == j:
                    continue
                else:
                    if self.if_intersect(self.bisector[i], self.bisector[j]):
                        a = self.intersection(self.bisector[i], self.bisector[j])
                        if a not in intersection:
                            intersection.append(a)
            intersection.sort(key=lambda n: n.x)
            n = len(intersection)
            if n == 0:
                candidates = [line(self.bisector[i].start, self.bisector[j].end)]
            elif n == 1:
                candidates = [line(self.bisector[i].start, intersection[0]),
                              line(intersection[0], self.bisector[j].end)]
            else:
                candidates = [line(self.bisector[i].start, intersection[0]),
                              line(intersection[-1], self.bisector[j].end)]
                for i in range(n - 1):
                    candidates.append(line(intersection[i], intersection[i + 1]))
                    # # this only for one intersection on one line
                    # for s in subcandidates:
                    #     if s not in candidates:
                    #         candidates.append(s)

        dstart = []
        dend = []
        for candidate in candidates:
            for site in self.sites:
                dstart.append(self.dist(candidate.start, site))
                dend.append(self.dist(candidate.end, site))
            dstart.sort()
            dend.sort()
            if dstart[0] == dstart[1] and dend[0] == dend[1]:
                self.edges.append(candidate)
            dstart = []
            dend = []

    def find_all_bisector(self):
        for i in range(len(self.sites)):
            for j in range(i + 1, len(self.sites)):
                self.bisector.append(self.get_bisector(self.sites[i], self.sites[j]))

    def get_bisector(self, a, b):
        # this k is for line ab
        # this kb is for bisector
        # this b is for bisector, function for bisector is y=kb*x+b
        if b.x == a.x:
            kb = 0
            b = 1 / 2 * (a.y + b.y)
            # find point for x=xmin
            yxmin = kb * self.xmin + b
            # find point for x=xmax
            yxmax = kb * self.xmax + b
            start = node(self.xmin, yxmin)
            end = node(self.xmax, yxmax)
            return line(start, end)
        elif a.y == b.y:
            # find point for y=ymin
            xymin = 1 / 2 * (a.x + b.x)
            # find point for y=ymax
            xymax = 1 / 2 * (a.x + b.x)
            start = node(xymin, self.ymin)
            end = node(xymax, self.ymax)
            return line(start, end)
        else:
            k = (b.y - a.y) / (b.x - a.x)
            b = (a.x + b.x) / (2 * k) + (a.y + b.y) / 2
            kb = -1 / k
            candidates = []
            # find point for x=xmin
            yxmin = kb * self.xmin + b
            candidates.append(node(self.xmin, yxmin))
            # find point for x=xmax
            yxmax = kb * self.xmax + b
            candidates.append(node(self.xmax, yxmax))
            # find point for y=ymin
            xymin = (self.ymin - b) / kb
            candidates.append(node(xymin, self.ymin))
            # find point for y=ymax
            xymax = (self.ymax - b) / kb
            candidates.append(node(xymax, self.ymax))
            candidates.sort(key=lambda n: n.x)
            return line(candidates[1], candidates[2])


def main():
    # specify max range for x and y
    xmin, xmax, ymin, ymax = 0, 10, 0, 10
    # specify number of sites
    n = 3
    sites = []
    decimal = 4
    # generate sites
    random.seed(1)
    for i in range(n):
        sites.append(node(random.randint(xmin + 1, xmax - 1), random.randint(ymin + 1, ymax - 1)))
    # draw sites
    for site in sites:
        plt.plot(site.x, site.y, 'bo', ms=5)
    V = Voronoi(sites, xmin, xmax, ymin, ymax, decimal)
    V.find_all_bisector()
    V.find_edges()
    # draw bisector
    for edge in V.edges:
        plt.plot([edge.start.x, edge.end.x], [edge.start.y, edge.end.y], 'black')
        print([edge.start.x, edge.start.y], [edge.end.x, edge.end.y])
    plt.xlim(xmin, xmax)
    plt.ylim(ymin, ymax)
    plt.show()


if __name__ == '__main__':
    main()
