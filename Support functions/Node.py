class Node:
    def __init__(self, x: int or float, y: int or float):
        self.x = x
        self.y = y
        self.successor = []
        self.predecessor = []

    def __str__(self):
        return "x: " + str(self.x)[:4] + ", y: " + str(self.y)[:4]

    # def __eq__(self, other):
    #     if other.x == self.x and other.y == self.y:
    #         return True
    #     else:
    #         return False
