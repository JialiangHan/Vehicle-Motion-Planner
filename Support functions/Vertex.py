import Node


class Vertex:
    def __init__(self, node: Node.Node, edge_list=None):
        self.node = node
        self.edge_position = {}
        self.edge_list = edge_list
        self.check_position(node, edge_list)

    def __str__(self):
        return "x:" + str(self.node.x)[:4] + ",y:" + str(self.node.y)[:4] + ",position:" + str(self.edge_position)

    def __gt__(self, other):
        if self.node.x != other.node.x:
            return self.node.x > other.node.x
        else:
            return self.node.y > other.node.y

    def check_position(self, node, edge_list):
        for edge in edge_list:
            if node == edge.start:
                self.edge_position[edge] = "start"
            else:
                self.edge_position[edge] = "end"
