import Vertex


class Polygon:
    def __init__(self, edge_list: list):
        self.vertices = []
        self.edge_list = edge_list
        self.get_vertices()

    def get_vertices(self):
        Temp = {}
        for edge in self.edge_list:
            Temp[edge.start] = []
            Temp[edge.end] = []
        for edge in self.edge_list:
            Temp[edge.start].append(edge)
            Temp[edge.end].append(edge)
        for key, value in Temp.items():
            self.vertices.append(Vertex.Vertex(key, value))
