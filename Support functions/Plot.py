# this file is a collection for plot functions
# include plot_node, plot_edge, plot_polygon
import matplotlib.pyplot as plt


def plot_Node(node):
    plt.plot(node.x, node.y, ".k")


def plot_Edge(edge, figure=None):
    if figure == None:
        plt.plot([edge.start.x, edge.end.x], [edge.start.y, edge.end.y], "k")
    else:
        figure.plot([edge.start.x, edge.end.x], [edge.start.y, edge.end.y], "k")
    # plt.show()


def plot_Polygon(polygon, fill_or_not: bool, figure=None,color=None):
    x, y = [], []
    for edge in polygon.edge_list:
        plot_Edge(edge, figure)
    for vertex in polygon.vertices:
        x.append(vertex.node.x)
        y.append(vertex.node.y)
    if fill_or_not is True:
        if color==None:
            plt.fill(x, y, "b")
        else:
            plt.fill(x,y,color)
    # plt.show()

