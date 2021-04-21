"""
this is a path smoother class for path got from hybrid A star.py
function should include:
objective function P = Pobs+Pcurvature+Psmooth+Pvoronoi
gradient descent
"""
import distance


class Path_smoother:
    def __init__(self,path,map):
        self.path=path
        self.map=map

    def objective_function(self):

    def obstacle_term(self,node):
        # find distance to closest obstacle
        distance_to_closest_obstacle = float('inf')
        for obstacle in self.map.obstacle_list:
            distance_to_obstacle=distance.distance_node_to_polygon(node,obstacle)
            if distance_to_obstacle<distance_to_closest_obstacle:
                distance_to_closest_obstacle=distance_to_obstacle

    def curvature_term(self):

    def smooth_term(self):

    def voronoi_term(self):


    def gradient_descent(self):

