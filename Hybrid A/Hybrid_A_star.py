"""
this A star is only for calculating unconstrained heuristic for hybrid A
"""
from Node_3D import *
import Vehicle_model
from A_star import *
import dubins_path_planning as dubins
import reeds_shepp_path_planning as rs

REVERSE = True
TIEBREAKER = 0.75


class Hybrid_A_star:
    def __init__(self, start: Node_3D, goal: Node_3D, map: Map.Map, xy_grid_resolution, angle_resolution):
        self.start = start
        self.goal = goal
        self.map = map
        self.grid_resolution = xy_grid_resolution
        self.grid_map = Grid_map(map, self.grid_resolution)
        self.angle_resolution = angle_resolution
        self.open_list = dict()
        self.close_list = dict()

    def run(self):
        self.update_heuristics(self.start)
        self.start.update_cost_so_far()
        self.start.get_f_value()
        index = self.get_index(self.start)
        self.open_list[index] = self.start
        while len(self.open_list.keys()) > 0:
            current_index = min(self.open_list, key=lambda x: self.open_list[x].f_value)
            current_node = self.open_list[current_index]
            del self.open_list[current_index]
            self.close_list[current_index] = current_node
            if current_index == self.get_index(self.goal):
                self.goal = current_node
                break
            # todo need add dubins or RS expansion here
            successor = current_node.create_successor()  # need consider direction
            for succ in successor:
                if succ.is_in_map(self.map):
                    succ_index = self.get_index(succ)
                    if succ_index not in self.close_list or succ_index == current_index:
                        succ_index_2D = self.grid_map.get_index(succ.x, succ.y)
                        if self.grid_map.grid_map[succ_index_2D] != 1:
                            succ.update_cost_so_far()
                        else:
                            succ.cost_so_far = 10000
                        self.update_heuristics(succ)
                        succ.get_f_value()
                        if succ_index not in self.open_list or succ.cost_so_far < self.open_list[succ_index].cost_so_far or succ_index == current_index:
                            #self.update_heuristics(succ)
                            if succ_index == current_index and succ.f_value > current_node.f_value + TIEBREAKER:
                                pass
                            elif succ_index == current_index and succ.f_value <= current_node.f_value + TIEBREAKER:
                                self.open_list[succ_index] = succ
                            else:
                                self.open_list[succ_index] = succ

    def update_heuristics(self, node):
        constrained_heuristic = self.constrained_heuristic(node)
        unconstrained_heuristic = self.unconstrained_heuristic(node)
        node.heuristic = max(constrained_heuristic, unconstrained_heuristic)

    def constrained_heuristic(self, node):
        """
        consider vehicle constraint but neglect obstacle
        """
        # todo make this function an offline lookup table
        curvature = 1 / R
        if REVERSE is True:
            _, _, _, _, result = rs.reeds_shepp_path_planning(node.x, node.y, node.theta,
                                                              self.goal.x, self.goal.y, self.goal.theta, curvature)
        else:
            _, _, _, _, result = dubins.dubins_path_planning(node.x, node.y, node.theta,
                                                             self.goal.x, self.goal.y, self.goal.theta, curvature)
        return result

    def unconstrained_heuristic(self, node):
        """
        this is heuristic from A star, not consider vehicle constraint, consider obstacle
        """
        start = Node_2D(self.goal.x, self.goal.y)
        goal = Node_2D(node.x, node.y)
        a_star = A_star(start, goal, self.map, self.grid_resolution)
        a_star.run()
        result = a_star.goal.cost_so_far
        return result

    def get_path(self):
        path_x, path_y, path_theta = [self.goal.x], [self.goal.y], [self.goal.theta]
        current_node = self.goal
        while 1:
            current_node = current_node.Predecessor
            path_x.append(current_node.x)
            path_y.append(current_node.y)
            path_theta.append(current_node.theta)
            if current_node == self.start:
                break
        return path_x, path_y, path_theta

    def get_index(self, node):
        index = (node.x - self.map.size[0]) // self.grid_resolution + \
                (node.y - self.map.size[2]) // self.grid_resolution * self.grid_map.width + \
                node.theta // self.angle_resolution * self.grid_map.width * self.grid_map.height
        return int(index)

    def plot_path(self):
        self.map.Plot()
        path_x, path_y, path_theta = self.get_path()
        plt.plot(path_x, path_y, "-r", label="Hybrid A* path")

    def plot_vehicle(self):
        path_x, path_y, path_theta = self.get_path()
        for x, y, yaw in zip(path_x, path_y, path_theta):
            plt.grid(True)
            plt.axis("equal")
            Vehicle_model.plot_car(x, y, yaw)
            plt.pause(0.0001)
