import math
import matplotlib.pyplot as plt
import os
import sys

sys.path.append(os.path.dirname(os.path.abspath(__file__))
                + "/../Support functions")

try:
    import Map
    import Node_3D
    import DWA
    import Config
except Exception:
    raise

config = Config.Config()


def main():
    # size: list, a list which show max range of map, [min_x, max_x, min_y, max_y],all units are meter
    # n: number of obstacle
    grid_size = 20
    size = [0, grid_size, 0, grid_size]
    n = 1
    # generate map
    map = Map.Map(size, n)

    initial = Node_3D.Node_3D(size[0], size[2], math.radians(0))
    goal = Node_3D.Node_3D(size[1] - 1, size[1] - 1, math.radians(45))
    # plt.subplot(221)
    dwa = DWA.dynamic_window_approach(initial, goal, map, config)
    dwa.run()
    dwa.plot_path()
    plt.show()


if __name__ == '__main__':
    main()
