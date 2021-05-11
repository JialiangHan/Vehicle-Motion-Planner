"""
this is a python code for dynamic window approach.

model used here is bicycle model
"""
import math
import matplotlib.pyplot as plt
import numpy as np


class dynamic_window_approach:
    def __init__(self,start,goal,map):
        self.start=start
        self.goal=goal
        self.map=map

    def get_dynamic_window(self):



