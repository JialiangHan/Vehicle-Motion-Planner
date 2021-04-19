"""
table of content:
angle: convert an angle into [0,360]deg
"""
import math


def convert_into_2pi(angle):
    # angle here are in radians
    remainder = angle % (2 * math.pi)
    if remainder >= 0.0:
        return remainder
    else:
        return remainder + math.pi * 2
