import numpy as np
import math
import matplotlib.pyplot as plt


def cartesian_to_frenet1D(rs, rx, ry, rtheta, x, y):
    s_condition = np.zeros(1)
    d_condition = np.zeros(1)

    dx = x - rx
    dy = y - ry

    cos_theta_r = math.cos(rtheta)
    sin_theta_r = math.sin(rtheta)

    cross_rd_nd = cos_theta_r * dy - sin_theta_r * dx
    d_condition[0] = math.copysign(math.sqrt(dx * dx + dy * dy), cross_rd_nd)

    s_condition[0] = rs

    return s_condition, d_condition


def find_nearest_rs(fx, fy, x, y):
    min_dist = 99999.0
    rs = 0.0
    for s in snew:
        dx = x - fx(s)
        dy = y - fy(s)
        dist = np.sqrt(dx * dx + dy * dy)
        if min_dist > dist:
            min_dist = dist
            rs = s
    return rs


theta = np.linspace(0, np.pi, 10)
x = 5.0 * np.cos(theta)
y = 5.0 * np.sin(theta)
s = theta * 5.0

fx = np.poly1d(np.polyfit(s, x, 5))
dfx = fx.deriv()

fy = np.poly1d(np.polyfit(s, y, 5))
dfy = fy.deriv()

snew = np.linspace(0, 5 * np.pi, 1000)

newx = fx(snew)
newy = fy(snew)

rs = find_nearest_rs(fx, fy, 2.0, 3.5)
rx = fx(rs)
ry = fy(rs)
rtheta = math.atan2(dfy(rs), dfx(rs))
s_condition, d_condition = cartesian_to_frenet1D(rs, rx, ry, rtheta, 2.0, 3.5)
plt.plot(snew, np.zeros(1000), 'y')
plt.plot(snew, np.zeros(1000) + 1.5, 'b')
plt.plot(snew, np.zeros(1000) - 1.5, 'b')
plt.scatter(s_condition, d_condition, marker='o', color='k')
plt.axis([0, 16, -8, 8])
plt.show()
