print(__doc__)

import os
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from bolero.representation import CartesianDMPBehavior


import csv

def matrix_from_quaternion(q):
    w, x, y, z = q
    x2 = 2.0 * x * x
    y2 = 2.0 * y * y
    z2 = 2.0 * z * z
    xy = 2.0 * x * y
    xz = 2.0 * x * z
    yz = 2.0 * y * z
    xw = 2.0 * x * w
    yw = 2.0 * y * w
    zw = 2.0 * z * w

    R = np.array([[1.0 - y2 - z2,       xy - zw,       xz + yw],
                  [      xy + zw, 1.0 - x2 - z2,       yz - xw],
                  [      xz - yw,       yz + xw, 1.0 - x2 - y2]])

    return R


def plot_pose(ax, x, s=1.0, **kwargs):
    p = x[:3]
    R = matrix_from_quaternion(x[3:])
    for d, c in enumerate(["r", "g", "b"]):
        ax.plot([p[0], p[0] + s * R[0, d]],
                [p[1], p[1] + s * R[1, d]],
                [p[2], p[2] + s * R[2, d]], color=c, **kwargs)

    return ax


def plot_trajectory(ax, X, color="k"):
    ax.plot(X[:, 0], X[:, 1], X[:, 2], lw=2, color=color)
    for x in X[20:-20:20]:
        plot_pose(ax, x, s=0.03, lw=2, alpha=0.5)
    plot_pose(ax, X[0], s=0.05, lw=3)
    plot_pose(ax, X[-1], s=0.05, lw=3)

def getCartesianTrajectoryFromCsvFile(filename):
    trajectory = []
    with open(filename, newline='') as csvfile:
        spamreader = csv.reader(csvfile, delimiter=',', quotechar=' ')
        for row in spamreader:
            pose = [float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5]), float(row[6]), float(row[7])]
            trajectory.append(pose)
    return np.array(trajectory).T

trajectory = getCartesianTrajectoryFromCsvFile('trajectories/graspcup1/test-right-arm-motion-smooth1-optimized.csv')

execution_time = 1.0
dt = execution_time/(trajectory.shape[1]-1)
dmp = CartesianDMPBehavior(execution_time, dt, n_features=15)
dmp.init(7, 7)
print(trajectory.shape)
dmp.imitate(trajectory,0.0, False)
x0 = trajectory[:3,0]
q0 = trajectory[3:,0]

g =  np.array(trajectory[:3, trajectory.shape[1]-1], copy=True)
qg = trajectory[3:, trajectory.shape[1]-1]
print(x0)
dmp.set_meta_parameters(["x0", "g", "q0", "qg", "execution_time"], [x0, g, q0, qg, execution_time])
X = dmp.trajectory()


plt.figure(figsize=(18, 10))
ax = plt.subplot(221, projection="3d", aspect="auto")
plt.setp(ax, xlim=(-0.5, 0.5), ylim=(-0.5, 0.0), zlim=(0.5, 1.0),
         xlabel="X", ylabel="Y", zlabel="Z")


plot_trajectory(ax, trajectory.T, "k")
plot_trajectory(ax, X, "r")

ax = plt.subplot(223)
ax.plot(X[:, 0], label="X", c="r")
ax.plot(X[:, 1], label="Y", c="g")
ax.plot(X[:, 2], label="Z", c="b")
ax.legend(loc="upper right")
plt.setp(ax, xlabel="Step", ylabel="Position")

ax = plt.subplot(224)
dt = dmp.dt
ax.plot(np.diff(X[:, 0]) / dt, label="X", c="r")
ax.plot(np.diff(X[:, 1]) / dt, label="Y", c="g")
ax.plot(np.diff(X[:, 2]) / dt, label="Z", c="b")
ax.legend(loc="upper right")
plt.setp(ax, xlabel="Step", ylabel="Velocity")

plt.show()