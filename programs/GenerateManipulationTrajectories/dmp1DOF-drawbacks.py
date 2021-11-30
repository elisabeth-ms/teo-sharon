# Example of the 3 main drawbacks of the basic formulation of the DMPs-
# Using library https://github.com/studywolf/pydmps.git

import numpy as np
import matplotlib.pyplot as plt
from pydmps.dmp_discrete import DMPs_discrete
import math

def generateRandomColor():
    r = random.random()
    b = random.random()
    g = random.random()

    color = (r, g, b)
    return color
dt = 0.001
dmp = DMPs_discrete(dt=dt, n_dmps=1, n_bfs=20, w=np.zeros((1, 20)))
time = np.linspace(0,1.0, math.ceil(1.0/dt))
path1 = np.sin(time*3*np.pi/2)             
dmp.imitate_path(y_des=np.array([path1]))

# Adapt trajectory to a goal close to demonstrated trajectory goal                   
dmp.goal[0] = -1.2
y_track_good_goal, dy_track_good_goal, ddy_track_good_goal = dmp.rollout()

# Adapt trajectory to a goal equal to start. First drawback, no movement generated if goal=x0                
dmp.goal[0] = 0
y_track_same_goal_start, dy_track_same_goal_start, ddy_track_same_goal_start = dmp.rollout()

# g-x0 changes the sign, the trajectory is mirrored
dmp.y0[0] = 0.0                
dmp.goal[0] = 1.2
y_track_change_sign_goal_start, dy_track_change_sign_goal_start, ddy_track_change_sign_goal_start = dmp.rollout()


plt.figure(1)
plt.plot(np.linspace(0, 1, path1.shape[0]), path1, 'k--', linewidth=4, label="Demostrated trajectory")
plt.plot(np.linspace(0, 1, path1.shape[0]),y_track_good_goal, lw=2, label="imitated trajectory to new goal g")
plt.plot(np.linspace(0, 1, path1.shape[0]),y_track_same_goal_start, lw=2, label="imitated trajectory start = goal, no movement generated")
plt.plot(np.linspace(0, 1, path1.shape[0]),y_track_change_sign_goal_start, lw=2, label="g-x0 change sign, the trajectory inverts.")
plt.legend(loc="upper left")


plt.show()
