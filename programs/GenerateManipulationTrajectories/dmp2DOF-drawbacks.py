# Example of the 3 main drawbacks of the basic formulation of the DMPs-
# Using library https://github.com/studywolf/pydmps.git

import numpy as np
import matplotlib.pyplot as plt
from pydmps.dmp_discrete import DMPs_discrete
import math

def gaussian(x, mu, sig):
    return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))


dt = 0.001
dmp = DMPs_discrete(dt=dt, n_dmps=2, n_bfs=2) # 2 declopupled dmps(one for each dimension) with 20 GBFs(Gaussian Basic Fucntions)
time = np.linspace(0,1.0, math.ceil(1.0/dt))
path1 = 3*time
path2 = gaussian(time, 0.5, 0.3)
             
dmp.imitate_path(y_des=np.array([path1, path2]))

# Adapt trajectory to a goal to demonstrated trajectory goal                   
dmp.goal[0] = path1[-1]
dmp.goal[1] =  path2[-1]+0.05
y_track_good_goal, dy_track_good_goal, ddy_track_good_goal = dmp.rollout()


plt.figure(1)
plt.plot(path1,path2, 'k--', linewidth=4, label="Demostrated trajectory")
plt.plot(y_track_good_goal[:,0],y_track_good_goal[:,1], lw=2, label="imitated trajectory to new goal g=[-1.5,0.2]")
plt.plot(dmp.goal[0], dmp.goal[1], '*')

dmp.goal[0] = 0
dmp.goal[1] = 1
y_track_goal_close_start, dy_track_goal_close_start, ddy_track_goal_close_start = dmp.rollout()

plt.plot(y_track_goal_close_start[:,0],y_track_goal_close_start[:,1], lw=2, label="imitated trajectory start = goal, no movement generated")
# plt.plot(np.linspace(0, 1, path1.shape[0]),y_track_change_sign_goal_start, lw=2, label="g-x0 change sign, the trajectory inverts.")
plt.legend(loc="upper left")


plt.show()
