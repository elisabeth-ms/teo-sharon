import numpy as np
import matplotlib.pyplot as plt
from bolero.datasets import make_minimum_jerk 
from bolero.representation import DMPBehavior
import math
import random
from dmp_pp import dmp_cartesian


def generateRandomColor():
    r = random.random()
    b = random.random()
    g = random.random()

    color = (r, g, b)
    return color

x0 = np.zeros(1)
x0[0]= np.sin(0.0*3.5*np.pi/2)*15
g = np.ones(1)
g [0] = np.sin(1.0*3.5*np.pi/2)*15
dt = 0.001
execution_time = 1.0

X_demo = make_minimum_jerk(x0, g, execution_time, dt)[0] # Use make_minimum_jerk to create a demonstration trajectory of one dof.
time = np.linspace(0,execution_time, math.ceil(execution_time/dt)+1)
X_demo =  np.sin(time*3.5*np.pi/2)*15
gamma = np.transpose(np.array([time, X_demo, X_demo]))

X_demo = X_demo.reshape((1,X_demo.shape[0],1))
print(X_demo.shape)

K = 1000
alpha = 4.0
n_dim = 3
n_bfs = 50
dt = 0.001
tol = 0.05

MP_new = dmp_cartesian.DMPs_cartesian(n_dmps = n_dim, n_bfs = n_bfs, K = K, dt = dt, alpha_s = alpha, tol = tol, rescale='diagonal')

t = np.linspace(0,execution_time, math.ceil(execution_time/dt)+1)
x = t
y = np.sin(time*3.5*np.pi/2)*15
gamma = np.transpose(np.array([x, y, y*2]))
g_old = gamma[-1]

MP_new.imitate_path(x_des = gamma)

MP_new.x_goal = g_old

mp_new_high = MP_new.rollout()[0]

print(gamma.shape, g_old.shape)

plt.figure()
plt.plot(np.linspace(0, 1, X_demo.shape[1]), X_demo[0,:], 'g--', linewidth=4, label="Demostrated trajectory")
plt.plot(np.linspace(0, 1, X_demo.shape[1]), 2*X_demo[0,:], 'g--', linewidth=4, label="Demostrated trajectory")

for ni in range(10,50, 10):
    dmp = DMPBehavior(execution_time, dt, n_features=ni) # Can be used to optimize the weights of a DMP with a black box optimizer.
                                                     # Only the weights of the DMP will be optimized. We will use n_features gausssians.  
    dmp.init(3,3) #1*3 inputs and 1*3 outputs
    dmp.set_meta_parameters(["x0", "g"], [x0, g]) # Set the dmp metaparameters initial state x0 and goal state g.

    dmp.imitate(X_demo) # Learn weights of the DMP from a demonstration.


    plt.plot(np.linspace(0, 1, X_demo.shape[1]), dmp.trajectory()[0], '--', c=generateRandomColor(),label=str(ni) + "basis functions")

plt.plot(mp_new_high[:, 0], mp_new_high[:, 1], '--g', label='Park et al.')
plt.plot(mp_new_high[:, 0], mp_new_high[:, 2], '--g', label='Park et al.')
plt.legend(loc="upper left")
plt.show()
