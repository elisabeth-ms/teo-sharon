import numpy as np
import matplotlib.pyplot as plt
from bolero.datasets import make_minimum_jerk 
from bolero.representation import DMPBehavior
import math
import random

def generateRandomColor():
    r = random.random()
    b = random.random()
    g = random.random()

    color = (r, g, b)
    return color

x0 = np.zeros(1)
g = np.ones(1)
g [0] = -1
dt = 0.001
execution_time = 1.0

X_demo = make_minimum_jerk(x0, g, execution_time, dt)[0] # Use make_minimum_jerk to create a demonstration trajectory of one dof.
time = np.linspace(0,execution_time, math.ceil(execution_time/dt)+1)
X_demo =  np.sin(time*3*np.pi/2)

X_demo = X_demo.reshape((1,X_demo.shape[0],1))
print(X_demo.shape)
# X_demo = make_minimum_jerk(x0, g, execution_time, dt)[0]
# print(X_demo.shape)



plt.figure()
plt.plot(np.linspace(0, 1, X_demo.shape[1]), X_demo[0,:], 'g--', linewidth=4, label="Demostrated trajectory")
for ni in range(2,12, 2):
    dmp = DMPBehavior(execution_time, dt, n_features=ni) # Can be used to optimize the weights of a DMP with a black box optimizer.
                                                     # Only the weights of the DMP will be optimized. We will use n_features gausssians.  
    dmp.init(3,3) #1*3 inputs and 1*3 outputs
    dmp.set_meta_parameters(["x0", "g"], [x0, g]) # Set the dmp metaparameters initial state x0 and goal state g.

    dmp.imitate(X_demo) # Learn weights of the DMP from a demonstration.


    plt.plot(np.linspace(0, 1, X_demo.shape[1]), dmp.trajectory()[0], '--', c=generateRandomColor(),label=str(ni) + "basis functions")

plt.legend(loc="upper left")
plt.show()
