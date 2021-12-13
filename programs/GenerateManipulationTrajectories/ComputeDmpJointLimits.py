import numpy as np
import matplotlib.pyplot as plt
from bolero.representation import DMPBehavior
import math
import random



qmin = np.array([-30.0, -8.5])
qmin= np.expand_dims(qmin, axis=1)
print(qmin.shape)
qmax = np.array([30.0, 14.0])
qmax= np.expand_dims(qmax, axis=1)
print(qmax.shape)

x0 = np.zeros(2)
x0 =  np.expand_dims(x0, axis=1)

print("x0 shape:", x0.shape)
g = np.ones(2)
g [0] = np.sin(1.0*3.5*np.pi/2)*15
g [1] =-np.sin(1*3*np.pi/2)*50
g =  np.expand_dims(g, axis=1)
dt = 0.001
execution_time = 1.0

time = np.linspace(0,execution_time, math.ceil(execution_time/dt)+1)
y_demo = np.sin(time*3.5*np.pi/2)*15
y_demo = np.vstack((y_demo,-np.sin(time*3*np.pi/2)*50))
print("shape:",y_demo.shape)


y_0 = 1/2*(qmax+qmin)
print("y_0 shape:",y_0.shape)
y_gamma = np.diagflat(0.5*(qmax-qmin))
inv_y_gamma = np.linalg.inv(y_gamma) 
print("inv_gamma:",inv_y_gamma)


#e = y_gamma*invert()*(np.tanh(y_demo-y_0)).invert()
print("x0:", x0)
print("y_0:", y_0)
print("atanh:", np.arctanh(x0-y_0))
x0_new_space = np.arctanh(np.dot(inv_y_gamma,(x0-y_0)))
print("x0_new_space: ", x0_new_space)
print("check x0",np.dot(y_gamma,np.tanh(x0_new_space))+y_0)
g_new_space = np.arctanh(np.dot(inv_y_gamma,(g-y_0)))
print("g_new_space: ", g_new_space)
print("g: ",g)
print("check g",np.dot(y_gamma,np.tanh(g_new_space))+y_0)


e_demo_new_space = []
for i in range(len(y_demo[0])):
    # print("1: ",np.arctanh(x0-y_0))
    # print("1-1:",x0)
    # print("2: ",np.arctanh(y_demo[:,i].T-y_0))
    # print("2-1: ",y_demo[:,0])
    y = y_demo[:,i]
    y = np.expand_dims(y, axis=1)
    # print("2-2: ",y)
    e = np.arctanh(np.dot(inv_y_gamma,(y-y_0)))
    print(g_new_space)
    print(e)
    # print(e)
    e_demo_new_space.append(e)
    # print(i)
print(y_demo.shape)

e_demo_new_space = np.array(e_demo_new_space)
e_demo_new_space = np.reshape(e_demo_new_space, (1001,2))
e_demo_new_space = e_demo_new_space.T 
print("e_Demo:", e_demo_new_space.shape)
print(e_demo_new_space[0,:])
print(e_demo_new_space[1,:])
print(g_new_space)
print(x0_new_space)

# Compute dmp in the original state space
ni = 10
dmp = DMPBehavior(execution_time, dt, n_features=ni) # Can be used to optimize the weights of a DMP with a black box optimizer.
                                                     # Only the weights of the DMP will be optimized. We will use n_features gausssians.  
dmp.init(6,6) #1*3 inputs and 1*3 outputs
print("x0.shape", x0.shape)
print("g.shape", g.shape)
print("y_demo.shape", y_demo.shape)

x0 = np.squeeze(x0)
g = np.squeeze(g)

dmp.set_meta_parameters(["x0", "g"], [x0, g]) # Set the dmp metaparameters initial state x0 and goal state g.
y_demo = np.expand_dims(y_demo, axis=2)
dmp.imitate(y_demo) # Learn weights of the DMP from a demonstration.
trajectory_goal_demo = dmp.trajectory()

# New goal 
g [0] = 10*np.pi/180
g [1] = -10*np.pi/180
dmp.set_meta_parameters(["x0", "g"], [x0, g]) # Set the dmp metaparameters initial state x0 and goal state g.
dmp.imitate(y_demo) # Learn weights of the DMP from a demonstration.
trajectory_new_goal_demo = dmp.trajectory()


# Compute dmp in the transformation state space
ni = 10
dmpNewSpace = DMPBehavior(execution_time, dt, n_features=ni) # Can be used to optimize the weights of a DMP with a black box optimizer.
                                                     # Only the weights of the DMP will be optimized. We will use n_features gausssians. 

dmpNewSpace.init(6,6) #1*3 inputs and 1*3 outputs
print("x0.shape", x0_new_space.shape)
print("g.shape", g_new_space.shape)
print("y_demo.shape", e_demo_new_space.shape)

x0_new_space = np.squeeze(x0_new_space)
g_new_space = np.squeeze(g_new_space)
dmpNewSpace.set_meta_parameters(["x0", "g"], [x0_new_space, g_new_space]) # Set the dmp metaparameters initial state x0 and goal state g.
e_demo_new_space = np.expand_dims(e_demo_new_space, axis=2)
dmpNewSpace.imitate(e_demo_new_space) # Learn weights of the DMP from a demonstration.
trajectory_goal_demo_new_space = dmpNewSpace.trajectory()

# print(trajectory_goal_demo_new_space[0].shape)
# print(trajectory_new_goal_demo[0].shape)

# From new space to original space

trajectory_from_cdmp_demo = []
for i in range(len(trajectory_goal_demo_new_space[0])):
    e_trajectory = np.expand_dims(trajectory_goal_demo_new_space[0][i,:], axis = 1)
    q_trajectory = np.dot(y_gamma,np.tanh(e_trajectory))+y_0
    trajectory_from_cdmp_demo.append(np.dot(y_gamma,np.tanh(e_trajectory))+y_0)
    # print(x0_new_space.shape)
    # print(e_trajectory.shape)
    # print("1:",e_trajectory)
    # print("2:",g_new_space)
    # print("3:",q_trajectory) 
    # print("4:",g)
trajectory_from_cdmp_demo = np.array(trajectory_from_cdmp_demo)



x0 = np.zeros(2)
x0 =  np.expand_dims(x0, axis=1)
g = np.ones(2)
g [0] = 10*np.pi/180
g [1] = -10*np.pi/180
g =  np.expand_dims(g, axis=1)

x0_new_space = np.arctanh(np.dot(inv_y_gamma,(x0-y_0)))
g_new_space = np.arctanh(np.dot(inv_y_gamma,(g-y_0)))

x0_new_space = np.squeeze(x0_new_space)
g_new_space = np.squeeze(g_new_space)
dmpNewSpace.set_meta_parameters(["x0", "g"], [x0_new_space, g_new_space]) # Set the dmp metaparameters initial state x0 and goal state g.
dmpNewSpace.imitate(e_demo_new_space) # Learn weights of the DMP from a demonstration.
trajectory_new_goal_demo_new_space = dmpNewSpace.trajectory()

# print(trajectory_goal_demo_new_space[0].shape)
# print(trajectory_new_goal_demo[0].shape)

# From new space to original space

trajectory_from_cdmp_demo_new_goal = []
for i in range(len(trajectory_new_goal_demo_new_space[0])):
    e_trajectory = np.expand_dims(trajectory_new_goal_demo_new_space[0][i,:], axis = 1)
    q_trajectory = np.dot(y_gamma,np.tanh(e_trajectory))+y_0
    trajectory_from_cdmp_demo_new_goal.append(np.dot(y_gamma,np.tanh(e_trajectory))+y_0)
    # print(x0_new_space.shape)
    # print(e_trajectory.shape)
    # print("1:",e_trajectory)
    # print("2:",g_new_space)
    # print("3:",q_trajectory) 
    # print("4:",g)
trajectory_from_cdmp_demo_new_goal = np.array(trajectory_from_cdmp_demo_new_goal)



plt.figure()
plt.subplot(211)
plt.xlabel("time (s)")
plt.ylabel("Joint 0 position (rad)")
plt.plot(np.linspace(0, 1, y_demo.shape[1]), y_demo[0,:], 'g--', linewidth=4, label="Demostrated trajectory")
plt.plot(np.linspace(0, execution_time, y_demo.shape[1]), trajectory_goal_demo[0][:,0], 'b--',label= "Demontration DMP")
plt.plot(np.linspace(0, execution_time, y_demo.shape[1]), trajectory_new_goal_demo[0][:,0], 'k--',label= "Adapt to new goal DMP")
plt.plot(np.linspace(0, execution_time, y_demo.shape[1]), trajectory_from_cdmp_demo[:,0], 'y--',label= "Demonstration CDMP")
plt.plot(np.linspace(0, execution_time, y_demo.shape[1]), trajectory_from_cdmp_demo_new_goal[:,0], '--',color="darkviolet",label= "Adapt to new goal CDMP")

plt.plot((0,execution_time), (qmin[0], qmin[0]), 'r--', label="Maximum limit")
plt.plot((0,execution_time), (qmax[0], qmax[0]), 'r--', label="Minimum limit")
plt.legend(loc="upper left")

plt.subplot(212)
plt.xlabel("time (s)")
plt.ylabel("Joint 1 position (rad)")
plt.plot(np.linspace(0, execution_time, y_demo.shape[1]), y_demo[1,:], 'g--', linewidth=4, label="Demostrated trajectory")
plt.plot(np.linspace(0, execution_time, y_demo.shape[1]), trajectory_new_goal_demo[0][:,1], 'b--',label="demontration DMP")
plt.plot(np.linspace(0, execution_time, y_demo.shape[1]), trajectory_new_goal_demo[0][:,1], 'k--',label= "adapt to new goal DMP")
plt.plot(np.linspace(0, execution_time, y_demo.shape[1]), trajectory_from_cdmp_demo[:,1], 'y--',label= "demonstration CDMP")
plt.plot(np.linspace(0, execution_time, y_demo.shape[1]), trajectory_from_cdmp_demo_new_goal[:,1], '--',color="darkviolet",label= "Adapt to new goal CDMP")
plt.plot((0,execution_time), (qmin[1], qmin[1]), 'r--',  label="Maximum limit")
plt.plot((0,execution_time), (qmax[1], qmax[1]), 'r--', label="Minimum limit")
plt.legend(loc="upper left")


    
plt.show()