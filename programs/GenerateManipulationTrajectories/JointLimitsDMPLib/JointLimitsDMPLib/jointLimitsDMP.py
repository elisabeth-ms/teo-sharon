from attr import s
import numpy as np
import matplotlib.pyplot as plt
from bolero.representation import DMPBehavior
import math
import random
import logging

class JointLimitsDMP():
    def __init__(self, dt, execution_time,n_features, q_min, q_max):
        self.dt = dt
        self.execution_time = execution_time
        self.n_features = n_features
        self.q_min = np.array(q_min)*np.pi/180
        self.q_min= np.expand_dims(self.q_min, axis=1)
        self.q_max = np.array(q_max)*np.pi/180
        self.q_max= np.expand_dims(self.q_max, axis=1)
        self.dmp = DMPBehavior(self.execution_time, self.dt, self.n_features)
        self.y_0 = None
        self.gamma = None
        self.inv_gamma = None
        self.computeGammaY0()
    
    def computeGammaY0(self):
        self.y_0 = 1/2*(self.q_max+self.q_min)
        self.gamma = np.diagflat(0.5*(self.q_max-self.q_min))
        self.inv_gamma = np.linalg.inv(self.gamma)
    def transformTrajectoryToNewSpace(self, y_demo):
        e_demo_new_space = []
        for i in range(len(y_demo[0])):
            y = y_demo[:,i]*np.pi/180.0
            y = np.expand_dims(y, axis=1)
            e =  np.arctanh(np.dot(self.inv_gamma,(y-self.y_0)))
            e_demo_new_space.append(e)
        
        e_demo_new_space = np.array(e_demo_new_space)
        print(e_demo_new_space.shape)
        
        e_demo_new_space = np.reshape(e_demo_new_space, (e_demo_new_space.shape[0],e_demo_new_space.shape[1]))
        e_demo_new_space = e_demo_new_space.T
        print(e_demo_new_space.shape)
        return e_demo_new_space

    def tranformJointsPositionToNewSpace(self,x):
        return np.arctanh(np.dot(self.inv_gamma,(x-self.y_0)))
    
    def transformStateToOriginalSpace(self, e):
        return np.dot(self.gamma,np.tanh(e))+self.y_0
    
    def transformTrajectoryToOriginalSpace(self, trajectory_new_space):
        trajectory_from_cdmp_demo = []
        for i in range(len(trajectory_new_space[0])):
            e_trajectory = np.expand_dims(trajectory_new_space[0][i,:], axis = 1)
            q_trajectory = np.dot(self.gamma,np.tanh(e_trajectory))+self.y_0
            trajectory_from_cdmp_demo.append(np.dot(self.gamma,np.tanh(e_trajectory))+self.y_0)
        return np.array(trajectory_from_cdmp_demo)
    
    def imitateDemoInsideBounds(self, y_demo, x0, g):
        trajectory = []
        if y_demo.shape[0] != x0.shape[0]:
            print("demo trajectory first dimension is different to x0 first dimension")
            raise Exception("demo.rows != x0.rows")
        if y_demo.shape[0] != g.shape[0]:
            print("demo trajectory first dimension is different to g first dimension")
            raise Exception("demo.rows != g.rows")
        if y_demo.shape[1] != self.execution_time/self.dt+1:
            print("demo trajectory columns != execution_time/dt+1")
            raise Exception("demo.columns != execution_time/dt+1")
        
        print("t")
        e_demo_new_space = self.tranformJointsPositionToNewSpace(y_demo)
        
        x0_new_space = self.tranformJointsPositionToNewSpace(x0)
        g_new_space = self.tranformJointsPositionToNewSpace(g)
        

        self.dmp.init(g.shape[0]*3, g.shape[0]*3)
        x0_new_space = np.squeeze(x0_new_space)
        g_new_space = np.squeeze(g_new_space)
        self.dmp.set_meta_parameters(["x0", "g"], [x0_new_space, g_new_space]) # Set the dmp metaparameters initial state x0 and goal state g.
        e_demo_new_space = np.expand_dims(e_demo_new_space, axis=2)
        self.dmp.imitate(e_demo_new_space) # Learn weights of the DMP from a demonstration.
        trajectory_goal_demo_new_space = self.dmp.trajectory()
        
        trajectory_goal_original_space = self.transformTrajectoryToOriginalSpace(trajectory_goal_demo_new_space)
        
        return True, trajectory_goal_original_space

    def plotDMPS(self, y_demo, trajectory_from_cdmp_demo):
        plt.figure()
        plotN = 1
        for i in range(y_demo.shape[0]):
            plt.subplot(str(y_demo.shape[0])+"1"+str(plotN))
            plt.xlabel("time (s)")
            plt.ylabel("Joint "+str(i)+" position (rad)")
            plt.plot(np.linspace(0, 1, y_demo.shape[1]), y_demo[i,:], 'g--', linewidth=4, label="Demostrated trajectory")
            plt.plot(np.linspace(0, self.execution_time, y_demo.shape[1]), trajectory_from_cdmp_demo[:,i], 'y--',label= "Demonstration CDMP")
            plt.plot((0,self.execution_time), (self.q_min[i], self.q_min[i]), 'r--', label="Maximum limit")
            plt.plot((0,self.execution_time), (self.q_max[i], self.q_max[i]), 'r--', label="Minimum limit")
            plt.legend(loc="upper left")
            plotN+=1

        plt.show()