from attr import s
import numpy as np
import matplotlib.pyplot as plt
from dmp_pp import dmp_cartesian
import math
import random
import logging

class JointLimitsDMP():
    def __init__(self, dt, execution_time,n_features, q_min, q_max, dmp_name, n_dmps):
        self.dt = dt
        self.execution_time = execution_time
        self.n_features = n_features
        self.n_dmps = n_dmps
        self.q_min = np.array(q_min)*np.pi/180
        self.q_min= np.expand_dims(self.q_min, axis=1)
        self.q_max = np.array(q_max)*np.pi/180
        self.q_max= np.expand_dims(self.q_max, axis=1)
        self.dmp_name = dmp_name
        self.y_0 = None
        self.gamma = None
        self.inv_gamma = None
        self.K = 1000
        self.alpha = 4.0
        self.tol = 0.05
        self.dmp = dmp_cartesian.DMPs_cartesian(n_dmps = self.n_dmps, n_bfs = self.n_features, K = self.K, dt = self.dt, alpha_s = self.alpha, tol = self.tol, rescale=None, T=self.execution_time)

        self.computeGammaY0()
    
    def computeGammaY0(self):
        self.y_0 = 1/2*(self.q_max+self.q_min)
        self.gamma = np.diagflat(0.5*(self.q_max-self.q_min))
        self.inv_gamma = np.linalg.inv(self.gamma)
    def transformTrajectoryToNewSpace(self, y_demo):
        e_demo_new_space = []
        for i in range(y_demo.shape[0]):
            y = y_demo[i,1:]*np.pi/180.0
            y = np.expand_dims(y, axis=1)
            e =  np.arctanh(np.dot(self.inv_gamma,(y-self.y_0)))
            e = np.append(y_demo[i,0], e)
            e_demo_new_space.append(e)
        
        e_demo_new_space = np.array(e_demo_new_space)
        e_demo_new_space = np.reshape(e_demo_new_space, (e_demo_new_space.shape[0],e_demo_new_space.shape[1]))
        return e_demo_new_space

    def transformJointsPositionToNewSpace(self,x):
        return np.arctanh(np.dot(self.inv_gamma,(x*np.pi/180.0-self.y_0)))
    
    def transformStateToOriginalSpace(self, e):
        return (np.dot(self.gamma,np.tanh(e))+self.y_0)*180.0/np.pi
    
    def transformTrajectoryToOriginalSpace(self, trajectory_new_space):
        trajectory_from_cdmp_demo = []
        for i in range(trajectory_new_space.shape[0]):
            e_trajectory =  np.expand_dims(trajectory_new_space[i,1:], axis = 1)
            q_trajectory = (np.dot(self.gamma,np.tanh(e_trajectory))+self.y_0)*180.0/np.pi
            q_trajectory = np.append(trajectory_new_space[i,0],q_trajectory)
            trajectory_from_cdmp_demo.append(q_trajectory)
        return np.array(trajectory_from_cdmp_demo)
    
    def setDMPDt(self, dt):
        self.dt = dt
        self.dmp = dmp_cartesian.DMPs_cartesian(n_dmps = self.n_dmps, n_bfs = self.n_features, K = self.K, dt = self.dt, alpha_s = self.alpha, tol = self.tol, rescale=None, T=self.execution_time)
        # DMPBehavior(self.execution_time, self.dt, self.n_features)
        print("DMP",self.dmp_name,"dt","set to ",self.dt)
    def setExecutionTime(self, execution_time):
        self.execution_time = execution_time
        self.dmp = dmp_cartesian.DMPs_cartesian(n_dmps = self.n_dmps, n_bfs = self.n_features, K = self.K, dt = self.dt, alpha_s = self.alpha, tol = self.tol, rescale=None, T=self.execution_time)
    
    def setNFeatures(self, n_features):
        self.n_features = n_features
        self.dmp =  dmp_cartesian.DMPs_cartesian(n_dmps = self.n_dmps, n_bfs = self.n_features, K = self.K, dt = self.dt, alpha_s = self.alpha, tol = self.tol, rescale=None, T= self.execution_time)
    
    
    def getArgs(self):
        d={'n_dmps': self.n_dmps, 'n_features': self.n_features, 'K': self.K, 'dt': self.dt, 'alpha': self.alpha, 'tol': self.tol, 'execution_time': self.execution_time}
        return d
    
    def imitateDemoInsideBounds(self, y_demo, x0, g):
        trajectory = []
        print("demo:", y_demo.shape, x0.shape)
        
        if y_demo.shape[1] != x0.shape[0]:
            print("demo trajectory first dimension is different to x0 first dimension")
            raise Exception("demo.rows != x0.rows")
        if y_demo.shape[1] != g.shape[0]:
            print("demo trajectory first dimension is different to g first dimension")
            raise Exception("demo.rows != g.rows")
        if y_demo.shape[0] != int(self.execution_time/self.dt):
            print(y_demo.shape[0], int(self.execution_time/self.dt))
            print("demo trajectory columns != execution_time/dt")
            raise Exception("demo.columns != execution_time/dt")
        print("x0: ", x0)
        print("g: ", g)
        
        print("Transform x0 to new space")
        x0_new_space = self.transformJointsPositionToNewSpace(np.expand_dims(x0[1:], axis=1))
        
        
        print("Transform g to new space")
        g_new_space = self.transformJointsPositionToNewSpace(np.expand_dims(g[1:], axis=1))
        print(self.transformStateToOriginalSpace(g_new_space))
        print("Transform trajectory y_demo to new space")
        e_demo_new_space = self.transformTrajectoryToNewSpace(y_demo)

        aux  = np.append(np.array(y_demo[0,0]),[x0_new_space])
   

        self.dmp.imitate_path(x_des = e_demo_new_space)
        self.dmp.x_goal = np.append(np.array(y_demo[-1,0]),[g_new_space])
        print("aux: ",self.dmp.x_goal)
        self.dmp.x_0 = np.append(np.array(y_demo[0,0]),[x0_new_space])
        
        trajectory_goal_demo_new_space = self.dmp.rollout()[0]


        # x0_new_space = np.squeeze(x0_new_space)
        # g_new_space = np.squeeze(g_new_space)
        #self.dmp.set_meta_parameters(["x0", "g"], [x0_new_space, g_new_space]) # Set the dmp metaparameters initial state x0 and goal state g.
        #self.dmp.imitate(e_demo_new_space) # Learn weights of the DMP from a demonstration.
        #trajectory_goal_demo_new_space = self.dmp.trajectory()
        print("trajectoty_to_goal: ",trajectory_goal_demo_new_space.shape)
        trajectory_goal_original_space = self.transformTrajectoryToOriginalSpace(trajectory_goal_demo_new_space)
        
        return True, trajectory_goal_original_space

    def plotDMPS(self, y_demo, trajectory_from_cdmp_demo):
        plt.figure()
        plotN = 1
        i=1
        print(y_demo.shape)
        print(trajectory_from_cdmp_demo.shape)
        for i in range(1,y_demo.shape[1]):
            plt.subplot(str(2)+str(4)+str(i))
            plt.xlabel("time (s)")
            plt.ylabel("Joint "+str(i)+" position (deg.)")
            plt.plot(np.linspace(0, self.execution_time, y_demo.shape[0]), y_demo[:,i], 'g--', linewidth=4, label="Demostrated trajectory")
            plt.plot(trajectory_from_cdmp_demo[:,0], trajectory_from_cdmp_demo[:,i], 'y--',label= "Demonstration CDMP")
            plt.plot((0,self.execution_time), (self.q_min[i-1]*180.0/np.pi, self.q_min[i-1]*180.0/np.pi), 'r--', label="Maximum limit")
            plt.plot((0,self.execution_time), (self.q_max[i-1]*180.0/np.pi, self.q_max[i-1]*180.0/np.pi), 'r--', label="Minimum limit")
            # time = np.linspace(0,1.0, num = 1000, endpoint= True)
            # plt.plot(time, -np.sin(time*3.0*np.pi/2)*30-20, 'k--')
            plt.legend(loc="upper left")

        plt.show()
        