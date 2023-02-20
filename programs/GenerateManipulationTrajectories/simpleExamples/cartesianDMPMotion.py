from dmp_pp import dmp_cartesian
import numpy as np
import matplotlib.pyplot as plt 
import math
import csv
import yarp
import time
import roboticslab_kinematics_dynamics as kd


# Get demonstration trajectory from csv file (t,x,y,z,roll,pitch,yaw). But we will use only t, x,y,z.
cartesianDemonstraionPathFile ='trajectories/straight-line-cartesian-0.csv'
gamma = []
orientations = []
with open(cartesianDemonstraionPathFile) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=',')
        line_count = 0
        for row in csv_reader:
            orientations.append(np.array([float(row[4]), float(row[5]), float(row[6])]))
            gamma.append(np.array([float(row[0])/200.0+1/200.0,float(row[1]),float(row[2]),float(row[3])]))
gamma = np.array(gamma)
print(gamma)

num_points = len(gamma)
print(num_points)

# Create a DMP object
# Parameters
execution_time = 1.0
K = 1000
alpha = 4.0
n_dim = 4 #t,x,y,z
n_bfs = 20
dt = execution_time/num_points
tol = 0.01
period = 50
# DMP initialization
DMP_cart = dmp_cartesian.DMPs_cartesian(n_dmps = n_dim, n_bfs = n_bfs, K = K, dt = dt, alpha_s = alpha, tol = tol, rescale='diagonal') 
#rescale ='diagonal' is the old formulation of the DMP. For some reason it is better to use this formulation with straight lines demonstration.
#rescale=None to use Park et al. formulation of the DMP

# Learning the DMP from the demonstration trajectory gamma
DMP_cart.imitate_path(x_des = gamma)






rf = yarp.ResourceFinder()
rf.setVerbose(False)
rf.setDefaultContext("testKdlSolverFromFile")
rf.setDefaultConfigFile("testKdlSolverFromFile.ini")
rf.setDefaultContext("kinematics")
kinematicsFileFullPath = rf.findFileByName("teo-trunk-rightArm-fetch.ini")

#-- Load the solver device with this configuration, view the interface

solverOptions = yarp.Property()
solverOptions.fromConfigFile(kinematicsFileFullPath)
solverOptions.put("device","KdlSolver")
solverOptions.put("ik", "nrjl")

solverOptions.fromString(
        "(mins (-30 -10.0 -98.1 -75.5 -80.1 -99.6 -80.4 -115.4))", False)
solverOptions.fromString(
         "(maxs (30 26.0 106 22.4 57 98.4 99.6 44.7))", False)
solverDevice = yarp.PolyDriver(solverOptions)

if not solverDevice.isValid():
    print("Cannot open the device!")
    raise SystemExit

cartesianSolver = kd.viewICartesianSolver(solverDevice) # view the actual interface


#Lets follow the path
name_device = "/teoSim/trunkAndRightArm"
print("configuring", name_device)
options = yarp.Property()
options.put('device', 'remote_controlboard')
options.put('remote', name_device)
options.put('local', name_device)
        
device = yarp.PolyDriver(options)
device.open(options)
if not device.isValid():
        print('Cannot open the device: ' + name_device +'!')
        raise SystemExit
print(name_device, "is Open!")

IEncoders = device.viewIEncoders()

if IEncoders == []:
        print(name_device +" Encoder interface NOT available.")
else:
        print(name_device + " Encoder interface available.")

IControlMode = device.viewIControlMode()
if IControlMode == []:
        print(name_device +" control mode interface NOT available.")
        raise SystemExit
else:
        print(name_device +" control mode interface available.")  
            
IPositionDirect = device.viewIPositionDirect()
if IPositionDirect == []:
        print(name_device + " position direct interface NOT available.")
        raise SystemExit
else:
        print(name_device + " position direct interface available.")

numJoints = IEncoders.getAxes()
print("Change to positionDirect mode.")
modes = yarp.IVector(numJoints, yarp.VOCAB_CM_POSITION_DIRECT)
if not IControlMode.setControlModes(modes):
        print("Unable to set to position mode.")
        raise SystemExit
else:
        print("Set to position mode.")
yarp.delay(5)


#Use the DMP to generate a new trajectory to the same goal

#getEncoders from the device and store it in a yarp vector, use fk to get the current position of the robot



currentQ = yarp.DVector(numJoints)
IEncoders.getEncoders(currentQ)
x0_vector = yarp.DVector()
cartesianSolver.fwdKin(currentQ,x0_vector);

DMP_cart.x_0 = np.array([0.0,x0_vector[0],x0_vector[1],x0_vector[2]])
print("start:", DMP_cart.x_0)
DMP_cart.x_goal = np.array([1,0.7,0.0, 0.15])
print("goal: ", DMP_cart.x_goal)
mp_g = DMP_cart.rollout()[0]





# start = time.time()
# for i in range(0, len(mp_g)):
#     q_goal = yarp.DVector()
#     x_vector_goal = [mp_g[i][1],mp_g[i][2],mp_g[i][3], x0_vector[3], x0_vector[4], x0_vector[5]]
#     x_goal = yarp.DVector(x_vector_goal)
#     gamma[i,1]
#     if(cartesianSolver.invKin(x_goal,currentQ, q_goal)):
#         currentQ=q_goal
#         for j in range(numJoints):
#             IPositionDirect.setPosition(j, q_goal[j])
#     time.sleep(period * 0.001 - ((time.time() - start) % (period * 0.001)))

# for j in range(numJoints):
#                 IPositionDirect.setPosition(j, list_qvectors[i][j])
#         time.sleep(period * 0.001 - ((time.time() - start) % (period * 0.001)))
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# plt.plot(gamma[:,1], gamma[:,2],gamma[:,3], 'b', label='learned traj.')
# plt.plot(mp_g_old[:, 1], mp_g_old[:, 2],mp_g_old[:, 3], '--g', label='Park et al.')
# plt.show()
