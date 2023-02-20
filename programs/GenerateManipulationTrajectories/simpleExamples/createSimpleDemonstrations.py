# Script to generate demonstrations in cartesian space from one starting point to an ending point.  
# The demonstrations are generated to be used in the DMPs. 


import roboticslab_kinematics_dynamics as kd
import yarp
import time
import csv
import numpy as np
period = 50.0
#-- Locate the file with the kinematic chain DH parameters

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

# Get the starting position for our known joint configuration
q = [0,12,-50, -50, 40, -70, 30, -20]

print('--- Joint space configuration 1: expect Cartesian space position 0.5589834816707282, -0.487592285682244, 0.28958288280264544')
q_vector = yarp.DVector(q)
x_vector = yarp.DVector()
cartesianSolver.fwdKin(q_vector,x_vector);

aux_vector = [0.2,0.5,-0.1]
aux_qvector = q_vector

list_vectors = []
list_qvectors = []
list_qvectors.append(q_vector)
x_vector2 = x_vector
print('x_vector2: [%s]' % ', '.join(map(str, x_vector2)))
list_vectors.append(np.array([x_vector2[0],x_vector2[1], x_vector2[2], x_vector2[3], x_vector2[4], x_vector2[5]]))

for i in range(1,200):
        q_vector2 = yarp.DVector()
        x_vector2[0]+=aux_vector[0]/200.0
        x_vector2[1]+=aux_vector[1]/200.0
        x_vector2[2]+=aux_vector[2]/200.0
        if(cartesianSolver.invKin(x_vector2,aux_qvector, q_vector2)):
                aux_qvector=q_vector2
                # print('x_vector2: [%s]' % ', '.join(map(str, x_vector2)))
                # print(x_vector2[0])
                list_vectors.append(np.array([x_vector2[0],x_vector2[1], x_vector2[2], x_vector2[3], x_vector2[4], x_vector2[5]]))
                list_qvectors.append(q_vector2)


traj_number = 0
csv_file_write = open("trajectories/straight-line-cartesian-"+str(traj_number)+".csv", "w+")
writer = csv.writer(csv_file_write, dialect='excel')
for i in range(0, len(list_vectors)):
        rowData = [i, list_vectors[i][0],list_vectors[i][1],list_vectors[i][2],list_vectors[i][3],list_vectors[i][4],list_vectors[i][5]]
        # print(rowData)
        writer.writerow(rowData)
csv_file_write.close()

csv_file_write = open("trajectories/straight-line-joint-"+str(traj_number)+".csv", "w")
writer = csv.writer(csv_file_write, dialect='excel')
for i in range(0, len(list_qvectors)):
    rowData = [i, list_qvectors[i][0],list_qvectors[i][1],list_qvectors[i][2],list_qvectors[i][3],list_qvectors[i][4],list_qvectors[i][5],
               list_qvectors[i][6],list_qvectors[i][7]]
    writer.writerow(rowData)
csv_file_write.close()



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

start = time.time()
for i in range(0, len(list_qvectors)):
        for j in range(numJoints):
                IPositionDirect.setPosition(j, list_qvectors[i][j])
        time.sleep(period * 0.001 - ((time.time() - start) % (period * 0.001)))
     
        
#Now backwards
yarp.delay(0.5)
start = time.time()
for i in range(len(list_qvectors)-1, -1, -1):
        for j in range(numJoints):
                IPositionDirect.setPosition(j, list_qvectors[i][j])
        time.sleep(period * 0.001 - ((time.time() - start) % (period * 0.001)))


print('bye!')
