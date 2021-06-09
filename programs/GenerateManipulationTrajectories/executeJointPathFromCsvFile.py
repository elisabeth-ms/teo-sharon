import yarp
import kinematics_dynamics
import os
import csv
import PyKDL
import numpy as np


yarp.Network.init()

if yarp.Network.checkNetwork() != True:
    print('[error] Please try running yarp server')
    raise SystemExit

rf = yarp.ResourceFinder()
rf.setVerbose(True)
rf.setDefaultContext("myContext")
rf.setDefaultConfigFile("default.ini")

# trunkRightArmOptions = yarp.Property()
# trunkRightArmOptions.put('device','remote_controlboard')
# trunkRightArmOptions.put('remote','/teoSim/trunkAndRightArm')
# trunkRightArmOptions.put('local','/teoSim/trunkAndRightArm')

# trunkRightArmDevice = yarp.PolyDriver(trunkRightArmOptions)  # calls open -> connects
# trunkRightArmDevice.open(trunkRightArmOptions);
# if not trunkRightArmDevice.isValid():
#     print('Cannot open the device!')
#     raise SystemExit


rightArmOptions = yarp.Property()
rightArmOptions.put('device','remote_controlboard')
rightArmOptions.put('remote','/teoSim/rightArm')
rightArmOptions.put('local','/teoSim/rightArm')

rightArmDevice = yarp.PolyDriver(rightArmOptions)  # calls open -> connects
rightArmDevice.open(rightArmOptions);
if not rightArmDevice.isValid():
    print('Cannot open the device!')
    raise SystemExit

trunkOptions = yarp.Property()
trunkOptions.put('device','remote_controlboard')
trunkOptions.put('remote','/teoSim/trunk')
trunkOptions.put('local','/teoSim/trunk')


trunkDevice = yarp.PolyDriver(trunkOptions)  # calls open -> connects
trunkDevice.open(trunkOptions);
if not trunkDevice.isValid():
    print('Cannot open the device!')
    raise SystemExit

# trunkRightArmIEncoders = trunkRightArmDevice.viewIEncoders()

# if trunkRightArmIEncoders ==[]:
#     print("Right arm Encoder interface NOT available.")
# else:
#     print("Right arm Encoder interface available.")

rightArmIEncoders = rightArmDevice.viewIEncoders()

if rightArmIEncoders ==[]:
    print("Right arm Encoder interface NOT available.")
else:
    print("Right arm Encoder interface available.")

numRightArmJoints = rightArmIEncoders.getAxes()

trunkIEncoders = trunkDevice.viewIEncoders()

if trunkIEncoders ==[]:
    print("Trunk Encoder interface NOT available.")
else:
    print("Trunk Encoder interface available.")

numTrunkJoints = trunkIEncoders.getAxes()

# trunkRighArmIPositionDirect = trunkRightArmDevice.viewIPositionDirect()
# if trunkRighArmIPositionDirect ==[]:
#     print("Right arm position direct interface NOT available.")
# else:
#     print("Right arm position direct interface available.")

# numRightArmJoints = trunkRightArmIEncoders.getAxes()

# trunkRightArmIControlMode = trunkRightArmDevice.viewIControlMode()
# #ightArmIControlMode.setControlMode(numRightArmJoints, yarp.VOCAB_CM_POSITION_DIRECT)

# if trunkRightArmIEncoders == []:
#     print("Right arm control mode interface NOT available.")
# else:
#     print("Right arm control mode interface available.")
    
# trunkRightArmIPositionControl = trunkRightArmDevice.viewIPositionControl()

# if trunkRightArmIPositionControl == []:
#     print("Right arm position control interface NOT available")
# else:
#     print("Right arm position control interface available.")

rightArmIPositionControl = rightArmDevice.viewIPositionControl()

if rightArmIPositionControl == []:
    print("Right arm position control interface NOT available")
else:
    print("Right arm position control interface available.")
    


trunkIPositionControl = trunkDevice.viewIPositionControl()

if trunkIPositionControl == []:
    print("Trunk position control interface NOT available")
else:
    print("Trunk position control interface available.")
    

    
# trunkRightArmIControlLimits = trunkRightArmDevice.viewIControlLimits()
# if trunkRightArmIControlLimits == []:
#     print("Right arm control limits interface NOT available")
# else:
#     print("Right arm control limits interface available.")

    
rightArmIControlLimits = rightArmDevice.viewIControlLimits()
if rightArmIControlLimits == []:
    print("Right arm control limits interface NOT available")
else:
    print("Right arm control limits interface available.")
    

qrMin = yarp.Bottle()
qrMax = yarp.Bottle()
lmin = yarp.DVector(numRightArmJoints)
lmax = yarp.DVector(numRightArmJoints)

for joint in range(numRightArmJoints):
    rightArmIControlLimits.getLimits(joint, lmin, lmax)
    
    qrMin.addDouble(float(lmin[0]))
    qrMax.addDouble(float(lmax[0]))
    print("Joint ", joint, " min: ",lmin[0], "max: ", lmax[0])
print (qrMin.toString())


limits = "(mins ("+qrMin.toString()+")) (maxs ("+ qrMax.toString()+"))"
trunkRightArmSolverOptions = yarp.Property()
trunkRightKinPath = rf.findFileByName("teo-trunk-rightArm-fetch.ini")
trunkRightArmSolverOptions.fromConfigFile(trunkRightKinPath)
trunkRightArmSolverOptions.put("device", "KdlSolver")
trunkRightArmSolverOptions.put("ik","nrjl")
#rightArmSolverOptions.fromString("(mins (-98.1 -75.5 -80.1 -99.6 -80.4 -115.4))", False)
#rightArmSolverOptions.fromString("(maxs (106 22.4 57 98.4 99.6 44.7))", False)
trunkRightArmSolverOptions.fromString("(mins (-59.3 -20.4 -98.1 -75.5 -80.1 -99.6 -80.4 -115.4))", False)
trunkRightArmSolverOptions.fromString("(maxs (46.3 10.1 106 22.4 57 98.4 99.6 44.7))", False)
print("mins")
print(trunkRightArmSolverOptions.find("mins").toString())
print("maxs")
print(trunkRightArmSolverOptions.find("maxs").toString())
#rightArmSolverOptions.put("ik", "st")
trunkRightArmSolverDevice = yarp.PolyDriver(trunkRightArmSolverOptions)  # calls open -> connects
trunkRightArmSolverDevice.open(trunkRightArmSolverOptions);

if trunkRightArmSolverDevice == []:
    print("Right arm solver device interface NOT available")
else:
    print("Right arm solver device interface available.")

trunkRightArmICartesianSolver = kinematics_dynamics.viewICartesianSolver(trunkRightArmSolverDevice)
if trunkRightArmICartesianSolver == []:
    print("Right arm cartesian solver interface NOT available")
else:
    print("Right arm cartesian solver interface available.")


# currentQ = yarp.DVector(numRightArmJoints)

# if not trunkRightArmIEncoders.getEncoders(currentQ):
#     print("Failed getEncoders")
# for i in range(numRightArmJoints):
#     print(currentQ[i])

currentQ = yarp.DVector(numRightArmJoints)

if not rightArmIEncoders.getEncoders(currentQ):
    print("Failed getEncoders")
for i in range(numRightArmJoints):
    print(currentQ[i])

currentTrunkQ = yarp.DVector(numTrunkJoints)

if not trunkIEncoders.getEncoders(currentTrunkQ):
    print("Failed getEncoders")
for i in range(numTrunkJoints):
    print(currentQ[i])

qTraj = []

with open('trajectories/graspcup1/test-right-arm-motion-smooth3-joint.csv') as csv_file:
    csv_reader = csv.reader(csv_file, delimiter=',')
    line_count = 0
    for row in csv_reader:
        qTraj.append([float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5]), float(row[6]), float(row[7]), float(row[8])])
        print(qTraj[line_count])
        line_count += 1

    print(f'Processed {line_count} lines.')

for i in range(0,len(qTraj),1):
    rightArmIEncoders.getEncoders(currentQ)
    trunkIEncoders.getEncoders(currentTrunkQ)
    desireQ = yarp.DVector(numRightArmJoints+2)

    current_Q = yarp.DVector(numRightArmJoints+numTrunkJoints)
    for j in range(0, numRightArmJoints):
        current_Q[j+2] = currentQ[j]
    for j in range(numTrunkJoints):
        current_Q[j] = currentTrunkQ[j]

    x_current = yarp.DVector(6)
    trunkRightArmICartesianSolver.fwdKin(current_Q,x_current)
    diffSumSquare = 0
    # for i in range(len(x_current)):
    #     diffSumSquare += (x_current[i] - x_vector[i])*(x_current[i]-x_vector[i])
    # if diffSumSquare>0.0000001:
 
    # if(trunkRightArmICartesianSolver.invKin(x_vector, current_Q, desireQ)):
    #    print("i: ", i)
    print("i: ", i, "Q:",qTraj[i][0], qTraj[i][1], qTraj[i][2], qTraj[i][3], qTraj[i][4], qTraj[i][5], qTraj[i][6], qTraj[i][7])
    #    print("Desire Q",desireQ[0], desireQ[1], desireQ[2], desireQ[3], desireQ[4], desireQ[5], desireQ[6], desireQ[7])
    for joint in range(0,numRightArmJoints,1):
        rightArmIPositionControl.positionMove(joint,qTraj[i][joint+2])
    for joint in range(numTrunkJoints):
    # print(numTrunkJoints, joint, desireQ[joint])
        trunkIPositionControl.positionMove(joint, qTraj[i][joint])
    yarp.delay(0.5)
