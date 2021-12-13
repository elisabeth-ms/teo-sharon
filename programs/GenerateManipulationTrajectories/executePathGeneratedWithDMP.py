import yarp
import kinematics_dynamics
import os
import csv
import PyKDL
import numpy as np
from bolero.representation import CartesianDMPBehavior

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
trunkRightArmSolverOptions.put("eps", 0.001)
trunkRightArmSolverOptions.put("maxIter", 10000)
#rightArmSolverOptions.fromString("(mins (-98.1 -75.5 -80.1 -99.6 -80.4 -115.4))", False)
#rightArmSolverOptions.fromString("(maxs (106 22.4 57 98.4 99.6 44.7))", False)
trunkRightArmSolverOptions.fromString("(mins (-59.3 -10.4 -98.1 -75.5 -80.1 -99.6 -80.4 -115.4))", False)
trunkRightArmSolverOptions.fromString("(maxs (46.3 10.1 106 22.4 57 98.4 99.6 44.7))", False)
print("mins")
print(trunkRightArmSolverOptions.find("mins").toString())
print("maxs")
print(trunkRightArmSolverOptions.find("maxs").toString())
#rightArmSolverOptions.put("ik", "st")
trunkRightArmSolverDevice = yarp.PolyDriver(trunkRightArmSolverOptions)  # calls open -> connects
trunkRightArmSolverDevice.open(trunkRightArmSolverOptions)

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


x = []
y = []
z = []
frames = []
qx = []
qy = []
qz = []
qw = []

model = 'model_dmp_approach1.yaml'
config = 'config_dmp_aproach1.yaml'

dmp = CartesianDMPBehavior(configuration_file=model)
dmp.init(7,7)
dmp.load_config(config)

x0 = np.array([0.12432408, -0.29592085, 0.68519397])
q0 = np.array([-0.57013293, 0.11130143, -0.47013887, 0.66447715])
q0 = q0/np.linalg.norm(q0)
g = np.array([  0.45566228, -0.11897716, 0.9401124])
qg = np.array([-0.2427928, -0.29033049, -0.44532354, 0.81144736])
g = g/np.linalg.norm(g)
execution_time = 1.0

dmp.set_meta_parameters(["x0", "g", "q0", "qg", "execution_time"], [x0, g, q0, qg, execution_time])
#Get the new generated trajectory from dmp.
trajectory = dmp.trajectory()

print(trajectory[len(trajectory)-1])

for pose in trajectory:
    x.append(pose[0])
    y.append(pose[1])
    z.append(pose[2])
    qx.append(pose[3])
    qy.append(pose[4])
    qz.append(pose[5])
    qw.append(pose[6])

next = False
i = 0
while(i<len(x)):
    rightArmIEncoders.getEncoders(currentQ)
    trunkIEncoders.getEncoders(currentTrunkQ)
    # print(x[i], y[i], z[i])
    pos = PyKDL.Vector(x[i],y[i],z[i]-0.894)
    norm = np.sqrt(qx[i]*qx[i]+qy[i]*qy[i]+qz[i]*qz[i]+qw[i]*qw[i])
    qx[i] = qx[i]/norm
    qy[i] = qy[i]/norm
    qz[i] = qz[i]/norm
    qw[i] = qw[i]/norm
    # print("norm: ", norm)
    rot = PyKDL.Rotation.Quaternion(qx[i], qy[i], qz[i], qw[i])    
    rot = rot*PyKDL.Rotation.RotX(-np.pi/2.0)
    rot = rot*PyKDL.Rotation.RotZ(-np.pi/2.0-np.pi/4.0)

    frame = PyKDL.Frame(rot, pos)
    rotVector = frame.M.GetRot()
    x_vector = yarp.DVector(6)
    x_vector[0] = x[i]
    x_vector[1] = y[i]
    x_vector[2] = z[i]-0.894
    
    x_vector[3] = rotVector.x()
    x_vector[4] = rotVector.y()
    x_vector[5] = rotVector.z()
    
    # x_vector[0] = 0.502685
    # x_vector[1] = -0.383609
    # x_vector[2] = 0.26869
    # x_vector[3] = -1.18141
    # x_vector[4] = 1.30545
    # x_vector[5] = -0.993912
    
    # print("x_vector")
    # print(x_vector[0], x_vector[1],x_vector[2], x_vector[3], x_vector[4], x_vector[5])
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
 
    if(trunkRightArmICartesianSolver.invKin(x_vector, current_Q, desireQ)):
        # print("i: ", i)
        # print("Current Q:",current_Q[0], current_Q[1], current_Q[2], current_Q[3], current_Q[4], current_Q[5], current_Q[6], current_Q[7])
        # print("Desire Q",desireQ[0], desireQ[1], desireQ[2], desireQ[3], desireQ[4], desireQ[5], desireQ[6], desireQ[7])

        for joint in range(0,numRightArmJoints,1):
            rightArmIPositionControl.positionMove(joint,desireQ[joint+2])
        for joint in range(numTrunkJoints):
            print(numTrunkJoints, joint, desireQ[joint])
            trunkIPositionControl.positionMove(joint, desireQ[joint])
        yarp.delay(0.35)
        i = i + 1
    else:

        if(trunkRightArmICartesianSolver.invKin(x_vector, current_Q, desireQ)):
                        # print("i: ", i)
                        # print("Current Q:",current_Q[0], current_Q[1], current_Q[2], current_Q[3], current_Q[4], current_Q[5], current_Q[6], current_Q[7])
                        # print("Desire Q",desireQ[0], desireQ[1], desireQ[2], desireQ[3], desireQ[4], desireQ[5], desireQ[6], desireQ[7])

            for joint in range(0,numRightArmJoints,1):
                rightArmIPositionControl.positionMove(joint,desireQ[joint+2])
            for joint in range(numTrunkJoints):
                trunkIPositionControl.positionMove(joint, desireQ[joint])
            yarp.delay(0.35)
        i = i + 1
                        
            
            
