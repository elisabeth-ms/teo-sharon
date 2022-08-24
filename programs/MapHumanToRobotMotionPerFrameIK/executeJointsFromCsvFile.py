
#!/usr/bin/env python3

#This code executes joint Path from a csv file. 
from tkinter import E
import rospy
from nav_msgs.msg import Path
from std_msgs.msg import String
from geometry_msgs.msg import Pose, PoseStamped, Point
from visualization_msgs.msg import MarkerArray, Marker
import yarp
import kinematics_dynamics
import os
import csv
import PyKDL
import numpy as np
import time
import copy
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import sys

robot='/teoSim'
posesPathFile = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba1-smoothed-link-adj.csv"
jointPosesCsvFile = csvFile = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba1-q-positions-optimization.csv";


handPoses = []
wristPoses = []
elbowPoses = []
shoulderPoses = []
neckPoses = []
trunkPoses = []
def getJointPosesCsvFile(pathFile):
    qTraj = []
    with open(pathFile) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=' ')
        line_count = 0
        for row in csv_reader:
            qPose = [float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5]), float(row[6]), float(row[7]), float(row[8])]
            qTraj.append(qPose)
            line_count += 1
        print(f'Processed {line_count} lines.')
    return qTraj

def getPosesCsvFile(pathFile):
    with open(pathFile) as csv_file:
        csv_reader = csv.reader(csv_file, delimiter=' ')
        line_count = 0
        for row in csv_reader:
            handPose = np.array([float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5]), float(row[6]), float(row[7])])
            print(handPose)
            wristPose = np.array([float(row[8]), float(row[9]), float(row[10]), float(row[11]), float(row[12]), float(row[13]), float(row[14])])           
            elbowPose = np.array([float(row[15]), float(row[16]), float(row[17]), float(row[18]), float(row[19]), float(row[20]), float(row[21])])
            shoulderPose = np.array([float(row[22]), float(row[23]), float(row[24]), float(row[25]), float(row[26]), float(row[27]), float(row[28])])
            neckPose = np.array([float(row[29]), float(row[30]), float(row[31]), float(row[32]), float(row[33]), float(row[34]), float(row[35])])
            trunkPose = np.array([float(row[36]), float(row[37]), float(row[38]), float(row[39]), float(row[40]), float(row[41]), float(row[42])])
            handPoses.append(handPose)
            wristPoses.append(wristPose)
            elbowPoses.append(elbowPose)
            shoulderPoses.append(shoulderPose)
            neckPoses.append(neckPose)
            trunkPoses.append(trunkPose)

        line_count += 1

        print(f'Processed {line_count} lines.')

       
def updateMarkerEndEffector(index):
    marker = Marker()
    marker.id = 5 
    marker.header.frame_id = "waist"
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD 
    marker.scale.x = 0.1
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.b = 0.0
    marker.color.r = 1.0
    marker.pose.orientation.w=1.0
    marker.pose.position.x = handPoses[index][0]
    marker.pose.position.y = handPoses[index][1]
    marker.pose.position.z =  handPoses[index][2]
    marker.points = []

    
    marker.points = []

    p3 = Point()
    p3.x = handPoses[index][0]
    p3.y = handPoses[index][1]
    p3.z = handPoses[index][2]
    print("index: ", index)
    print("p.x: ", p3.x, "p3.y: ", p3.y, "p3.z: ", p3.z)
    marker.points.append(p3)
    
    return marker 

def updateHumanMarkersAdj(index):
    markerArray = MarkerArray()
    marker = Marker()
    markerJoints = Marker()
    markerJoints.id = 2 
    markerJoints.header.frame_id = "waist"
    markerJoints.type = Marker.POINTS
    markerJoints.action = Marker.ADD 
    markerJoints.scale.x = 0.05
    markerJoints.scale.y = 0.05
    markerJoints.scale.z = 0.05
    markerJoints.color.a = 1.0
    markerJoints.color.b = 0.5
    markerJoints.color.r = 0.5
    markerJoints.pose.orientation.w=1.0
    markerJoints.pose.position.x = 0
    markerJoints.pose.position.y = 0
    markerJoints.pose.position.z = 0
    marker.id = 0 
    marker.header.frame_id = "waist"
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD 
    marker.scale.x = 0.01
    marker.color.a = 1.0
    marker.color.b = 0.5
    marker.color.r = 0.5
    marker.pose.orientation.w=1.0
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    
    marker.points = []
    markerJoints.points = []
        
    p_hip = Point()
    p_hip.x = trunkPoses[index][0]
    p_hip.y = trunkPoses[index][1]
    p_hip.z = trunkPoses[index][2]
    marker.points.append(p_hip)
    markerJoints.points.append(p_hip)
    
    p_neck = Point()
    p_neck.x = neckPoses[index][0]
    p_neck.y = neckPoses[index][1]
    p_neck.z = neckPoses[index][2]
    marker.points.append(p_neck)
    markerJoints.points.append(p_neck)
    
    p = Point()
    p.x = shoulderPoses[index][0]
    p.y = shoulderPoses[index][1]
    p.z = shoulderPoses[index][2]
    marker.points.append(p)
    markerJoints.points.append(p)
    
    p1 = Point()
    p1.x = elbowPoses[index][0]
    p1.y = elbowPoses[index][1]
    p1.z = elbowPoses[index][2]
    marker.points.append(p1)
    markerJoints.points.append(p1)
    
    p2 = Point()
    p2.x = wristPoses[index][0]
    p2.y = wristPoses[index][1]
    p2.z = wristPoses[index][2]
    marker.points.append(p2)
    markerJoints.points.append(p2)

    p3 = Point()
    p3.x =  handPoses[index][0]
    p3.y = handPoses[index][1]
    p3.z = handPoses[index][2]
    print("index: ", index)
    print("p.x: ", p3.x, "p3.y: ", p3.y, "p3.z: ", p3.z)
    marker.points.append(p3)
    markerJoints.points.append(p3)

    markerArray.markers.append(marker)#add linestrip to markerArray
    markerArray.markers.append(markerJoints)#add linestrip to markerArray
    
    return markerArray

class yarpDevice():
    def __init__(self, name_device):
        print("Yarp device: "+ name_device)
     # Right Arm device
        self.options = yarp.Property()
        self.device = None
        self.IEncoders = None
        self.ArmJoints = 0
        self.IPositionControl = None
        self.IPositionControl = None
        self.IControlMode = None
        self.IPositionDirect = None
        self.IRemoteVariables = False
        self.configureDevice(name_device)
    
    def configureDevice(self, name_device):
        print("configuring", name_device)
        self.options = yarp.Property()
        self.options.put('device', 'remote_controlboard')
        self.options.put('remote', name_device)
        self.options.put('local', name_device)
        
        self.device = yarp.PolyDriver(self.options)
        self.device.open(self.options)
        if not self.device.isValid():
            print('Cannot open the device: ' + name_device +'!')
            raise SystemExit
        print(name_device, "is Open!")

        self.IEncoders = self.device.viewIEncoders()

        if self.IEncoders == []:
            print(name_device +" Encoder interface NOT available.")
        else:
            print(name_device + " Encoder interface available.")
            
        self.IPositionDirect = self.device.viewIPositionDirect()
        if self.IPositionDirect == []:
            print(name_device + " position direct interface NOT available.")
            raise SystemExit
        else:
            print(name_device + " position direct interface available.")
        
        self.IPositionControl = self.device.viewIPositionControl()

        if self.IPositionControl == []:
            print(name_device + " position control interface NOT available")
        else:
            print(name_device + " position control interface available.")
            
        self.IControlMode = self.device.viewIControlMode()
        if self.IControlMode == []:
            print(name_device + " control mode interface NOT available.")
            raise SystemExit
        else:
            print(name_device + " control mode interface available.")
            
        self.IControlLimits = self.device.viewIControlLimits()
        if self.IControlLimits == []:
            print(name_device + " control limits interface NOT available")
        else:
            print(name_device + " control limits interface available.")

# def vectorToFrame(x):

#     f = PyKDL.Frame()

#     f.p.x(x[0])
#     f.p.y(x[1])
#     f.p.z(x[2])

#     rotvec = PyKDL.Vector(x[3], x[4], x[5])
#     f.M = PyKDL.Rotation.Rot(rotvec, rotvec.Norm())
#     return f

# def updateHumanMarkers(index):
#     markerArray = MarkerArray()
#     marker = Marker()
#     markerJoints = Marker()
#     markerJoints.id = 2 
#     markerJoints.header.frame_id = "waist"
#     markerJoints.type = Marker.POINTS
#     markerJoints.action = Marker.ADD 
#     markerJoints.scale.x = 0.05
#     markerJoints.scale.y = 0.05
#     markerJoints.scale.z = 0.05
#     markerJoints.color.a = 1.0
#     markerJoints.color.b = 0.5
#     markerJoints.color.r = 0.5
#     markerJoints.pose.orientation.w=1.0
#     markerJoints.pose.position.x = 0
#     markerJoints.pose.position.y = 0
#     markerJoints.pose.position.z = 0
#     marker.id = 0 
#     marker.header.frame_id = "waist"
#     marker.type = Marker.LINE_STRIP
#     marker.action = Marker.ADD 
#     marker.scale.x = 0.01
#     marker.color.a = 1.0
#     marker.color.b = 0.5
#     marker.color.r = 0.5
#     marker.pose.orientation.w=1.0
#     marker.pose.position.x = 0
#     marker.pose.position.y = 0
#     marker.pose.position.z = 0
#     p = Point()
#     marker.points = []
#     p.x = x_shoulder[index]
#     p.y = y_shoulder[index]
#     p.z = z_shoulder[index]-trunkHeight
#     marker.points.append(p)
#     markerJoints.points = []
#     markerJoints.points.append(p)
    
#     p1 = Point()
#     p1.x = x_elbow[index]
#     p1.y = y_elbow[index]
#     p1.z = z_elbow[index]-trunkHeight
#     marker.points.append(p1)
#     markerJoints.points.append(p1)

    
#     p2 = Point()
#     p2.x = x_wrist[index]
#     p2.y = y_wrist[index]
#     p2.z = z_wrist[index]-trunkHeight
#     marker.points.append(p2)
#     markerJoints.points.append(p2)

#     p3 = Point()
#     p3.x = x[index]
#     p3.y = y[index]
#     p3.z = z[index]-trunkHeight
#     marker.points.append(p3)
#     markerJoints.points.append(p3)

#     markerArray.markers.append(marker)#add linestrip to markerArray
#     markerArray.markers.append(markerJoints)#add linestrip to markerArray
    
#     return markerArray

def plotTrajectories(jointsTrajectory):
    arr = np.asarray(jointsTrajectory)
    t = np.linspace(0,len(jointsTrajectory)-1, len(jointsTrajectory))
    print(t)

    fig, ax = plt.subplots(8, sharex=True)
    plt.subplots_adjust(left=0.15, right=0.95, top=0.95, bottom=0.05)
        # set axes labels
        # ax[0].set_xlabel("num point")
    ax[0].set_ylabel("Axial Trunk [deg]", rotation=0, ha="right")
    ax[1].set_ylabel("Frontal Trunk [deg]",  rotation=0, ha="right")
        
    ax[2].set_ylabel("Frontal Right Shoulder [deg]", rotation=0, ha="right")
    ax[3].set_ylabel("Sagittal Right Shoulder [deg]",rotation=0, ha="right")
    ax[4].set_ylabel("Axial Right Shoulder [deg]",
                             rotation=0, ha="right")
    ax[5].set_ylabel("Frontal Right Elbow [deg]",
                             rotation=0, ha="right")
    ax[6].set_ylabel("Axial Right Wrist [deg]", rotation=0, ha="right")
    ax[7].set_ylabel("Frontal Right Wrist [deg]",
                             rotation=0, ha="right")

    fig.suptitle("Joints trajectories.")
    for i in range(8):
        ax[i].grid()
        ax[i].set_xlim(0, len(jointsTrajectory))
        ax[i].yaxis.set_major_locator(MaxNLocator(5))
 
    ax[0].plot(t, arr[:, 0], label='q0', color='skyblue')


    ax[1].plot(t, arr[:, 1], label='q1')


    ax[2].plot(t, arr[:, 2], label='q2')


    ax[3].plot(t, arr[:, 3], label='q3')


    ax[4].plot(t, arr[:, 4], label='q4')


    ax[5].plot(t, arr[:, 5], label='q5')


    ax[6].plot(t, arr[:, 6], label='q6')


    ax[7].plot(t, arr[:, 7], label='Joint trajectory')

        
    ax[7].xaxis.set_ticks(np.linspace(0, len(jointsTrajectory)-1, 10))

    handles, labels = ax[7].get_legend_handles_labels()
    fig.legend(handles, labels, loc='upper right')

    plt.show()
    plt.figure().clear()
    plt.close()
    
def main():
    yarp.Network.init()

    if yarp.Network.checkNetwork() != True:
        print('[error] Please try running yarp server')
        raise SystemExit
    
    rospy.init_node('execute_joints_path', anonymous=True)
    pub_marker_array_adj = rospy.Publisher('link_lenght_adjusted_markers', MarkerArray, queue_size=10)
    pub_end_effector = rospy.Publisher('end_effector', Marker, queue_size=10)
    pub_Hand_path = rospy.Publisher('hand_path', Path, queue_size=10)
    
    


    rf = yarp.ResourceFinder()
    rf.setVerbose(True)
    rf.setDefaultContext("myContext")
    rf.setDefaultConfigFile("default.ini")


    print("Configuring rightArm device")
    rightArmYarpDevice= yarpDevice(robot+"/rightArm")
    numRightArmJoints = rightArmYarpDevice.IEncoders.getAxes()
    print("rightArm: ", numRightArmJoints)
    
    trunkYarpDevice = yarpDevice(robot+"/trunk")

    numTrunkJoints = trunkYarpDevice.IEncoders.getAxes()
    print("trunk: ", numTrunkJoints)
    qrMin = yarp.Bottle()
    qrMax = yarp.Bottle()
    lmin = yarp.DVector(numTrunkJoints)
    lmax = yarp.DVector(numTrunkJoints)
    for i in range(numTrunkJoints):
        trunkYarpDevice.IControlLimits.getLimits(i, lmin, lmax)
        qrMin.addDouble(lmin[0])
        qrMax.addDouble(lmax[0])
        print("joint ", i, " limits: ", lmin[0], lmax[0])
        
    lmin = yarp.DVector(numRightArmJoints)
    lmax = yarp.DVector(numRightArmJoints)
    for joint in range(numRightArmJoints):
        rightArmYarpDevice.IControlLimits.getLimits(joint, lmin, lmax)
        qrMin.addDouble(float(lmin[0]))
        qrMax.addDouble(float(lmax[0]))
        print("Joint ", joint, " min: ", lmin[0], "max: ", lmax[0])
    print(qrMin.toString())


    limits = "(mins ("+qrMin.toString()+")) (maxs (" + qrMax.toString()+"))"
    trunkRightArmSolverOptions = yarp.Property()
    trunkRightKinPath = rf.findFileByName("teo-trunk-rightArm-fetch.ini")
    trunkRightArmSolverOptions.fromConfigFile(trunkRightKinPath)
    trunkRightArmSolverOptions.put("device", "KdlSolver")
    trunkRightArmSolverOptions.put("ik", "nrjl")
    trunkRightArmSolverOptions.put("eps",0.1)
    trunkRightArmSolverOptions.put("maxIter",10000)
    trunkRightArmSolverOptions.put("lambda",0.001)

    # rightArmSolverOptions.fromString("(mins (-98.1 -75.5 -80.1 -99.6 -80.4 -115.4))", False)
    # rightArmSolverOptions.fromString("(maxs (106 22.4 57 98.4 99.6 44.7))", False)
    trunkRightArmSolverOptions.fromString(
        "(mins (-59.3 -20.4 -98.1 -75.5 -80.1 -99.6 -80.4 -115.4))", False)
    trunkRightArmSolverOptions.fromString(
        "(maxs (46.3 10.1 106 22.4 57 98.4 99.6 44.7))", False)
    print("mins")
    print(trunkRightArmSolverOptions.find("mins").toString())
    print("maxs")
    print(trunkRightArmSolverOptions.find("maxs").toString())
    # rightArmSolverOptions.put("ik", "st")
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



    currentQ = yarp.DVector(numRightArmJoints)

    if not rightArmYarpDevice.IEncoders.getEncoders(currentQ):
        print("Failed getEncoders")
    for i in range(numRightArmJoints):
        print(currentQ[i])

    currentTrunkQ = yarp.DVector(numTrunkJoints)

    if not trunkYarpDevice.IEncoders.getEncoders(currentTrunkQ):
        print("Failed getEncoders")
    for i in range(numTrunkJoints):
        print(currentQ[i])

    getPosesCsvFile(posesPathFile)
    
    qTraj = getJointPosesCsvFile(jointPosesCsvFile)
    # print("qTraj: ", qTraj)
        # Move to the first position
    plotTrajectories(qTraj)
    
    hand_path = Path()
    hand_path.header.frame_id = 'waist'
    for i in range(len(handPoses)):
        pose = PoseStamped()
        pose.pose.position.x = handPoses[i][0]
        pose.pose.position.y = handPoses[i][1]
        pose.pose.position.z = handPoses[i][2]
        pose.pose.orientation.x = handPoses[i][3]
        pose.pose.orientation.y = handPoses[i][4]
        pose.pose.orientation.z = handPoses[i][5]
        pose.pose.orientation.w = handPoses[i][6]
        hand_path.poses.append(pose)
    
    
    hand_path.header.stamp = rospy.Time.now()
    pub_Hand_path.publish(hand_path)
    
    
    # # Suppose initial joints position to zero
    # current_Q = yarp.DVector(numRightArmJoints+numTrunkJoints)
    # for j in range(0, numRightArmJoints):
    #     current_Q[j+2] = 0
    # for j in range(numTrunkJoints):
    #     current_Q[j] = 0
    # qTraj1 = []
    # for i in range(10):
    #     try:
    #         pos = PyKDL.Vector(handPoses[i][0],handPoses[i][1],handPoses[i][2])
    #         norm = np.sqrt(handPoses[i][3]*handPoses[i][3]+handPoses[i][4]*handPoses[i][4]+handPoses[i][5]*handPoses[i][5]+handPoses[i][6]*handPoses[i][6])
    #         handPoses[i][3] = handPoses[i][3]/norm
    #         handPoses[i][4] = handPoses[i][4]/norm
    #         handPoses[i][5] = handPoses[i][5]/norm
    #         handPoses[i][6] = handPoses[i][6]/norm
    #         norm = np.sqrt(handPoses[i][3]*handPoses[i][3]+handPoses[i][4]*handPoses[i][4]+handPoses[i][5]*handPoses[i][5]+handPoses[i][6]*handPoses[i][6])

    #         # qx[i] = qx[i]/norm
    #         # qy[i] = qy[i]/norm
    #         # qz[i] = qz[i]/norm
    #         # qw[i] = qw[i]/norm
    #         print("norm: ", norm)
    #         rot = PyKDL.Rotation.Quaternion(handPoses[i][3], handPoses[i][4],handPoses[i][5], handPoses[i][6])    
    #         # rot = rot*PyKDL.Rotation.RotX(-np.pi/2.0)
    #         # rot = rot*PyKDL.Rotation.RotZ(-np.pi/2.0-np.pi/4)

    #         frame = PyKDL.Frame(rot, pos)
    #         rotVector = frame.M.GetRot()
    #         x_vector = yarp.DVector(6)
    #         x_vector[0] = handPoses[i][0]
    #         x_vector[1] = handPoses[i][1]
    #         x_vector[2] = handPoses[i][2]
            
    #         x_vector[3] = rotVector.x()
    #         x_vector[4] = rotVector.y()
    #         x_vector[5] = rotVector.z()
    #         desireQ = yarp.DVector(numRightArmJoints+numTrunkJoints)
    #         if(trunkRightArmICartesianSolver.invKin(x_vector, current_Q, desireQ)):
    #             print(desireQ[0]," ",desireQ[1]," ",desireQ[2]," ",desireQ[3]," ",desireQ[4]," ",desireQ[5]," ",desireQ[6]," ",desireQ[7])
    #             qPose = [desireQ[0],desireQ[1],desireQ[2],desireQ[3],desireQ[4],desireQ[5],desireQ[6],desireQ[7]]
    #             qTraj1.append(qPose)
    #             current_Q = desireQ
    #     except KeyboardInterrupt:
    #         sys.exit()
    # plotTrajectories(qTraj1)
    
    trunkModes = yarp.IVector(numTrunkJoints, yarp.VOCAB_CM_POSITION)
    if not trunkYarpDevice.IControlMode.setControlModes(trunkModes):
        print("Unable to set trunk  to position mode.")
        raise SystemExit
    else:
        print("Trunk set to position mode.")

    
    rightArmModes = yarp.IVector(numRightArmJoints, yarp.VOCAB_CM_POSITION)
    if not rightArmYarpDevice.IControlMode.setControlModes(rightArmModes):
        print("Unable to set right arm  to position  mode.")
        raise SystemExit
    else:
        print("Right arm set to position mode.")    
    yarp.delay(5.0)

    
    # x_rightArm_trunk = yarp.DVector(6)
    # Q_eeff = yarp.DVector(numTrunkJoints+numRightArmJoints)
    # for j in range(numRightArmJoints+numTrunkJoints):
    #     Q_eeff[j] = qTraj[0][j]
    # trunkRightArmICartesianSolver.fwdKin(Q_eeff, x_rightArm_trunk)
    # print("x_rightArm_trunk: ", x_rightArm_trunk[0], x_rightArm_trunk[1], x_rightArm_trunk[2], x_rightArm_trunk[3], x_rightArm_trunk[4], x_rightArm_trunk[5])
    print("Move to the first joints position in positionControl mode.")
    for joint in range(numTrunkJoints):
        print(numTrunkJoints, joint, qTraj[0][joint])
        trunkYarpDevice.IPositionControl.positionMove(joint, qTraj[0][joint])
    for joint in range(0, numRightArmJoints, 1):
        print(numRightArmJoints, joint, qTraj[0][joint+2])
        rightArmYarpDevice.IPositionControl.positionMove(joint, qTraj[0][joint+2])

    markerArray = updateHumanMarkersAdj(0)
    marker = updateMarkerEndEffector(0)
    print("Publish markers")
    pub_marker_array_adj.publish(markerArray)
    pub_marker_array_adj.publish(markerArray)
    pub_marker_array_adj.publish(markerArray)

    pub_end_effector.publish(marker)
    pub_end_effector.publish(marker)

    # i=0
    # print("i: ", i, "Q:", qTraj[i][0], qTraj[i][1], qTraj[i][2],
    #         qTraj[i][3], qTraj[i][4], qTraj[i][5], qTraj[i][6], qTraj[i][7])
    
    # if not trunkYarpDevice.IEncoders.getEncoders(currentTrunkQ):
    #     print("Failed getEncoders")
    # for i in range(numTrunkJoints):
    #     print(currentTrunkQ[i])
    # if not rightArmYarpDevice.IEncoders.getEncoders(currentQ):
    #     print("Failed getEncoders")
    # for i in range(numRightArmJoints):
    #     print(currentQ[i])

    # currentTrunkQ = yarp.DVector(numTrunkJoints)






    # # # # markerArray = updateHumanMarkers(0)
   

    # # pub_marker_array.publish(markerArray)
    # # yarp.delay(15.0)
    print("Change to positionDirect mode.")
    rightArmModes = yarp.IVector(numRightArmJoints, yarp.VOCAB_CM_POSITION_DIRECT)
    if not rightArmYarpDevice.IControlMode.setControlModes(rightArmModes):
        print("Unable to set right arm  to position direct.")
        raise SystemExit
    else:
        print("Right arm set to position direct.")


    trunkModes = yarp.IVector(numTrunkJoints, yarp.VOCAB_CM_POSITION_DIRECT)
    if not trunkYarpDevice.IControlMode.setControlModes(trunkModes):
        print("Unable to set trunk  to position direct.")
        raise SystemExit
    else:
        print("Trunk set to position direct.")

    yarp.delay(10.0)
    print("Moving through the joints position path in positionDirect mode ...")
    period = 200
    start = time.time()
    max = len(qTraj)
    for i in range(0, max, 1):
        try:
            markerArray = updateHumanMarkersAdj(i)
            marker = updateMarkerEndEffector(i)

            print("Publish markers")
            pub_marker_array_adj.publish(markerArray)
            pub_marker_array_adj.publish(markerArray)
            pub_marker_array_adj.publish(markerArray)
            pub_end_effector.publish(marker)

            for joint in range(0, numRightArmJoints, 1):
                rightArmYarpDevice.IPositionDirect.setPosition(joint, qTraj[i][joint+2])
            for joint in range(numTrunkJoints):
                # print(numTrunkJoints, joint, desireQ[joint])
                trunkYarpDevice.IPositionDirect.setPosition(joint, qTraj[i][joint])
            # for j in range(2):
            #     trunkYarpDevice.IPositionDirect.setPosition(j, qTraj[i][j])
            # for j in range(6):
            #     rightArmYarpDevice.IPositionDirect.setPosition(j, qTraj[i][j+2])
            
            time.sleep(period * 0.001 - ((time.time() - start) % (period * 0.001)))
        except KeyboardInterrupt:
            break






    # print("Change to positionDirect mode.")
    # rightArmModes = yarp.IVector(numRightArmJoints, yarp.VOCAB_CM_POSITION)
    # if not rightArmYarpDevice.IControlMode.setControlModes(rightArmModes):
    #     print("Unable to set right arm  to position direct mode.")
    #     raise SystemExit
    # else:
    #     print("Right arm set to position direct mode.")
        
#     pathEff = []
#     with open(optimizedEeffPathFile) as csv_file:
#         csv_reader = csv.reader(csv_file, delimiter=',')
#         line_count = 0
#         for row in csv_reader:
#             qTraj.append([float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5]), float(row[6]), float(row[7]), float(row[8])])
#             Q_eeff = yarp.DVector(numTrunkJoints+numRightArmJoints)
#             for j in range(numRightArmJoints+numTrunkJoints):
#                 Q_eeff[j] = float(row[j+1])
#             x_rightArm_trunk = yarp.DVector(6)
#             trunkRightArmICartesianSolver.fwdKin(Q_eeff, x_rightArm_trunk)
            
#             f = vectorToFrame(x_rightArm_trunk)

#             pose = PoseStamped()
#             pose.pose.position.x = x_rightArm_trunk[0]
#             pose.pose.position.y = x_rightArm_trunk[1]
#             pose.pose.position.z = x_rightArm_trunk[2]
#             pose.pose.orientation.x = f.M.GetQuaternion()[0]
#             pose.pose.orientation.y = f.M.GetQuaternion()[1]
#             pose.pose.orientation.z = f.M.GetQuaternion()[2]
#             pose.pose.orientation.w = f.M.GetQuaternion()[3]

#             eeff_path.poses.append(pose)
#             # print(qTraj[line_count])
#             line_count += 1
#         print(f'Processed {line_count} lines.')

#     eeff_path.header.stamp = rospy.Time.now()
#     pub_eef_path.publish(eeff_path)
        

            
          
#     with open(initPathFile) as csv_file:
#         csv_reader = csv.reader(csv_file, delimiter=',')
#         line_count = 0
#         for row in csv_reader:
#             x.append(float(row[1]))  
#             y.append(float(row[2]))
#             z.append(float(row[3]))
            
#             qw.append(float(row[7]))
#             qx.append(float(row[4]))
#             qy.append(float(row[5]))
#             qz.append(float(row[6]))
            
            
#             # #wrist position+orientation
            
#             x_wrist.append(float(row[8]))
#             y_wrist.append(float(row[9]))
#             z_wrist.append(float(row[10]))
#             qx_wrist.append(float(row[11]))
#             qy_wrist.append(float(row[12]))
#             qz_wrist.append(float(row[13]))
#             qw_wrist.append(float(row[14]))
            
#             # #elbow position+orientation
            
#             x_elbow.append(float(row[15]))
#             y_elbow.append(float(row[16]))
#             z_elbow.append(float(row[17]))
#             qx_elbow.append(float(row[18]))
#             qy_elbow.append(float(row[19]))
#             qz_elbow.append(float(row[20]))
#             qw_elbow.append(float(row[21]))
            
#             x_shoulder.append(float(row[22]))
#             y_shoulder.append(float(row[23]))
#             z_shoulder.append(float(row[24]))
#             qx_shoulder.append(float(row[25]))
#             qy_shoulder.append(float(row[26]))
#             qz_shoulder.append(float(row[27]))
#             qw_shoulder.append(float(row[28]))
            
#             line_count += 1

#         print(f'Processed {line_count} lines.')
        
#     with open(handPathFile) as csv_file:
#         csv_reader = csv.reader(csv_file, delimiter=' ')
#         line_count = 0
#         for row in csv_reader:
#             cx.append(float(row[1]))  
#             cy.append(float(row[2]))
#             cz.append(float(row[3]))
            
#             cqw.append(float(row[7]))
#             cqx.append(float(row[4]))
#             cqy.append(float(row[5]))
#             cqz.append(float(row[6]))
            
#     with open(handPathFile) as csv_file:
#         csv_reader = csv.reader(csv_file, delimiter=' ')
#         current_line = 0
#         for i in range(len(cx)):
#             pose = PoseStamped()
#             pose.pose.position.x = cx[i]
#             pose.pose.position.y = cy[i]
#             pose.pose.position.z = cz[i]-trunkHeight
#             pose.pose.orientation.x = cqx[i]
#             pose.pose.orientation.y = cqy[i]
#             pose.pose.orientation.z = cqz[i]
#             pose.pose.orientation.w = cqw[i]
#             print(current_line)
#             current_line +=1
#             human_hand_path.poses.append(pose)
    
#             # f = PyKDL.Frame(PyKDL.Rotation.Quaternion(pose.pose.orientation.x,pose.pose.orientation.y,pose.pose.orientation.z, pose.pose.orientation.w),
#             #                 PyKDL.Vector(pose.pose.position.x,pose.pose.position.y,pose.pose.position.z))

#             # #f.M = f.M*PyKDL.Rotation.RotX(-np.pi/2.0)
#             # # f.M = f.M*PyKDL.Rotation.RotY(-np.pi/2.0)
#             # # f.M = f.M*PyKDL.Rotation.RotX(-np.pi/4.0)
#             # # f.M = f.M*PyKDL.Rotation.RotY(-np.pi/5)
#             # pose_test = PoseStamped()
#             # pose_test.pose.position.x = f.p.x()
#             # pose_test.pose.position.y = f.p.y()
#             # pose_test.pose.position.z = f.p.z()
            
#             # qx_ = 0
#             # qy_ = 0
#             # qz_ = 0
#             # qw_ = 0
#             # qx_, qy_, qz_, qw_ = f.M.GetQuaternion()
#             # pose_test.pose.orientation.x = qx_
#             # pose_test.pose.orientation.y = qy_
#             # pose_test.pose.orientation.z = qz_
#             # pose_test.pose.orientation.w = qw_
            
#             # test_path.poses.append(pose_test)
#         #     print(frame_goal)
#         # frame_goal.M = frame_goal.M*PyKDL.Rotation.RotZ(-np.pi/2.0)
    
#     human_hand_path.header.stamp = rospy.Time.now()
#     pub_humanHand_path.publish(human_hand_path)
    
#     test_path.header.stamp = rospy.Time.now()
#     pub_test_path.publish(test_path)


#     # Move to the first position
    
#     print("Move to the first joints position in positionControl mode.")
#     for joint in range(0, numRightArmJoints, 1):
#         rightArmYarpDevice.IPositionControl.positionMove(joint, qTraj[0][joint+2])
#     for joint in range(numTrunkJoints):
#         # print(numTrunkJoints, joint, desireQ[joint])
#         trunkYarpDevice.IPositionControl.positionMove(joint, qTraj[0][joint])
    
#     markerArray = updateHumanMarkers(0)
   

#     pub_marker_array.publish(markerArray)
#     yarp.delay(15.0)

#     print("Change to positionDirect mode.")
#     rightArmModes = yarp.IVector(numRightArmJoints, yarp.VOCAB_CM_POSITION_DIRECT)
#     if not rightArmYarpDevice.IControlMode.setControlModes(rightArmModes):
#         print("Unable to set right arm  to position mode.")
#         raise SystemExit
#     else:
#         print("Right arm set to position mode.")


#     trunkModes = yarp.IVector(numTrunkJoints, yarp.VOCAB_CM_POSITION_DIRECT)
#     if not trunkYarpDevice.IControlMode.setControlModes(trunkModes):
#         print("Unable to set trunk  to position mode.")
#         raise SystemExit
#     else:
#         print("Trunk set to position mode.")

#     yarp.delay(1.0)


#     print("Moving through the joints position path in positionDirect mode ...")
#     period = 100
#     start = time.time()
#     max = len(qTraj)
#     for i in range(1, max, 1):
        
#         markerArray = updateHumanMarkers(i)
#         pub_marker_array.publish(markerArray)
        
#         for j in range(2):
#             trunkYarpDevice.IPositionDirect.setPosition(j, qTraj[i][j])
#         for j in range(6):
#             rightArmYarpDevice.IPositionDirect.setPosition(j, qTraj[i][j+2])
        
#         time.sleep(period * 0.001 - ((time.time() - start) % (period * 0.001)))

#         # print("i: ", i, "Q:", qTraj[i][0], qTraj[i][1], qTraj[i][2],
#         #     qTraj[i][3], qTraj[i][4], qTraj[i][5], qTraj[i][6], qTraj[i][7])

#     print("Change to positionDirect mode.")
#     rightArmModes = yarp.IVector(numRightArmJoints, yarp.VOCAB_CM_POSITION)
#     if not rightArmYarpDevice.IControlMode.setControlModes(rightArmModes):
#         print("Unable to set right arm  to position direct mode.")
#         raise SystemExit
#     else:
#         print("Right arm set to position direct mode.")


#     trunkModes = yarp.IVector(numTrunkJoints, yarp.VOCAB_CM_POSITION)
#     if not trunkYarpDevice.IControlMode.setControlModes(trunkModes):
#         print("Unable to set trunk  to position direct mode.")
#         raise SystemExit
#     else:
#         print("Trunk set to position direct mode.")

#     yarp.delay(1.0)


#     print("Move to the first joints position in positionControl mode.")
#     for joint in range(0, numRightArmJoints, 1):
#         rightArmYarpDevice.IPositionControl.positionMove(joint, qTraj[max-1][joint+2])
#     for joint in range(numTrunkJoints):
#         # print(numTrunkJoints, joint, desireQ[joint])
#         trunkYarpDevice.IPositionControl.positionMove(joint, qTraj[max-1][joint])

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass


