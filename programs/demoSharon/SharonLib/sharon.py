from sys import prefix
import yarp
import numpy as np
from scipy import interpolate
import time
import roboticslab_kinematics_dynamics
from SharonLib.config_asr import word_dict
import PyKDL
import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator
import datetime
import os
import csv
from PIL import Image

parent_directory = '/home/elisabeth/repos/teo-sharon/data/'

VOCAB_OK = yarp.createVocab32('o', 'k')
VOCAB_FAIL = yarp.createVocab32('f', 'a', 'i', 'l')

class Sharon():
    def __init__(self, robot, prefix, only_asr, use_right_arm, 
                 available_right_arm_controlboard, available_left_arm_controlboard,
                 available_trunk_controlboard, available_right_hand_controlboard,
                 available_left_hand_controlboard, reachingDistance, upDistance):
        
        self.robot = robot
        self.prefix = prefix
        self.only_asr = only_asr
        self.use_right_arm = use_right_arm
        self.reachingDistance = reachingDistance
        self.available_right_hand_controlboard = available_right_hand_controlboard
        self.available_left_hand_controlboard = available_left_hand_controlboard
        self.upDistance = upDistance 
        self.available_right_arm_controlboard = available_right_arm_controlboard
        self.available_left_arm_controlboard = available_left_arm_controlboard
        self.available_trunk_controlboard = available_trunk_controlboard
        now = datetime.datetime.now()
        directory = str(now.year)+"-"+str(now.month)+"-"+str(now.day)+"_"+str(now.hour)+":"+str(now.minute)+":"+str(now.second)
        
        self.path = os.path.join(parent_directory, directory)
        os.mkdir(self.path)
        os.mkdir(self.path+'/rgb')
        os.mkdir(self.path+'/depth')
        self.init = None
        
        
        self.task_execution_csv = open(self.path+"/"+"tasks_execution.csv", "w", newline="\n") 
        # now you have an empty file already
        writer = csv.writer(self.task_execution_csv)
        # write a row to the csv file
        writer.writerow(['Demo executed with '+ self.robot])
        if self.use_right_arm:
            writer.writerow(['Using trunk and right arm.'])
        else:
            writer.writerow(['Using trunk and left arm.'])
            
        if self.only_asr:
            writer.writerow(['Using only asr.'])
        else:
            writer.writerow(['Using asr and gaze.'])

        
        
        # Trunk device
        self.trunkOptions = yarp.Property()
        self.trunkDevice = None
        self.trunkIEncoders = None
        self.numTrunkJoints = 2
        self.trunkIPositionControl = None
        self.trunkIControlMode = None
        self.trunkIPositionDirect = None
        self.trunkIRemoteVariables = False
        
        
        # Right Arm device
        self.rightArmOptions = yarp.Property()
        self.rightArmDevice = None
        self.rightArmIEncoders = None
        self.numRightArmJoints = 0
        self.rightArmIPositionControl = None
        self.rightArmIControlMode = None
        self.rightArmIPositionDirect = None
        self.rightArmIRemoteVariables = False
        self.rightArmIControlLimits = None
        self.rightArmMin = None
        self.rightArmMax = None

        # Right hand device
        self.rightHandOptions = yarp.Property()
        self.rightHandDevice = None
        self.rightHandIEncoders = None
        self.numRightHandJoints = 0
        self.rightHandIPositionControl = None
        self.rightHandIPWMControl = None
        
        # Left Arm device
        self.leftArmOptions = yarp.Property()
        self.leftArmDevice = None
        self.leftArmIEncoders = None
        self.numLeftArmJoints = 0
        self.leftArmIPositionControl = None
        self.leftArmIPositionControl = None
        self.leftArmIControlMode = None
        self.leftArmIPositionDirect = None
        self.leftArmIRemoteVariables = False
        
        # Left hand device
        self.leftHandOptions = yarp.Property()
        self.leftHandDevice = None
        self.leftHandIEncoders = None
        self.numLeftHandJoints = 0
        self.leftHandIPositionControl = None
        self.leftHandIPWMControl = None
        
        # TrunkAndRightArm solver device
        self.trunkRightArmSolverOptions = yarp.Property()
        self.trunkRightArmSolverDevice = None
        self.trunkRightArmICartesianSolver = None
        
        # TrunkAndLeftArm solver device
        self.trunkLeftArmSolverOptions = yarp.Property()
        self.trunkLeftArmSolverDevice = None
        self.trunkLeftArmICartesianSolver = None

        # Trajectory Generation client
        self.rpcClientTrajectoryGeneration = yarp.RpcClient()

        # Get Grasping poses client
        self.rpcClientGetGraspingPoses = yarp.RpcClient()
        self.superquadrics = yarp.Bottle()
        self.labelSuperquadricsCategory = yarp.Bottle()
        self.superquadricsBoundingBoxes = yarp.Bottle()
        self.currentObject = yarp.Bottle()
        self.bGraspingPoses = yarp.Bottle()
        # Asr 
        self.asrPort = yarp.BufferedPortBottle()
        self.asrData = None
        
        # RgbdObjectDetection
        self.xtionObjectDetectionsPort = yarp.BufferedPortBottle()
        self.realsenseImagePort = yarp.Port()
        self.realsenseDepthImagePort = yarp.Port()


        self.graspingPose=yarp.Bottle()
        self.graspingQ = yarp.Bottle()
        self.reachingPose = yarp.Bottle()
        self.reachingQ = yarp.Bottle()
        
        
        self.img_array = np.zeros((480, 640, 3), dtype=np.uint8)
        self.yarp_image = yarp.ImageRgb()
        self.yarp_image.resize(640, 480)
        self.yarp_image.setExternal(self.img_array.data, self.img_array.shape[1], self.img_array.shape[0])
        
        self.depthImgArray = np.random.uniform(0., 65535., (480, 640)).astype(np.float32)
        self.yarpImgDepth = yarp.ImageFloat()
        self.yarpImgDepth.resize(640,480)
        self.yarpImgDepth.setExternal(self.depthImgArray.data,self.depthImgArray.shape[1], self.depthImgArray.shape[0])
        
        
    def configure(self):
        print("Configuring...")


        #Open asr port
        self.asrPort.open(self.prefix+"/asr:i")
        yarp.Network.connect("/asr:o", self.prefix+"/asr:i")
        
        
        # Open rightArm device
        if self.available_right_arm_controlboard:
            self.rightArmOptions.put('device', 'remote_controlboard')
            self.rightArmOptions.put('remote', self.robot+'/trunkAndRightArm')
            self.rightArmOptions.put('local', self.prefix+self.robot+'/trunkAndRightArm')

            self.rightArmDevice = yarp.PolyDriver(self.rightArmOptions)
            self.rightArmDevice.open(self.rightArmOptions)
            
            if not self.rightArmDevice.isValid():
                print('Cannot open rightArm device!')
                raise SystemExit
            print("rightArm device opened!")
            
            self.rightArmIEncoders = self.rightArmDevice.viewIEncoders()

            if self.rightArmIEncoders == []:
                print("Right arm Encoder interface NOT available.")
                raise SystemExit
            else:
                print("Right arm Encoder interface available.")

            self.numRightArmJoints = self.rightArmIEncoders.getAxes()
            
            print("rightArm has", self.numRightArmJoints, "joints.")
            
            
            #Right arm control mode interface
            self.rightArmIControlMode = self.rightArmDevice.viewIControlMode()
            if self.rightArmIControlMode == []:
                print("Right arm control mode interface NOT available.")
                raise SystemExit
            else:
                print("Right arm control mode interface available.")
            
            # Right arm position control interface
            self.rightArmIPositionControl = self.rightArmDevice.viewIPositionControl()

            if self.rightArmIPositionControl == []:
                print("Right arm position control interface NOT available")
                raise SystemExit
            else:
                print("Right arm position control interface available.")
            
            # Right arm position direct interface
            self.rightArmIPositionDirect = self.rightArmDevice.viewIPositionDirect()
            if self.rightArmIPositionDirect == []:
                print("Right arm position direct interface NOT available.")
                raise SystemExit
            else:
                print("Right arm position direct interface available")
            
            # Right arm remote variables interface
            self.rightArmIRemoteVariables = self.rightArmDevice.viewIRemoteVariables()
            if self.rightArmIRemoteVariables == []:
                print("Right arm remote variables interface NOT available.")
                raise SystemExit
            else:
                print("Right arm remote variables interface available.")
            
            self.rightArmIControlLimits = self.rightArmDevice.viewIControlLimits()
            if self.rightArmIControlLimits == []:
                print("Right arm control limits interface NOT available.")
                raise SystemExit
            else:
                print("Right arm control limits interface available.")
            
            self.rightArmMin = yarp.Vector(self.numRightArmJoints)
            self.rightArmMax = yarp.Vector(self.numRightArmJoints)
            
            for joint in range(0, self.numRightArmJoints):
                aux1 = yarp.Vector(1)
                aux2 = yarp.Vector(1)

                self.rightArmIControlLimits.getLimits(joint,aux1.data(), aux2.data())
                
                self.rightArmMin[joint] = aux1.get(0)
                self.rightArmMax[joint] = aux2.get(0)
            
            self.rightArmMin[0] = -31.0
            self.rightArmMax[0] = 31.0
            self.rightArmMax[1] = 16.5

            
            rf = yarp.ResourceFinder()
            rf.setDefaultContext("kinematics")
            trunkRightKinPath = rf.findFileByName("teo-trunk-rightArm-fetch.ini")
            
            print(trunkRightKinPath)
            self.trunkRightArmSolverOptions.fromConfigFile(trunkRightKinPath)
            self.trunkRightArmSolverOptions.put("device", "KdlSolver")
            self.trunkRightArmSolverOptions.put("ik", "nrjl")
            self.trunkRightArmSolverOptions.put("eps",0.005)
            self.trunkRightArmSolverOptions.put("maxIter",100000)
            mins = "(mins ("+self.rightArmMin.toString()+"))"
            maxs = "(maxs ("+self.rightArmMax.toString()+"))"
            self.trunkRightArmSolverOptions.put("mins", self.rightArmMin.toString())
            self.trunkRightArmSolverOptions.put("maxs", self.rightArmMax.toString())
            self.trunkRightArmSolverOptions.fromString(mins, False)
            self.trunkRightArmSolverOptions.fromString(maxs, False)
            self.trunkRightArmSolverDevice = yarp.PolyDriver(self.trunkRightArmSolverOptions)  # calls open -> connects
            self.trunkRightArmSolverDevice.open(self.trunkRightArmSolverOptions)

            if self.trunkRightArmSolverDevice == []:
                print("trunk right arm solver device interface NOT available")
                raise SystemExit
            else:
                print("trunk right arm solver device interface available.")
                
            # Trunk and right arm cartesian solver
            self.trunkRightArmICartesianSolver = roboticslab_kinematics_dynamics.viewICartesianSolver(self.trunkRightArmSolverDevice)
            if self.trunkRightArmICartesianSolver == []:
                print("Right arm cartesian solver interface NOT available")
                raise SystemExit
            else:
                print("Right arm cartesian solver interface available.")
            print(self.trunkRightArmSolverOptions.find("mins").toString())
            print(self.trunkRightArmSolverOptions.find("maxs").toString())

        # Open leftArm device
        if self.available_left_arm_controlboard:
            self.leftArmOptions.put('device', 'remote_controlboard')
            self.leftArmOptions.put('remote', self.robot+'/trunkAndLeftArm')
            self.leftArmOptions.put('local', self.prefix+self.robot+'/trunkAndLeftArm')

            self.leftArmDevice = yarp.PolyDriver(self.leftArmOptions)
            self.leftArmDevice.open(self.leftArmOptions)
            
            if not self.leftArmDevice.isValid():
                print('Cannot open leftArm device!')
                raise SystemExit
            print("leftArm device opened!")
            
            self.leftArmIEncoders = self.leftArmDevice.viewIEncoders()

            if self.leftArmIEncoders == []:
                print("Left arm Encoder interface NOT available.")
                raise SystemExit
            else:
                print("Left arm Encoder interface available.")

            self.numLeftArmJoints = self.leftArmIEncoders.getAxes()
            
            print("leftArm has", self.numLeftArmJoints, "joints.")
            
            
            # Left arm control mode interface
            self.leftArmIControlMode = self.leftArmDevice.viewIControlMode()
            if self.leftArmIControlMode == []:
                print("Left arm control mode interface NOT available.")
                raise SystemExit
            else:
                print("Left arm control mode interface available.")
            
            # Left arm position control interface
            self.leftArmIPositionControl = self.leftArmDevice.viewIPositionControl()

            if self.leftArmIPositionControl == []:
                print("Left arm position control interface NOT available")
                raise SystemExit
            else:
                print("Left arm position control interface available.")
            
            # Left arm position direct interface
            self.leftArmIPositionDirect = self.leftArmDevice.viewIPositionDirect()
            if self.leftArmIPositionDirect == []:
                print("Left arm position direct interface NOT available.")
                raise SystemExit
            else:
                print("Left arm position direct interface available")
            
            # Left arm remote variables interface
            self.leftArmIRemoteVariables = self.leftArmDevice.viewIRemoteVariables()
            if self.leftArmIRemoteVariables == []:
                print("Left arm remote variables interface NOT available.")
                raise SystemExit
            else:
                print("Left arm remote variables interface available.")
            
            self.leftArmIControlLimits = self.leftArmDevice.viewIControlLimits()
            if self.leftArmIControlLimits == []:
                print("Left arm control limits interface NOT available.")
                raise SystemExit
            else:
                print("Left arm control limits interface available.")
            
            self.leftArmMin = yarp.Vector(self.numLeftArmJoints)
            self.leftArmMax = yarp.Vector(self.numLeftArmJoints)
            
            for joint in range(0, self.numLeftArmJoints):
                aux1 = yarp.Vector(1)
                aux2 = yarp.Vector(1)

                self.leftArmIControlLimits.getLimits(joint,aux1.data(), aux2.data())
                
                self.leftArmMin[joint] = aux1.get(0)
                self.leftArmMax[joint] = aux2.get(0)
            
            self.leftArmMin[0] = -31.0
            self.leftArmMax[0] = 31.0
            self.leftArmMax[1] = 16.5

            
            rf = yarp.ResourceFinder()
            rf.setDefaultContext("kinematics")
            trunkLeftKinPath = rf.findFileByName("teo-trunk-leftArm-fetch.ini")
            
            print(trunkLeftKinPath)
            self.trunkLeftArmSolverOptions.fromConfigFile(trunkLeftKinPath)
            self.trunkLeftArmSolverOptions.put("device", "KdlSolver")
            self.trunkLeftArmSolverOptions.put("ik", "nrjl")
            self.trunkLeftArmSolverOptions.put("eps",0.005)
            self.trunkLeftArmSolverOptions.put("maxIter",100000)
            mins = "(mins ("+self.leftArmMin.toString()+"))"
            maxs = "(maxs ("+self.leftArmMax.toString()+"))"
            self.trunkLeftArmSolverOptions.put("mins", self.leftArmMin.toString())
            self.trunkLeftArmSolverOptions.put("maxs", self.leftArmMax.toString())
            self.trunkLeftArmSolverOptions.fromString(mins, False)
            self.trunkLeftArmSolverOptions.fromString(maxs, False)
            self.trunkLeftArmSolverDevice = yarp.PolyDriver(
            self.trunkLeftArmSolverOptions)  # calls open -> connects
            self.trunkLeftArmSolverDevice.open(self.trunkLeftArmSolverOptions)

            if self.trunkLeftArmSolverDevice == []:
                print("trunk left arm solver device interface NOT available")
                raise SystemExit
            else:
                print("trunk left arm solver device interface available.")
                
            # Trunk and left arm cartesian solver
            self.trunkLeftArmICartesianSolver = roboticslab_kinematics_dynamics.viewICartesianSolver(self.trunkLeftArmSolverDevice)
            if self.trunkLeftArmICartesianSolver == []:
                print("Left arm cartesian solver interface NOT available")
                raise SystemExit
            else:
                print("Left arm cartesian solver interface available.")
            print(self.trunkLeftArmSolverOptions.find("mins").toString())
            print(self.trunkLeftArmSolverOptions.find("maxs").toString())

        
        
        
        self.rpcClientGetGraspingPoses.open(self.prefix+"/getGraspingPoses/rpc:c")
        yarp.Network.connect(self.prefix+"/getGraspingPoses/rpc:c",
                             "/getGraspingPoses/rpc:s")

        if self.available_right_arm_controlboard:
            self.rpcClientTrajectoryGeneration.open('/trajectoryGeneration/trunkAndRightArm/rpc:c')
            if self.robot == "/teo":
                yarp.Network.connect("/trajectoryGeneration/trunkAndRightArm/rpc:c",
                                    self.robot+"/trajectoryGeneration/trunkAndRightArm/rpc:s")
            else:
                yarp.Network.connect("/trajectoryGeneration/trunkAndRightArm/rpc:c",
                                    "/trajectoryGeneration/trunkAndRightArm/rpc:s")
            print("connected /trajectoryGeneration/trunkAndRightArm/rpc:s")
        
        if self.available_left_arm_controlboard:
            self.rpcClientTrajectoryGeneration.open('/trajectoryGeneration/trunkAndLeftArm/rpc:c')
            if self.robot == "/teo":
                yarp.Network.connect("/trajectoryGeneration/trunkAndLeftArm/rpc:c",
                                    self.robot+"/trajectoryGeneration/trunkAndLeftArm/rpc:s")
            else:
                yarp.Network.connect("/trajectoryGeneration/trunkAndLeftArm/rpc:c",
                                    "/trajectoryGeneration/trunkAndLeftArm/rpc:s") 
            
        
        self.xtionObjectDetectionsPort.open(self.prefix+"/rgbdObjectDetection/state:i")
        yarp.Network.connect("/rgbdObjectDetection/state:o", self.prefix+"/rgbdObjectDetection/state:i")
        
        self.realsenseImagePort.open(self.prefix+"/realsense2/rgbImage:i")
        yarp.Network.connect("/realsense2/rgbImage:o", self.prefix+"/realsense2/rgbImage:i")
        
        self.realsenseDepthImagePort.open(self.prefix+"/realsense2/depthImage:i")
        yarp.Network.connect("/realsense2/depthImage:o", self.prefix+"/realsense2/depthImage:i")
        

        
        if self.available_right_hand_controlboard:
            # Open rightHand device
            self.rightHandOptions = yarp.Property()

            self.rightHandOptions.put('device', 'remote_controlboard')
            self.rightHandOptions.put('remote', self.robot+'/rightHand')
            self.rightHandOptions.put('local', self.prefix+self.robot+'/rightHand')

            self.rightHandDevice = yarp.PolyDriver(self.rightHandOptions)
            self.rightHandDevice.open(self.rightHandOptions)

            if not self.rightHandDevice.isValid():
                print('Cannot open rightHand device!')
                raise SystemExit

            if self.robot == "/teoSim":
                self.rightHandIPositionControl = self.rightHandDevice.viewIPositionControl()

                if self.rightHandIPositionControl == []:
                    print("Right hand position control interface NOT available")
                    raise SystemExit
                else:
                    print("Right hand position control interface available.")
            elif self.robot == "/teo":
                self.rightHandIPWMControl = self.rightHandDevice.viewIPWMControl()
                if self.rightHandIPWMControl == []:
                    print("Right hand ipwm control interface NOT available")
                    raise SystemExit
                else:
                    print("Right hand ipwm control interface available.")
                    
        if self.available_left_hand_controlboard:
            # Open leftHand device
            self.leftHandOptions = yarp.Property()

            self.leftHandOptions.put('device', 'remote_controlboard')
            self.leftHandOptions.put('remote', self.robot+'/leftHand')
            self.leftHandOptions.put('local', self.prefix+self.robot+'/leftHand')

            self.leftHandDevice = yarp.PolyDriver(self.leftHandOptions)
            self.leftHandDevice.open(self.leftHandOptions)

            if not self.leftHandDevice.isValid():
                print('Cannot open leftHand device!')
                raise SystemExit

            if self.robot == "/teoSim":
                self.leftHandIPositionControl = self.leftHandDevice.viewIPositionControl()

                if self.leftHandIPositionControl == []:
                    print("Left hand position control interface NOT available")
                    raise SystemExit
                else:
                    print("Left hand position control interface available.")
        
        print("Configuring done!")
    
    
    def vectorToFrame(self, x):

        f = PyKDL.Frame()

        f.p.x(x[0])
        f.p.y(x[1])
        f.p.z(x[2])

        rotvec = PyKDL.Vector(x[3], x[4], x[5])
        f.M = PyKDL.Rotation.Rot(rotvec, rotvec.Norm())
        return f

    def frameToVector(self, frame):
        x = []

        x.append(frame.p.x())
        x.append(frame.p.y())
        x.append(frame.p.z())

        rotVector = frame.M.GetRot()
        x.append(rotVector.x())
        x.append(rotVector.y())
        x.append(rotVector.z())
        return x
    
    def putTrunkToInitPositionWithControlPosition(self, initTrunkPosition):
        # Put to control position 
        if self.use_right_arm:
            rightArmModes = yarp.IVector(self.numRightArmJoints,yarp.VOCAB_CM_POSITION)
            if not self.rightArmIControlMode.setControlModes(rightArmModes):
                print("Unable to set right arm  to position  mode.")
                raise SystemExit
            else:
                print("Right arm set to position  mode.")
            
            for j in range(len(initTrunkPosition)):
                self.rightArmIPositionControl.positionMove(j, initTrunkPosition[j])
        else:
            leftArmModes = yarp.IVector(self.numLeftArmJoints,yarp.VOCAB_CM_POSITION)
            if not self.leftArmIControlMode.setControlModes(leftArmModes):
                print("Unable to set left arm  to position  mode.")
                raise SystemExit
            else:
                print("Left arm set to position  mode.")
            
            for j in range(len(initTrunkPosition)):
                self.leftArmIPositionControl.positionMove(j, initTrunkPosition[j])
        
        yarp.delay(5.0)
    
    def computeIntersectionOverUnion(self, superquadricBbox, detectionBbox):
        print("Compute")
        if (detectionBbox[0] < superquadricBbox[2] and detectionBbox[2] > superquadricBbox[0] and
            detectionBbox[1] < superquadricBbox[3] and detectionBbox[3] > superquadricBbox[1]):
            xA = detectionBbox[0]
            if superquadricBbox[0] > xA:
                xA = superquadricBbox[0]
                
            yA = detectionBbox[1]
            if superquadricBbox[1] > yA:
                yA = superquadricBbox[1]

            xB = detectionBbox[2]
            if superquadricBbox[2] < xB:
                xB = superquadricBbox[2]
                
            yB = detectionBbox[3]
            if superquadricBbox[3] < yB:
                yB = superquadricBbox[3]
                
            inter_area = (xB - xA) * (yB - yA)
            print("Inter area: ",inter_area)

            detection_bbox_area = (detectionBbox[2] - detectionBbox[0]) * (detectionBbox[3] - detectionBbox[1])
            print("detection bbox area: ",detection_bbox_area)
            cluster_bbox_area = (superquadricBbox[2] - superquadricBbox[0]) * (superquadricBbox[3] - superquadricBbox[1])
            print("cluster bbox area: ",cluster_bbox_area)

            IoU = inter_area / (detection_bbox_area + cluster_bbox_area - inter_area)
            print("IoU: ", IoU)

            return True, IoU
        else:
            print("No IoU")
            IoU = 0
        return False, IoU
    
    def setLabelSuperquadricsCategory(self):
        self.labelSuperquadricsCategory = yarp.Bottle()
        print(self.superquadricsBoundingBoxes.size())
        print(self.detections.size())
        
        
        for j in range(self.detections.size()):
            detection = self.detections.get(j)
            tlxDetection = detection.find("tlx").asFloat64()
            tlyDetection = detection.find("tly").asFloat64()
            brxDetection = detection.find("brx").asFloat64()
            bryDetection = detection.find("bry").asFloat64()
            bboxDetection = [tlxDetection, tlyDetection, brxDetection, bryDetection]
            print("j: ",j, bboxDetection)
        
            IoU = 0
            current_label = None
            for i in range(self.superquadricsBoundingBoxes.size()):
                superquadricBB = self.superquadricsBoundingBoxes.get(i)
                label_idx = superquadricBB.find("label_idx").asInt32()
                print("label_idx: ",label_idx)
                tlx = superquadricBB.find("tlx").asFloat64()
                tly = superquadricBB.find("tly").asFloat64()
                brx = superquadricBB.find("brx").asFloat64()
                bry = superquadricBB.find("bry").asFloat64()
                bbox = [tlx, tly, brx, bry]
                print("i: ",i, bbox)
                auxSuperquadric = yarp.Bottle()
                print(i,j)
                found,currentIoU = self.computeIntersectionOverUnion(bbox, bboxDetection)
                if currentIoU > IoU:
                    IoU = currentIoU
                    current_label = superquadricBB.find("label_idx").asInt32()
                 
            if IoU != 0:
                dictObject =  auxSuperquadric.addDict()
                dictObject.put('category',detection.find("category").asString() ) 
                dictObject.put('label_idx',current_label)                    
                print(auxSuperquadric.toString())
                self.labelSuperquadricsCategory.append(auxSuperquadric)
        print(self.labelSuperquadricsCategory.toString())
   
    def computeTrajectoryToJointsPosition(self, rightArm, jointsPosition):

        cmd = yarp.Bottle()
        cmd.addString('cpgj')
        goal = cmd.addList()

        for jointPosition in jointsPosition:
            goal.addFloat64(jointPosition)
        print(cmd.toString())
        response = yarp.Bottle()
        jointsTrajectory = []
        
        self.rpcClientTrajectoryGeneration.write(cmd, response)

        if response.get(0).asVocab32() == VOCAB_FAIL:
            print(response.get(1).toString())
            return False, jointsTrajectory
        elif response.get(0).asVocab32() == VOCAB_OK:
            bJointsTrajectory = response.get(1).asList()
            print('Solution Found with ', bJointsTrajectory.size(), "points.")
            for i in range(bJointsTrajectory.size()):
                bJointsPosition = bJointsTrajectory.get(i).asList()
                jointsPosition = []
                for i in range(bJointsPosition.size()):
                    jointsPosition.append(bJointsPosition.get(i).asFloat64())
                jointsTrajectory.append(jointsPosition)
            print("JointsTrajectory")
            print(jointsTrajectory)
            return True, jointsTrajectory
   
    def computeTrajectoryToPose(self, rightArm, reachingPose):

        cmd = yarp.Bottle()
        cmd.addVocab32('cpgp')
        goal = cmd.addList()
        goal.addDouble(reachingPose[0])
        goal.addDouble(reachingPose[1])
        goal.addDouble(reachingPose[2])
        goal.addDouble(reachingPose[3])
        goal.addDouble(reachingPose[4])
        goal.addDouble(reachingPose[5])
        print(cmd.toString())
        response = yarp.Bottle()
        jointsTrajectory = []
        if self.use_right_arm:
            self.rpcClientTrajectoryGeneration.write(cmd, response)
        else:
            raise SystemExit
        
        if response.get(0).asVocab32() == VOCAB_FAIL:
            print(response.get(1).asString())
            return False, jointsTrajectory
        elif response.get(0).asVocab32() == VOCAB_OK:
            bJointsTrajectory = response.get(1).asList()
            print('Solution Found with ', bJointsTrajectory.size(), "points.")
            for i in range(bJointsTrajectory.size()):
                bJointsPosition = bJointsTrajectory.get(i).asList()
                jointsPosition = []
                for i in range(bJointsPosition.size()):
                    jointsPosition.append(bJointsPosition.get(i).asFloat64())
                jointsTrajectory.append(jointsPosition)
            print("JointsTrajectory")
            return True, jointsTrajectory
    
    def plotTrajectories(self, jointsTrajectory, smoothJointsTrajectory):
        arr = np.asarray(jointsTrajectory)
        t = np.linspace(0,1.0, len(jointsTrajectory))

        arr_smooth = np.asarray(smoothJointsTrajectory)
        t_smooth = np.linspace(0,1.0, len(smoothJointsTrajectory))


        fig, ax = plt.subplots(8, sharex=True)
        plt.subplots_adjust(left=0.15, right=0.95, top=0.95, bottom=0.05)
        # set axes labels
        # ax[0].set_xlabel("num point")
        ax[0].set_ylabel("Axial Trunk [deg]", rotation=0, ha="right")
        ax[1].set_ylabel("Frontal Trunk [deg]",  rotation=0, ha="right")
        
        if self.use_right_arm:
            ax[2].set_ylabel("Frontal Right Shoulder [deg]",
                             rotation=0, ha="right")
            ax[3].set_ylabel("Sagittal Right Shoulder [deg]",
                             rotation=0, ha="right")
            ax[4].set_ylabel("Axial Right Shoulder [deg]",
                             rotation=0, ha="right")
            ax[5].set_ylabel("Frontal Right Elbow [deg]",
                             rotation=0, ha="right")
            ax[6].set_ylabel("Axial Right Wrist [deg]", rotation=0, ha="right")
            ax[7].set_ylabel("Frontal Right Wrist [deg]",
                             rotation=0, ha="right")
        else:
            ax[2].set_ylabel("Frontal Left Shoulder [deg]",
                             rotation=0, ha="right")
            ax[3].set_ylabel("Sagittal Left Shoulder [deg]",
                             rotation=0, ha="right")
            ax[4].set_ylabel("Axial Left Shoulder [deg]",
                             rotation=0, ha="right")
            ax[5].set_ylabel("Frontal Left Elbow [deg]",
                             rotation='horizontal', labelpad=15)
            ax[6].set_ylabel("Axial Left Wrist [deg]",
                             rotation='horizontal', labelpad=15)
            ax[7].set_ylabel("Frontal Left Wrist [deg]",
                             rotation='horizontal', labelpad=15)

        fig.suptitle("Joints trajectories.")
        for i in range(8):
            ax[i].grid()
            ax[i].set_xlim(0, 1)
            ax[i].yaxis.set_major_locator(MaxNLocator(5))
        ax[0].plot(t, arr[:, 0], label='q0', color='skyblue')
        ax[0].plot(t_smooth, arr_smooth[:, 0], label='q0 smooth',
                   color='red', linestyle='dashed')

        ax[1].plot(t, arr[:, 1], label='q1')
        ax[1].plot(t_smooth, arr_smooth[:, 1], label='q1 smooth',
                   color='red', linestyle='dashed')

        ax[2].plot(t, arr[:, 2], label='q2')
        ax[2].plot(t_smooth, arr_smooth[:, 2], label='q2 smooth',
                   color='red', linestyle='dashed')

        ax[3].plot(t, arr[:, 3], label='q3')
        ax[3].plot(t_smooth, arr_smooth[:, 3], label='q3 smooth',
                   color='red', linestyle='dashed')

        ax[4].plot(t, arr[:, 4], label='q4')
        ax[4].plot(t_smooth, arr_smooth[:, 4], label='q4 smooth',
                   color='red', linestyle='dashed')

        ax[5].plot(t, arr[:, 5], label='q5')
        ax[5].plot(t_smooth, arr_smooth[:, 5], label='q5 smooth',
                   color='red', linestyle='dashed')

        ax[6].plot(t, arr[:, 6], label='q6')
        ax[6].plot(t_smooth, arr_smooth[:, 6], label='q6 smooth',
                   color='red', linestyle='dashed')

        ax[7].plot(t, arr[:, 7], label='Joint trajectory')
        ax[7].plot(t_smooth, arr_smooth[:, 7],
                   label='Smoothed joint trajectory', color='red', linestyle='dashed')
        
        ax[7].xaxis.set_ticks(np.linspace(0, 1.0, 100))

        handles, labels = ax[7].get_legend_handles_labels()
        fig.legend(handles, labels, loc='upper right')

        plt.show()
        plt.figure().clear()
        plt.close()
    
    def computeSmoothJointsTrajectory(self, jointsTrajectory, nPoints):
        smoothJointsTrajectory = []
        arr = np.asarray(jointsTrajectory)
        t = range(0, len(jointsTrajectory))
        s = 0.5
        print("interpolate")
        tck, u = interpolate.splprep([t, arr[:, 0]], s=s)

        # create interpolated lists of points
        tnew, q0new = interpolate.splev(np.linspace(0, 1.0, nPoints), tck)

        print(arr[:, 1])
        tck, u = interpolate.splprep([t, arr[:, 1]], s=s)
        # create interpolated lists of points
        tnew, q1new = interpolate.splev(np.linspace(0, 1.0, nPoints), tck)

        print("first done")
        tck, u = interpolate.splprep([t, arr[:, 2]], s=s)
        # create interpolated lists of points
        tnew, q2new = interpolate.splev(np.linspace(0, 1.0, nPoints), tck)

        tck, u = interpolate.splprep([t, arr[:, 3]], s=s)
        # create interpolated lists of points
        tnew, q3new = interpolate.splev(np.linspace(0, 1.0, nPoints), tck)

        tck, u = interpolate.splprep([t, arr[:, 4]], s=s)
        # create interpolated lists of points
        tnew, q4new = interpolate.splev(np.linspace(0, 1.0, nPoints), tck)

        tck, u = interpolate.splprep([t, arr[:, 5]], s=s)
        # create interpolated lists of points
        tnew, q5new = interpolate.splev(np.linspace(0, 1.0, nPoints), tck)

        tck, u = interpolate.splprep([t, arr[:, 6]], s=s)
        # create interpolated lists of points
        tnew, q6new = interpolate.splev(np.linspace(0, 1.0, nPoints), tck)

        tck, u = interpolate.splprep([t, arr[:, 7]], s=s)
        # create interpolated lists of points
        tnew, q7new = interpolate.splev(np.linspace(0, 1.0, nPoints), tck)
        
        print("done")
        
        print(len(q0new), len(q1new), len(q2new), len(q3new), len(q4new), len(q5new), len(q6new), len(q7new))

        for i in range(0, len(q7new)):
            jointsPosition = [q0new[i], q1new[i], q2new[i], q3new[i], q4new[i], q5new[i], q6new[i], q7new[i]]
            smoothJointsTrajectory.append(jointsPosition)
        return smoothJointsTrajectory    
    
    def followJointsTrajectory(self, jointsTrajectory):

        self.numPointTrajectory = 0
        print("followJointsTrajectory ", self.numPointTrajectory, len(jointsTrajectory))
        period = 40
        
        if self.use_right_arm:
     
            rightArmModes = yarp.IVector(self.numRightArmJoints,yarp.VOCAB_CM_POSITION_DIRECT)
            if not self.rightArmIControlMode.setControlModes(rightArmModes):
                print("Unable to set right arm  to position direct mode.")
                raise SystemExit
            else:
                print("Right arm set to position direct mode.") 
                            
            start = time.time()

            while self.numPointTrajectory < len(jointsTrajectory)-1:
                for j in range(self.numRightArmJoints):
                    self.rightArmIPositionDirect.setPosition(j, jointsTrajectory[self.numPointTrajectory][j])
                time.sleep(period * 0.001 - ((time.time() - start) % (period * 0.001)))
                self.numPointTrajectory+=1
            return True
        else:
            leftArmModes = yarp.IVector(self.numLeftArmJoints,yarp.VOCAB_CM_POSITION_DIRECT)
            if not self.leftArmIControlMode.setControlModes(leftArmModes):
                print("Unable to set left arm  to position direct mode.")
                raise SystemExit
            else:
                print("Left arm set to position direct mode.") 
                            
            start = time.time()

            while self.numPointTrajectory < len(jointsTrajectory)-1:
                for j in range(self.numLeftArmJoints):
                    self.leftArmIPositionDirect.setPosition(j, jointsTrajectory[self.numPointTrajectory][j])
                time.sleep(period * 0.001 - ((time.time() - start) % (period * 0.001)))
                self.numPointTrajectory+=1
            return True
        
    
    def getFeasibleGraspingPose(self, bGraspingPoses, reachingDistance):
        graspingPose = []
        reaching_pose = []
        grasping_q = []
        reaching_q = []
        print(bGraspingPoses.size())

        for j in range(1, bGraspingPoses.size()):
            
            print("Lets check grasping pose: ", j)
            bGraspingPose = bGraspingPoses.get(j).asList()
            print(bGraspingPose.toString())
            graspingPose = []
            for i in range(bGraspingPose.size()):
                graspingPose.append(bGraspingPose.get(i).asFloat64())
            print(graspingPose)

            cmd = yarp.Bottle()
            response = yarp.Bottle()
      
            frame_end_base = self.vectorToFrame(graspingPose)
            z_axes = frame_end_base.M.UnitZ()
            
            if (True):

                frame_reaching_end = PyKDL.Frame()
                frame_reaching_end.p.z(-reachingDistance)

                frame_reaching_base = frame_end_base*frame_reaching_end

                reaching_pose = self.frameToVector(frame_reaching_base)
                    
                print("Let's check the reaching pose")
                
                cmd.addVocab32("chgp")
                goal = cmd.addList()
                for i in range(len(reaching_pose)):
                    goal.addFloat64(reaching_pose[i])
                self.rpcClientTrajectoryGeneration.write(cmd, response)
                
                print(response.toString())

                if response.get(0).asVocab32() == VOCAB_OK:
                    print("Valid reaching position")
                    aux_q = response.get(1).asList()
                    print(aux_q.toString())
                    if self.use_right_arm:
                        for i in range(self.numRightArmJoints):
                            reaching_q.append(aux_q.get(i).asFloat32())
                    else:
                        for i in range(self.numLeftArmJoints):
                            reaching_q.append(aux_q.get(i).asFloat32())
                    return True, graspingPose,grasping_q, reaching_pose, reaching_q
                else:
                    print("Not valid reaching position")
        return False, graspingPose,grasping_q, reaching_pose, reaching_q
    
    def moveTowardsObject(self):
        
        cmd  = yarp.Bottle()
        response = yarp.Bottle()
        cmd.addVocab32("cjlp")
        poses = yarp.Bottle()
        poses = cmd.addList()
        goal_q = []

        print("Aproach to object")

        if self.use_right_arm: 
            armCurrentQ = yarp.DVector(self.numRightArmJoints)
            if not self.rightArmIEncoders.getEncoders(armCurrentQ):
                print("Unable to get arm encoders position")
                raise SystemExit

            current_Q = yarp.DVector(self.numRightArmJoints)
            desire_Q = yarp.DVector(self.numRightArmJoints)
            aproachingQs = []
                            

            for j in range(0, self.numRightArmJoints):
                current_Q[j] = armCurrentQ[j]
                goal_q.append(current_Q[j])
        
            self.reachingPose = yarp.DVector(self.numRightArmJoints)
            self.trunkRightArmICartesianSolver.fwdKin(current_Q, self.reachingPose)
            
            print('> current_Q [%s]' % ', '.join(map(str, current_Q)))
            print('> reachingPose [%s]' % ', '.join(map(str, self.reachingPose)))

        # else:
        #     armCurrentQ = yarp.DVector(self.numRightArmJoints)
        #     if not self.rightArmIEncoders.getEncoders(armCurrentQ):
        #         print("Unable to get arm encoders position")
        #         raise SystemExit

        #     current_Q = yarp.DVector(self.numRightArmJoints)
        #     desire_Q = yarp.DVector(self.numRightArmJoints)
        #     aproachingQs = []
        #     goal_q = []
                            

        #     for j in range(0, self.numRightArmJoints):
        #         current_Q[j] = armCurrentQ[j]
        #         goal_q.append(current_Q[j])
        
        #     self.reachingPose = yarp.DVector(self.numRightArmJoints)
        #     self.trunkRightArmICartesianSolver.fwdKin(current_Q, self.reachingPose)
        frame_reaching_base = self.vectorToFrame(self.reachingPose)
                        
        # First we need to get the encoders for the first postion
        aproachingQs.append(goal_q)
        d = 0
        while d <= 1.2*self.reachingDistance:
            d = d + self.reachingDistance/100.0

            frame_aux_reaching = PyKDL.Frame()
            frame_aux_reaching.p.z(d)

            frame_aux_base = frame_reaching_base*frame_aux_reaching

            aux_pose = self.frameToVector(frame_aux_base)
            print(aux_pose)                
            x_vector = yarp.DVector(6)
            pose = yarp.Bottle()
            pose = poses.addList()
            for i in range(len(x_vector)):
                x_vector[i] = aux_pose[i]
                pose.addFloat64(x_vector[i])
            
        if self.use_right_arm:
            self.rpcClientTrajectoryGeneration.write(cmd, response)
            print(response.toString())
            bJointsTrajectory = response.get(1).asList()
            #print(bJointsTrajectory.toString())
            jointsTrajectory = []
            jointsTrajectory.append(goal_q)
            for i in range(bJointsTrajectory.size()):
                bJointsPosition = bJointsTrajectory.get(i).asList()
                jointsPosition = []
                for j in range(bJointsPosition.size()):
                    jointsPosition.append(bJointsPosition.get(j).asFloat64())
                jointsTrajectory.append(jointsPosition)   
            print(len(jointsTrajectory))
            smoothJointsTrajectory = self.computeSmoothJointsTrajectory(jointsTrajectory,500)
            self.followJointsTrajectory(smoothJointsTrajectory)

    def moveUpObject(self):
        print("Move up object")
        
        cmd  = yarp.Bottle()
        response = yarp.Bottle()
        cmd.addVocab32("cjlp")
        poses = yarp.Bottle()
        poses = cmd.addList()
        jointsTrajectory = []

        
        print("Aproach to object")

        armCurrentQ = yarp.DVector(self.numRightArmJoints)
        if not self.rightArmIEncoders.getEncoders(armCurrentQ):
            print("Unable to get arm encoders position")
            raise SystemExit

        current_Q = yarp.DVector(self.numRightArmJoints)
        desire_Q = yarp.DVector(self.numRightArmJoints)
        aproachingQs = []
        goal_q = []
                        

        for j in range(0, self.numRightArmJoints):
            current_Q[j] = armCurrentQ[j]
            goal_q.append(current_Q[j])
       
        self.reachingPose = yarp.DVector(self.numRightArmJoints)
        self.trunkRightArmICartesianSolver.fwdKin(current_Q, self.reachingPose)
        frame_reaching_base = self.vectorToFrame(self.reachingPose)
        
        # First we need to get the encoders for the first postion
        aproachingQs.append(goal_q)
        distance = 0
        frame_aux_base = frame_reaching_base
        init_z = frame_reaching_base.p.z()
        print("init_z", init_z)
        jointsTrajectory.append(goal_q)
        while distance <= 0.95*self.upDistance:
     
            distance = distance + self.upDistance/100.0

            frame_aux_base.p.z(init_z+distance)

            aux_pose = self.frameToVector(frame_aux_base)
                            
            x_vector = yarp.DVector(6)
            pose = yarp.Bottle()
            pose = poses.addList()
            for i in range(len(x_vector)):
                x_vector[i] = aux_pose[i]
                pose.addFloat64(x_vector[i])
            print(pose.toString())
        
        if self.use_right_arm:
            self.rpcClientTrajectoryGeneration.write(cmd, response)
            bJointsTrajectory = response.get(1).asList()
            print(bJointsTrajectory.toString())
            for i in range(bJointsTrajectory.size()):
                bJointsPosition = bJointsTrajectory.get(i).asList()
                jointsPosition = []
                for j in range(bJointsPosition.size()):
                    jointsPosition.append(bJointsPosition.get(j).asFloat64())
                jointsTrajectory.append(jointsPosition)   
            print(len(jointsTrajectory))
            smoothJointsTrajectory = self.computeSmoothJointsTrajectory(jointsTrajectory,200)
            self.followJointsTrajectory(smoothJointsTrajectory)
                      
      
    
    def state0(self, initTrunkPosition):
        
        writer = csv.writer(self.task_execution_csv)
        # write a row to the csv file
        

        
        # First we need to stop the /getGraspingPoses process
        cmd = yarp.Bottle()
        response = yarp.Bottle()
        cmd.addString('paus')
        self.rpcClientGetGraspingPoses.write(cmd, response)
        yarp.delay(1.0)
        
        writer.writerow(['------ State0: Moving the trunk to the initial position ------'])
        if self.use_right_arm:
            armCurrentQ = yarp.DVector(self.numRightArmJoints)
            if not self.rightArmIEncoders.getEncoders(armCurrentQ):
                print("Unable to get trunkAndRightArm encoders position")
                writer.writerow(['Unable to get trunkAndRightArm encoders position.'])
                raise SystemExit
            values = []
            for i in range(0,2):
                values.append(armCurrentQ[i])
            writer.writerow(['Current Joints Position: '+ str(values)])
            writer.writerow(['Goal Trunk Joints Position: '+str(initTrunkPosition)])
        
        self.init = time.time()
        writer.writerow(['Start_time: '+ str(self.init-self.init)])
        # Put only the trunk in the init position
        self.putTrunkToInitPositionWithControlPosition(initTrunkPosition)
        current = time.time()
        writer.writerow(['End_time: '+ str(current-self.init)])
        if self.use_right_arm:
            armCurrentQ = yarp.DVector(self.numRightArmJoints)
            if not self.rightArmIEncoders.getEncoders(armCurrentQ):
                print("Unable to get trunkAndRightArm encoders position")
                writer.writerow(['Unable to get trunkAndRightArm encoders position.'])
                raise SystemExit
            values = []
            for i in range(0,2):
                values.append(armCurrentQ[i])
        writer.writerow(['Trunk joints position: '+ str(values)])
        
        
        # rsm the getGraspingPoses server
        cmd = yarp.Bottle()
        response = yarp.Bottle()
        cmd.addString('rsm')
        self.rpcClientGetGraspingPoses.write(cmd, response)
        yarp.delay(2.0)
        
        # pause the getGraspingPoses server
        cmd = yarp.Bottle()
        response = yarp.Bottle()
        cmd.addString('paus')
        self.rpcClientGetGraspingPoses.write(cmd, response)
        yarp.delay(1.0)
        
        
        self.realsenseImagePort.read(self.yarp_image)
        current = time.time()
  
        im = Image.fromarray(self.img_array)
        aux = current-self.init
        im.save(self.path+"/rgb/"+ str(aux)+'.jpg')

        self.realsenseDepthImagePort.read(self.yarpImgDepth)            
        formatted = (self.depthImgArray/self.depthImgArray.max()*65535/2.0).astype('uint16')
        # print("depth max: ", depthImgArray.max())
        # print(formatted)
        self.imDepth = Image.fromarray(formatted)
        self.imDepth.save(self.path+"/depth/"+ str(aux)+'.png', 'png')

        writer.writerow(['Store rgb and depth images for timestamp: '+str(aux)])
        
        # get all the superquadrics from GetGraspingPoses server
        writer.writerow(['------ State0: Get Superquadrics ------'])
        start = time.time()
        cmd.clear()
        cmd.addString('gsup')
        self.rpcClientGetGraspingPoses.write(cmd, self.superquadrics)
        current = time.time()
        print(self.superquadrics.toString())
        writer.writerow(['Start_time: '+ str(start-self.init)])
        writer.writerow(['End_time: '+ str(current-self.init)])
        writer.writerow(['Superquadrics: '+ self.superquadrics.toString()])
        yarp.delay(4.0)

        # get all the superquadrics from GetGraspingPoses server
        writer.writerow(['------ State0: Get Superquadrics Bounding Boxes------'])
        start = time.time()
        cmd.clear()
        cmd.addString('gsbb')
        self.rpcClientGetGraspingPoses.write(cmd, self.superquadricsBoundingBoxes)
        current = time.time()
        writer.writerow(['Start_time: '+ str(start-self.init)])
        writer.writerow(['End_time: '+ str(current-self.init)])
        writer.writerow(['Superquadrics BBoxes: '+ self.superquadricsBoundingBoxes.toString()])
        print(self.superquadricsBoundingBoxes.toString())
        yarp.delay(2.0)
        
        

        writer.writerow(['------ State0: Get Object Detections------'])

        # get the object detections
        print("Now detections")
        start = time.time()
        self.detections = self.xtionObjectDetectionsPort.read(False)
        current = time.time()
        writer.writerow(['Start_time: '+ str(start-self.init)])
        writer.writerow(['End_time: '+ str(current-self.init)])
        writer.writerow(['Object Detections: '+ self.detections.toString()])

        print(self.detections.toString())
        
        
        start = time.time()
        self.setLabelSuperquadricsCategory()
        current = time.time()
        writer.writerow(['Start_time: '+ str(start-self.init)])
        writer.writerow(['End_time: '+ str(current-self.init)])
        writer.writerow(['Category associated to labeled supercuadrics: '+ self.labelSuperquadricsCategory.toString()])
        
        # print("Lets remove")
        # for i in range(self.superquadrics.size()):
        #     label_idx = self.superquadrics.get(i).find("label_idx").asInt32()
        #     print("label_idx: ", label_idx)
        #     found = False
        #     for j in range(self.labelSuperquadricsCategory.size()):
        #         if label_idx == self.labelSuperquadricsCategory.get(j).find("label_idx").asInt32():
        #             print("Found label_idx: ", label_idx)
        #             found = True
        #             break
        #     if not found:
        #         print("NOT found label_idx: ", label_idx)
        #         cmd.clear()
        #         cmd.addString('rsup')
        #         cmd.addInt32(label_idx)
        #         self.rpcClientGetGraspingPoses.write(cmd, response)
        #         print(response.toString())
        #         yarp.delay(0.5)
        
        # set the superquadrics in the trajectoryGenerator for collision checking
        print("set super  quadrics")
        cmd.clear()
        response.clear()
        cmd.addString('ssup')
        self.rpcClientTrajectoryGeneration.write(cmd, response)
        yarp.delay(1.0)
        print(response.toString())
        print(response.size())  
        
        
    def state1(self, initTrunkAndArm):
        found, jointsTrajectory = self.computeTrajectoryToJointsPosition(self.use_right_arm,initTrunkAndArm)
        
        if found: 
            if len(jointsTrajectory) < 100:
                nPoints = 800
            else:
                nPoints = 1000 
            smoothJointsTrajectory = self.computeSmoothJointsTrajectory(jointsTrajectory,nPoints)
            self.plotTrajectories(jointsTrajectory, smoothJointsTrajectory)
            self.followJointsTrajectory(smoothJointsTrajectory)
        
        
    def state2(self):
        # Here we wait for the command from asr
        print("State 2: Waiting for asr command object ...") 
        while self.asrData == None or self.asrData.toString() not in word_dict:
            self.asrData = self.asrPort.read(False)
        
        objectFound = False
        for i in range(self.labelSuperquadricsCategory.size()):
            self.currentObject = self.labelSuperquadricsCategory.get(i)
            if self.asrData.toString() in self.currentObject.find("category").asString():
                print("Object detected in the table.")
                objectFound = True
                break
        if not objectFound:
            raise SystemExit
        
        print("Lets grasp ", self.asrData.toString())
        print(self.currentObject.toString())
        print(self.asrData.toString())
        
    def state3(self):
        #Compute grasping poses
        print("Compute grasping poses")
        cmd = yarp.Bottle()
        cmd.addString('ggp')
        cmd.addInt32(self.currentObject.find("label_idx").asInt32())
        self.rpcClientGetGraspingPoses.write(cmd, self.bGraspingPoses)
        print(self.bGraspingPoses.toString())
        
        feasible, self.graspingPose, self.graspingQ, self.reachingPose, self.reachingQ = self.getFeasibleGraspingPose(self.bGraspingPoses, self.reachingDistance)

        if feasible:
            print(self.reachingPose)
            print(self.reachingQ)
        
            found, jointsTrajectory = self.computeTrajectoryToJointsPosition(self.use_right_arm, self.reachingQ)
            if len(jointsTrajectory) < 100:
                nPoints = 700
            else:
                nPoints = 1300 
            smoothJointsTrajectory = self.computeSmoothJointsTrajectory(jointsTrajectory,nPoints)
            self.plotTrajectories(jointsTrajectory, smoothJointsTrajectory)
            
            self.followJointsTrajectory(smoothJointsTrajectory)
            
            self.openHand()
            yarp.delay(2.0)
            
            cmd = yarp.Bottle()
            response = yarp.Bottle()

            cmd.addString('rsup')
            cmd.addInt32(self.currentObject.find("label_idx").asInt32())
            self.rpcClientGetGraspingPoses.write(cmd, response)
            print(response.toString())
            yarp.delay(2.0)

        
            print("set super quadrics")
            cmd.clear()
            response.clear()
            cmd.addString('ssup')
            self.rpcClientTrajectoryGeneration.write(cmd, response)
            print(response.toString())
            yarp.delay(0.5)     
            
            cmd.clear()
            response.clear()
            cmd.addString('gsup')
            self.rpcClientTrajectoryGeneration.write(cmd, response)
            print(response.toString())
            print("Move towards object")
            self.moveTowardsObject()
            
            print("Close rightHand")
            self.closeHand()
            yarp.delay(1.0)
            
            return True
        return False
    
    def openHand(self):
        print("Open Hand")
        if self.use_right_arm:
            if self.robot== "/teoSim":
                self.rightHandIPositionControl.positionMove(0, 1200)
            elif self.robot == "/teo":
                self.rightHandIPWMControl.setRefDutyCycle(0, 100)
        else:
            if self.robot== "/teoSim":
                self.leftHandIPositionControl.positionMove(0, 1200)
            elif self.robot == "/teo":
                self.leftHandIPWMControl.setRefDutyCycle(0, 100)

    def closeHand(self):
        print("Close Hand")
        if self.use_right_arm:
            if self.robot== "/teoSim":
                self.rightHandIPositionControl.positionMove(0, -200)
            elif self.robot == "/teo":
                self.rightHandIPWMControl.setRefDutyCycle(0, -100)
        else:
            if self.robot== "/teoSim":
                self.leftHandIPositionControl.positionMove(0, -200)
            elif self.robot == "/teo":
                self.leftHandIPWMControl.setRefDutyCycle(0, -100)

        
        
        
        
        
        
        
        
    
        
        
        
        
        
        
        
        
        
        
        
        
        
        

        

        
        
        

        