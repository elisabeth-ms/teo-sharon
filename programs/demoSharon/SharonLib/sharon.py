from sys import prefix
import yarp
import numpy as np
from scipy import interpolate
import time
import kinematics_dynamics
from SharonLib.config_asr import word_dict
import PyKDL

VOCAB_OK = yarp.createVocab32('o', 'k')
VOCAB_FAIL = yarp.createVocab32('f', 'a', 'i', 'l')

class Sharon():
    def __init__(self, robot, prefix, only_asr, use_right_arm, 
                 available_right_arm_controlboard, available_left_arm_controlboard,
                 available_trunk_controlboard, reachingDistance, upDistance):
        
        self.robot = robot
        self.prefix = prefix
        self.only_asr = only_asr
        self.use_right_arm = use_right_arm
        self.reachingDistance = reachingDistance 
        self.upDistance = upDistance 
        self.available_right_arm_controlboard = available_right_arm_controlboard
        self.available_left_arm_controlboard = available_left_arm_controlboard
        self.available_trunk_controlboard = available_trunk_controlboard
        print(self.robot)
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

        # Trajectory Generation client
        self.rpcClientTrajectoryGenerationRight = yarp.RpcClient()

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

        self.graspingPose=yarp.Bottle()
        self.graspingQ = yarp.Bottle()
        self.reachingPose = yarp.Bottle()
        self.reachingQ = yarp.Bottle()
        
        
        
    def configure(self):
        print("Configuring...")


        #Open asr port
        self.asrPort.open(self.prefix+"/asr:i")
        yarp.Network.connect("/asr:o", self.prefix+"/asr:i")
        
        # if self.available_trunk_controlboard:
        #     # Open trunk device
        #     self.trunkOptions.put('device', 'remote_controlboard')
        #     self.trunkOptions.put('remote', self.robot+'/trunk')
        #     self.trunkOptions.put('local', self.prefix+self.robot+'/trunk')
        #     # self.trunkOptions.put('writeStrict', 'on')

        #     self.trunkDevice = yarp.PolyDriver(self.trunkOptions)
        #     self.trunkDevice.open(self.trunkOptions)
        #     if not self.trunkDevice.isValid():
        #         print('Cannot open trunk device!')
        #         raise SystemExit
        #     print("trunk device opened!")
            
        #     self.trunkIEncoders = self.trunkDevice.viewIEncoders()

        #     if self.trunkIEncoders == []:
        #         print("Trunk Encoder interface NOT available.")
        #         raise SystemExit
        #     else:
        #         print("Trunk Encoder interface available.")

        #     self.numTrunkJoints = self.trunkIEncoders.getAxes()
            
        #     print("trunk has", self.numTrunkJoints, "joints.")
            
            
        #     self.trunkIControlMode = self.trunkDevice.viewIControlMode()
        #     if self.trunkIControlMode == []:
        #         print("Trunk control mode interface NOT available.")
        #         raise SystemExit
        #     else:
        #         print("Trunk control mode interface available.")
            
            
        #     # Trunk position control interface
        #     self.trunkIPositionControl = self.trunkDevice.viewIPositionControl()

        #     if self.trunkIPositionControl == []:
        #         print("Trunk position control interface NOT available")
        #         raise SystemExit
        #     else:
        #         print("Trunk position control interface available.")
            
        #     # Trunk position direct interface

        #     self.trunkIPositionDirect = self.trunkDevice.viewIPositionDirect()
        #     if self.trunkIPositionDirect == []:
        #         print("Trunk position direct interface NOT available.")
        #         raise SystemExit
        #     else:
        #         print("Trunk position direct interface available")
            
        #     #Trunk remote variables interface
        #     self.trunkIRemoteVariables = self.trunkDevice.viewIRemoteVariables()
        #     if self.trunkIRemoteVariables == []:
        #         print("Trunk remote variables interface NOT available.")
        #         raise SystemExit
        #     else:
        #         print("Trunk remote variables interface available.")
        
        
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
            self.trunkRightArmSolverDevice = yarp.PolyDriver(
            self.trunkRightArmSolverOptions)  # calls open -> connects
            self.trunkRightArmSolverDevice.open(self.trunkRightArmSolverOptions)

            if self.trunkRightArmSolverDevice == []:
                print("trunk right arm solver device interface NOT available")
                raise SystemExit
            else:
                print("trunk right arm solver device interface available.")
                
            # Trunk and right arm cartesian solver
            self.trunkRightArmICartesianSolver = kinematics_dynamics.viewICartesianSolver(self.trunkRightArmSolverDevice)
            if self.trunkRightArmICartesianSolver == []:
                print("Right arm cartesian solver interface NOT available")
                raise SystemExit
            else:
                print("Right arm cartesian solver interface available.")
            print(self.trunkRightArmSolverOptions.find("mins").toString())
            print(self.trunkRightArmSolverOptions.find("maxs").toString())

        
        if self.available_left_arm_controlboard:
            self.leftArmOptions.put('device', 'remote_controlboard')
            self.leftArmOptions.put('remote', self.robot+'/leftArm')
            self.leftArmOptions.put('local', self.prefix+self.robot+'/leftArm')
            self.leftArmDevice = yarp.PolyDriver(self.leftArmOptions)
            self.leftArmDevice.open(self.leftArmOptions)
            if not self.leftArmDevice.isValid():
                print('Cannot open leftArm device!')
                raise SystemExit
            print("leftArm device opened!")
        
        
        self.rpcClientGetGraspingPoses.open(prefix+"/getGraspingPoses/rpc:c")
        yarp.Network.connect(prefix+"/getGraspingPoses/rpc:c",
                             "/getGraspingPoses/rpc:s")

        if self.available_right_arm_controlboard:
            self.rpcClientTrajectoryGenerationRight.open('/trajectoryGeneration/trunkAndRightArm/rpc:c')
            yarp.Network.connect("/trajectoryGeneration/trunkAndRightArm/rpc:c",
                                "/trajectoryGeneration/trunkAndRightArm/rpc:s")
        
        self.xtionObjectDetectionsPort.open(self.prefix+"/rgbdObjectDetection/state:i")
        yarp.Network.connect("/rgbdObjectDetection/state:o", self.prefix+"/rgbdObjectDetection/state:i")
        
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
        rightArmModes = yarp.IVector(self.numRightArmJoints,yarp.VOCAB_CM_POSITION)
        if not self.rightArmIControlMode.setControlModes(rightArmModes):
            print("Unable to set right arm  to position  mode.")
        else:
            print("Right arm set to position  mode.")
        
        for j in range(len(initTrunkPosition)):
            self.rightArmIPositionControl.positionMove(j, initTrunkPosition[j])
        
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
            IoU = 0
        return False, IoU
    
    def setLabelSuperquadricsCategory(self):
        self.labelSuperquadricsCategory = yarp.Bottle()
        print(self.superquadricsBoundingBoxes.size())
        print(self.detections.size())
        
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
            IoU = 0
            auxSuperquadric = yarp.Bottle()
            for j in range(self.detections.size()):
                detection = self.detections.get(j)
                tlxDetection = detection.find("tlx").asFloat64()
                tlyDetection = detection.find("tly").asFloat64()
                brxDetection = detection.find("brx").asFloat64()
                bryDetection = detection.find("bry").asFloat64()
                bboxDetection = [tlxDetection, tlyDetection, brxDetection, bryDetection]
                print("j: ",j, bboxDetection)
                print(i,j)
                found,currentIoU = self.computeIntersectionOverUnion(bbox, bboxDetection)
                if currentIoU > IoU:
                    IoU = currentIoU
                    dictObject =  auxSuperquadric.addDict()  
                    dictObject.put('category',detection.find("category").asString() ) 
                    dictObject.put('label_idx',superquadricBB.find("label_idx").asInt32() )                    
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
        if self.use_right_arm:
            self.rpcClientTrajectoryGenerationRight.write(cmd, response)

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
            self.rpcClientTrajectoryGenerationRight.write(cmd, response)
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
                    jointsPosition.append(bJointsPosition.get(i).asDouble())
                jointsTrajectory.append(jointsPosition)
            print("JointsTrajectory")
            return True, jointsTrajectory
    
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
        period = 50
        
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
            raise SystemExit
    
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
            if self.use_right_arm and z_axes[1]>0:

                frame_reaching_end = PyKDL.Frame()
                frame_reaching_end.p.z(-reachingDistance)

                frame_reaching_base = frame_end_base*frame_reaching_end

                reaching_pose = self.frameToVector(frame_reaching_base)
                    
                print("Let's check the reaching pose")
                
                cmd.addVocab32("chgp")
                goal = cmd.addList()
                for i in range(len(reaching_pose)):
                    goal.addFloat64(reaching_pose[i])
                        
                if self.use_right_arm:
                    self.rpcClientTrajectoryGenerationRight.write(cmd, response)

                if response.get(0).asVocab32() == VOCAB_OK:
                    print("Valid reaching position")
                    aux_q = response.get(1).asList()
                    for i in range(self.numRightArmJoints):
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
        d = 0
        while d <= 1.2*self.reachingDistance:
            d = d + self.reachingDistance/100.0

            frame_aux_reaching = PyKDL.Frame()
            frame_aux_reaching.p.z(d)

            frame_aux_base = frame_reaching_base*frame_aux_reaching

            aux_pose = self.frameToVector(frame_aux_base)
                            
            x_vector = yarp.DVector(6)
            pose = yarp.Bottle()
            pose = poses.addList()
            for i in range(len(x_vector)):
                x_vector[i] = aux_pose[i]
                pose.addFloat64(x_vector[i])
            
        if self.use_right_arm:
            self.rpcClientTrajectoryGenerationRight.write(cmd, response)
            print(response.toString())
            bJointsTrajectory = response.get(1).asList()
            #print(bJointsTrajectory.toString())
            jointsTrajectory = []
            for i in range(bJointsTrajectory.size()):
                bJointsPosition = bJointsTrajectory.get(i).asList()
                jointsPosition = []
                for j in range(bJointsPosition.size()):
                    jointsPosition.append(bJointsPosition.get(j).asDouble())
                jointsTrajectory.append(jointsPosition)   
            print(len(jointsTrajectory))
            smoothJointsTrajectory = self.computeSmoothJointsTrajectory(jointsTrajectory,200)
            self.followJointsTrajectory(smoothJointsTrajectory)

    def moveUpObject(self):
        print("Move up object")
        
        cmd  = yarp.Bottle()
        response = yarp.Bottle()
        cmd.addVocab32("cjlp")
        poses = yarp.Bottle()
        poses = cmd.addList()
        
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
            self.rpcClientTrajectoryGenerationRight.write(cmd, response)
            bJointsTrajectory = response.get(1).asList()
            print(bJointsTrajectory.toString())
            jointsTrajectory = []
            for i in range(bJointsTrajectory.size()):
                bJointsPosition = bJointsTrajectory.get(i).asList()
                jointsPosition = []
                for j in range(bJointsPosition.size()):
                    jointsPosition.append(bJointsPosition.get(j).asDouble())
                jointsTrajectory.append(jointsPosition)   
            print(len(jointsTrajectory))
            smoothJointsTrajectory = self.computeSmoothJointsTrajectory(jointsTrajectory,200)
            self.followJointsTrajectory(smoothJointsTrajectory)
                      
      
    
    def state0(self, initTrunkPosition):
        # First we need to stop the /getGraspingPoses process
        cmd = yarp.Bottle()
        response = yarp.Bottle()
        cmd.addString('paus')
        self.rpcClientGetGraspingPoses.write(cmd, response)
        yarp.delay(1.0)
        
        # Put only the trunk in the init position
        self.putTrunkToInitPositionWithControlPosition(initTrunkPosition)
        
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
        
        # get all the superquadrics from GetGraspingPoses server
        cmd.clear()
        cmd.addString('gsup')
        self.rpcClientGetGraspingPoses.write(cmd, self.superquadrics)
        print(self.superquadrics.toString())
        yarp.delay(1.0)

        # get all the superquadrics from GetGraspingPoses server
        cmd.clear()
        cmd.addString('gsbb')
        self.rpcClientGetGraspingPoses.write(cmd, self.superquadricsBoundingBoxes)
        print(self.superquadricsBoundingBoxes.toString())
        yarp.delay(1.0)

        # set the superquadrics in the trajectoryGenerator for collision checking
        print("set super  quadrics")
        cmd.clear()
        response.clear()
        cmd.addString('ssup')
        self.rpcClientTrajectoryGenerationRight.write(cmd, response)
        print(response.toString())
        print(response.size())
        
        
        # get the object detections
        print("Now detections")
        self.detections = self.xtionObjectDetectionsPort.read(False)
        print(self.detections.toString())
        
        self.setLabelSuperquadricsCategory()
        
    def state1(self, initTrunkAndArm):
        found, jointsTrajectory = self.computeTrajectoryToJointsPosition(self.use_right_arm,initTrunkAndArm)
        if len(jointsTrajectory) < 100:
            nPoints = 400
        else:
             nPoints = 1000 
        smoothJointsTrajectory = self.computeSmoothJointsTrajectory(jointsTrajectory,nPoints)
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
                nPoints = 400
            else:
                nPoints = 1000 
            smoothJointsTrajectory = self.computeSmoothJointsTrajectory(jointsTrajectory,nPoints)
            self.followJointsTrajectory(smoothJointsTrajectory)
            
            cmd = yarp.Bottle()
            response = yarp.Bottle()

            cmd.addString('rsup')
            cmd.addInt32(self.currentObject.find("label_idx").asInt32())
            self.rpcClientGetGraspingPoses.write(cmd, response)
            print(response.toString())
            yarp.delay(0.1)

        
            print("set super quadrics")
            cmd.clear()
            response.clear()
            cmd.addString('ssup')
            self.rpcClientTrajectoryGenerationRight.write(cmd, response)
            print(response.toString())            
            
            cmd.clear()
            response.clear()
            cmd.addString('gsup')
            self.rpcClientTrajectoryGenerationRight.write(cmd, response)
            print(response.toString())
            # print("Move towards object")
            # self.moveTowardsObject()
            
            # print("Close rightHand")
            # self.rightHandIPositionControl.positionMove(0, -200)
            # yarp.delay(5.0)
            
            ##### TODO: CONTINUE UP OBJECT MOTION
    def openHand(self):
        print("Open rightHand")
        self.rightHandIPositionControl.positionMove(0, 1200)
        
    def closeHand(self):
        print("Close rightHand")
        self.rightHandIPositionControl.positionMove(0, -200)
        
        
        
        
        
        
        
        
    
        
        
        
        
        
        
        
        
        
        
        
        
        
        

        

        
        
        

        