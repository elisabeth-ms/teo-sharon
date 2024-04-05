import yarp
import sys
import PyKDL
import numpy as np
import kinematics_dynamics
from pynput import keyboard
import time

import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate
from matplotlib.ticker import MaxNLocator

from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
import rclpy
from rclpy.node import Node
import PyKDL

robot = '/teo'
prefix = '/demoSharon'
camera = '/teo/realsense2'
available_right_hand_controlboard = False
available_left_hand_controlboard = False
available_right_arm_controlboard = True
available_left_arm_controlboard = True
available_head_controlboard = False
available_trunk_controlboard = True
available_speak_device = True

only_asr = True
VOCAB_OK = yarp.createVocab32('o', 'k')
VOCAB_FAIL = yarp.createVocab32('f', 'a', 'i', 'l')
VOCAB_START_SUPERQUADRICS = yarp.createVocab32('r', 's', 'm')

class Ros2Publisher(Node):
  def __init__(self):
    super().__init__('ros2_publisher')
    self.publisher_maker_array = self.create_publisher(MarkerArray, 'grasp_pose', 10)
  



class DemoSharon(yarp.RFModule):
    def __init__(self):
        yarp.RFModule.__init__(self)
        rclpy.init()
        
        self.ros2_publisher_node = Ros2Publisher()
        
        # Right Arm device
        self.use_right_arm = False
        self.rightArmOptions = yarp.Property()
        self.rightArmDevice = None
        self.rightArmIEncoders = None
        self.numRightArmJoints = 0
        self.rightArmIPositionControl = None
        self.rightArmIPositionControl = None
        self.rightArmIControlMode = None
        self.rightArmIPositionDirect = None
        self.rightArmIRemoteVariables = False

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
        self.leftArmIPositionDirect= None
        self.leftArmIControlMode= None
        self.leftArmIRemoteVariables = False

        # Left hand device
        self.leftHandOptions = yarp.Property()
        self.leftHandDevice = None
        self.leftHandIEncoders = None
        self.numLeftHandJoints = 0
        self.leftHandIPositionControl = None
        self.leftHandIPWMControl = None

        # Trunk device
        self.trunkOptions = yarp.Property()
        self.trunkDevice = None
        self.trunkIEncoders = None
        self.numTrunkJoints = 2
        self.trunkIPositionControl = None
        self.trunkIControlMode = None
        self.trunkIPositionDirect = None
        self.trunkIRemoteVariables = False

        # Head device
        self.headOptions = yarp.Property()
        self.headDevice = None
        self.headIEncoders = None
        self.numHeadJoints = 2
        self.headIPositionControl = None
        self.trunkHeadIControlLimits = None

        self.cart = None

        # Trajectory Generation client
        self.rpcClientTrajectoryGenerationRight = yarp.RpcClient()
        self.rpcClientTrajectoryGenerationLeft = yarp.RpcClient()

        # Get Grasping poses client
        self.rpcClientGetGraspingPoses = yarp.RpcClient()

        # tts client
        self.rpcClientTts = yarp.RpcClient()


        # State
        self.state = 0
        self.firstInState = True  # For printing messages just once
        self.jointsPositionError = 2.5
        
        self.max_velocities = [20]*8
        self.period = 0.05


        # Init Joints position for grasping
        self.initTrunkJointsPosition = [0, 14.0]
        #self.initTrunkJointsPosition = [0, 0.0]
        self.initBackTrunkJointsPosition = [0, -9.0]
        self.initHeadJointsPosition = [0, -28.0]

        # [-90, 0, 0, 0, 90, 0]
        # self.initRightArmJointsPosition = [-50, -50, 40, -70, 30, -20]
        self.initRightArmJointsPosition = [-50, -50, 40, -70, 30, -20]
        
        self.initRightArmJointsPosition2 = [-50, -50, 40, -70, 30, -20]
        #self.initRightArmJointsPosition2 = [0, 0, 0, 0, 0, 0]
        self.initLeftArmJointsPosition = [-50, 50, -40, -70, -30, -20]
        self.initLeftArmJointsPosition2 = [-50, 50, -40, -70, -30, -20]

        # Object category and position port
        self.objectPositionPort = yarp.BufferedPortBottle()
        self.objectData = yarp.Bottle()
        self.asrPort = yarp.BufferedPortBottle()
        self.asrData = yarp.Bottle()
        self.xtionObjectDetectionsPort = yarp.BufferedPortBottle()


        self.category = None
        self.locationTCP = []
        self.orientationTCP = []
        self.jointsTrajectory = []
        self.aux = []
        self.numPointTrajectory = 0
        self.d = 0
        self.reachingDistance = 0

        # tcp frame wrt trunk when graspping
        self.frame_tcp_trunk = None
        self.up_distance = 0.1
        self.commands_pwm = 0

        # Trunk and Right Arm solver device
        self.trunkRightArmSolverOptions = yarp.Property()
        self.trunkRightArmSolverDevice = None

        # Trunk and Left Arm solver device
        self.trunkLeftArmSolverOptions = yarp.Property()
        self.trunkLeftArmSolverDevice = None

        # Trunk and head solver device
        self.trunkHeadSolverOptions = yarp.Property()
        self.trunkHeadSolverDevice = None

        self.rightArmJointsInPosition = [
            False, False, False, False, False, False]
        self.leftArmJointsInPosition = [
            False, False, False, False, False, False]
        self.minsTrunkAndRightArm = None
        self.maxsTrunkAndRightArm = None
        self.enter_is_pressed = False

        # For plotting the path

        self.smoothJointsTrajectoryTrunk = []
        self.smoothJointsTrajectoryRightArm = []
        self.adjustedTrajectoryTrunk = []
        self.adjustedTrajectoryArm = []

        self.matching_object_output_port_name = "/matching/object:o"
        self.demo_object_input_port_name = "/demoSharon/object:i"
        

    def config_device(self, options, device_name):
        device = yarp.PolyDriver(options)
        device.open(options)
        if not device.isValid():
            print('Cannot open device '+device_name)
            raise SystemExit
        else:
            print(device_name + " device opened.")
            return device
    
    def get_device_interfaces(self, device, device_name):
        iEncoders = device.viewIEncoders()
        if iEncoders == []:
            print(device_name + " Encoder interface NOT available.")
            raise SystemExit
        else:
            print(device_name + " Encoder interface available.")
        
        iPositionControl = device.viewIPositionControl()
        if iPositionControl == []:
            print(device_name + " position control interface NOT available")
            raise SystemExit
        else:
            print(device_name + " position control interface available.")
            
        iControlMode = device.viewIControlMode()
        if iControlMode == []:
            print(device_name + " control mode interface NOT available.")
            raise SystemExit
        else:
            print(device_name + " control mode interface available.")
        
        iPositionDirect = device.viewIPositionDirect()
        if iPositionDirect == []:
            print(device_name + " position direct interface NOT available.")
            raise SystemExit
        else:
            print(device_name + " position direct interface available")



        iRemoteVariables = device.viewIRemoteVariables()
        if iRemoteVariables == []:
            print(device_name + " remote variables interface NOT available.")
            raise SystemExit
        else:
            print(device_name + " remote variables interface available.")

        return iEncoders, iPositionControl, iPositionDirect, iControlMode, iRemoteVariables

    
    def configure(self, rf):
        print("Configuring...")

        self.objectPositionPort.open(self.demo_object_input_port_name)

        if(not only_asr):
            yarp.Network.connect(self.matching_object_output_port_name, self.demo_object_input_port_name)

        self.asrPort.open("/demoSharon/asr:i")
        yarp.Network.connect("/asr:o", "/demoSharon/asr:i")
        
        if available_trunk_controlboard:
            # Open trunk device
            self.trunkOptions.put('device', 'remote_controlboard')
            self.trunkOptions.put('remote', robot+'/trunk')
            self.trunkOptions.put('local', prefix+robot+'/trunk')
            self.trunkDevice = self.config_device(self.trunkOptions, "trunk")
            self.trunkIEncoders, self.trunkIPositionControl, self.trunkIPositionDirect, self.trunkIControlMode, self.trunkIRemoteVariables = self.get_device_interfaces(self.trunkDevice, "trunk")
            self.numTrunkJoints = self.trunkIEncoders.getAxes()
            # trunkModes = yarp.IVector(2, yarp.VOCAB_CM_POSITION_DIRECT)
            # if not self.trunkIControlMode.setControlModes(trunkModes):
            #     print("Unable to set trunk to position direct mode.")
            #     raise SystemExit
            # else:
            #     print("Trunk set to position direct mode.")

            
        if available_head_controlboard:
            # Open head device
            self.headOptions.put('device', 'remote_controlboard')
            self.headOptions.put('remote', robot+'/head')
            self.headOptions.put('local', robot+'/head')
            self.headDevice = self.config_device(self.headOptions, "head")
            self.headIEncoders, self.headIPositionControl, self.headIPositionDirect, self.headIControlMode, self.headIRemoteVariables = self.get_device_interfaces(self.headDevice, "head")
            self.numHeadJoints = self.headIEncoders.getAxes()
            
        if available_right_arm_controlboard:
            # Open rightArm device
            self.rightArmOptions.put('device', 'remote_controlboard')
            self.rightArmOptions.put('remote', robot+'/rightArm')
            self.rightArmOptions.put('local', prefix+robot+'/rightArm')
            # self.rightArmOptions.put('writeStrict', 'on')
            self.rightArmDevice = self.config_device(self.rightArmOptions, "rightArm")
            self.rightArmIEncoders, self.rightArmIPositionControl, self.rightArmIPositionDirect, self.rightArmIControlMode, self.rightArmIRemoteVariables = self.get_device_interfaces(self.rightArmDevice, "rightArm")

            self.numRightArmJoints = self.rightArmIEncoders.getAxes()
            armCurrentQ = yarp.DVector(self.numRightArmJoints)
            self.rightArmIEncoders.getEncoders(armCurrentQ)
            for j in range(0, self.numRightArmJoints):
                print(armCurrentQ[j])

            self.checkJointsPosition(self.initRightArmJointsPosition2, self.rightArmIEncoders, self.numRightArmJoints)

            
        if available_left_arm_controlboard:
            # Open leftArm device
            self.leftArmOptions.put('device', 'remote_controlboard')
            self.leftArmOptions.put('remote', robot+'/leftArm')
            self.leftArmOptions.put('local', robot+'/leftArm')
            self.leftArmDevice = self.config_device(self.leftArmOptions, "leftArm")
            self.leftArmIEncoders, self.leftArmIPositionControl, self.leftArmIPositionDirect, self.leftArmIControlMode, self.leftArmIRemoteVariables = self.get_device_interfaces(self.leftArmDevice, "leftArm") 
            self.numLeftArmJoints = self.leftArmIEncoders.getAxes()
            
            armCurrentQ = yarp.DVector(self.numLeftArmJoints)
            self.leftArmIEncoders.getEncoders(armCurrentQ)
            for j in range(0, self.numLeftArmJoints):
                print(armCurrentQ[j])
            self.checkJointsPosition(self.initLeftArmJointsPosition2, self.leftArmIEncoders, self.numLeftArmJoints)
        
        if available_right_hand_controlboard:
            # Open rightHand device
            self.rightHandOptions.put('device', 'remote_controlboard')
            self.rightHandOptions.put('remote', robot+'/rightHand')
            self.rightHandOptions.put('local', robot+'/rightHand')
            self.rightHandDevice = self.config_device(self.rightHandOptions, "rightHand")

     
        if available_left_hand_controlboard:
            # Open leftHand device
            self.leftHandOptions.put('device', 'remote_controlboard')
            self.leftHandOptions.put('remote', robot+'/leftHand')
            self.leftHandOptions.put('local', robot+'/leftHand')

            self.leftHandDevice = self.config_device(self.leftHandOptions, "leftHand")
            
            
           

        # Right hand position control interface
        if available_right_hand_controlboard and robot == '/teoSim':
            self.rightHandIPositionControl = self.rightHandDevice.viewIPositionControl()

            if self.rightHandIPositionControl == []:
                print("Right hand position control interface NOT available")
                raise SystemExit
            else:
                print("Right hand position control interface available.")
        elif available_right_hand_controlboard and robot == '/teo':
            self.rightHandIPWMControl = self.rightHandDevice.viewIPWMControl()


        if available_right_arm_controlboard:
          # trunk and right arm solver device
          rf.setDefaultContext("kinematics")
          trunkRightKinPath = rf.findFileByName("teo-trunk-rightArm-fetch.ini")
          self.trunkRightArmSolverOptions.fromConfigFile(trunkRightKinPath)
          self.trunkRightArmSolverOptions.put("device", "KdlSolver")
          self.trunkRightArmSolverOptions.put("ik", "nrjl")

          self.trunkRightArmSolverOptions.fromString(
              "(mins (-30 -10.0 -98.1 -75.5 -80.1 -99.6 -80.4 -115.4))", False)
          self.trunkRightArmSolverOptions.fromString(
              "(maxs (30 16.0 106 22.4 57 98.4 99.6 44.7))", False)
          print("mins")
          print(self.trunkRightArmSolverOptions.find("mins").toString())
          print("maxs")
          print(self.trunkRightArmSolverOptions.find("maxs").toString())

          self.minsTrunkAndRightArm = self.trunkRightArmSolverOptions.find(
              "mins").asList()
          self.maxsTrunkAndRightArm = self.trunkRightArmSolverOptions.find(
              "maxs").asList()

          self.trunkRightArmSolverDevice = yarp.PolyDriver(
              self.trunkRightArmSolverOptions)  # calls open -> connects
          self.trunkRightArmSolverDevice.open(self.trunkRightArmSolverOptions)

          if self.trunkRightArmSolverDevice == []:
              print("trunk right arm solver device interface NOT available")
          else:
              print("trunk right arm solver device interface available.")

          # Trunk and right arm cartesian solver
          self.trunkRightArmICartesianSolver = kinematics_dynamics.viewICartesianSolver(
              self.trunkRightArmSolverDevice)
          if self.trunkRightArmICartesianSolver == []:
              print("Right arm cartesian solver interface NOT available")
          else:
              print("Right arm cartesian solver interface available.")

        if available_left_arm_controlboard:
          rf.setDefaultContext("kinematics")

          trunkLeftKinPath = rf.findFileByName("teo-trunk-leftArm-fetch.ini")
          
          print("Aquiiiiiiii")
          self.trunkLeftArmSolverOptions.fromConfigFile(trunkLeftKinPath)
          self.trunkLeftArmSolverOptions.put("device", "KdlSolver")
          self.trunkLeftArmSolverOptions.put("ik", "nrjl")

          self.trunkLeftArmSolverOptions.fromString(
              "(mins (-40 -10.1 -98.1 -75.5 -80.1 -99.6 -80.4 -115.4))", False)
          self.trunkLeftArmSolverOptions.fromString(
              "(maxs (40 16.0 106 22.4 57 98.4 99.6 44.7))", False)
          print("mins")
          print(self.trunkLeftArmSolverOptions.find("mins").toString())
          print("maxs")
          print(self.trunkLeftArmSolverOptions.find("maxs").toString())

          self.trunkLeftArmSolverDevice = yarp.PolyDriver(
              self.trunkLeftArmSolverOptions)  # calls open -> connects
          self.trunkLeftArmSolverDevice.open(self.trunkLeftArmSolverOptions)

          if self.trunkLeftArmSolverDevice == []:
              print("trunk left arm solver device interface NOT available")
          else:
              print("trunk left arm solver device interface available.")

          # Trunk and right arm cartesian solver
          self.trunkLeftArmICartesianSolver = kinematics_dynamics.viewICartesianSolver(
              self.trunkLeftArmSolverDevice)
          if self.trunkLeftArmICartesianSolver == []:
              print("Left arm cartesian solver interface NOT available")
          else:
              print("Left arm cartesian solver interface available.")

        # Interface with head encoders
        if available_head_controlboard:
            self.headIEncoders = self.headDevice.viewIEncoders()

            if self.headIEncoders == []:
                print("Head Encoder interface NOT available.")
                raise SystemExit
            else:
                print("Head Encoder interface available.")
            self.numHeadJoints = self.headIEncoders.getAxes()

            # Head position control interface
            self.headIPositionControl = self.headDevice.viewIPositionControl()

            if self.headIPositionControl == []:
                print("Head position control interface NOT available")
                raise SystemExit
            else:
                print("Head position control interface available.")



        # Open rpc client for /trajectoryGeneration/trunkAndRightArm
        if available_right_arm_controlboard:
          self.rpcClientTrajectoryGenerationRight.open(
              '/trajectoryGeneration/trunkAndRightArm/rpc:c')
          yarp.Network.connect("/trajectoryGeneration/trunkAndRightArm/rpc:c",
                              "/trajectoryGeneration/trunkAndRightArm/rpc:s")

        # Open rpc client for /trajectoryGeneration/trunkAndLeftArm
        if available_left_arm_controlboard:
          self.rpcClientTrajectoryGenerationLeft.open(
              '/trajectoryGeneration/trunkAndLeftArm/rpc:c')
          yarp.Network.connect("/trajectoryGeneration/trunkAndLeftArm/rpc:c",
                              "/trajectoryGeneration/trunkAndLeftArm/rpc:s")

        if camera == '/teo/realsense2':
            self.rpcClientGetGraspingPoses.open(
                "/getGraspingPoses/xtion/rpc:c")
            yarp.Network.connect("/getGraspingPoses/xtion/rpc:c",
                             "/getGraspingPoses/xtion/rpc:s")
        elif camera == "/teoSim/camera":
            self.rpcClientGetGraspingPoses.open(
                "/getGraspingPoses/teoSim/camera/rpc:c")
            yarp.Network.connect("/getGraspingPoses/teoSim/camera/rpc:c",
                             "/getGraspingPoses/teoSim/camera/rpc:s")
        else:
            print("robot must be /teo or /teoSim")
            raise SystemExit
        self.rpcClientGetGraspingPoses.open(
                "/getGraspingPoses/rpc:c")
        yarp.Network.connect("/getGraspingPoses/rpc:c",
                             "/getGraspingPoses/rpc:s")  
        
        self.rpcClientTts.open("/teo/tts/rpc:c")
        yarp.Network.connect("/teo/tts/rpc:c", "/teo/tts/rpc:s")


        self.xtionObjectDetectionPortName = "/rgbdObjectDetection/state"
        self.detections = None
        self.asrData = None
       
        self.xtionObjectDetectionsPort.open("/demoSharon"+self.xtionObjectDetectionPortName+":i")
        yarp.Network.connect(self.xtionObjectDetectionPortName+":o", "/demoSharon"+self.xtionObjectDetectionPortName+":i")

        self.listener = keyboard.Listener(on_release=self.on_release)
        self.listener.start()

        print("demo_sharon Module initialized...")
        print(self.numRightArmJoints)
        if self.use_right_arm:
            print(self.checkJointsPosition(
            self.initRightArmJointsPosition2, self.rightArmIEncoders, self.numRightArmJoints))
            self.checkJointsPosition(
            self.initTrunkJointsPosition, self.trunkIEncoders, self.numTrunkJoints)
        else:
            print(self.checkJointsPosition(
            self.initLeftArmJointsPosition2, self.leftArmIEncoders, self.numLeftArmJoints))
            self.checkJointsPosition(
            self.initTrunkJointsPosition, self.trunkIEncoders, self.numTrunkJoints)
        self.runWithAsr()
        return True

    def interruptModel(self):
        print("Stopping the module")

    def close(self):
        print("close")
        return True

    def getPeriod(self):
        return 2.0

    def runWithAsr(self):
        while True:
            try:
                if self.state == 0:  # Put the trunk joints in the init state
                    self.doState0()
                if self.state == 1:
                    self.doState1()
                    print("We are at state 1")
                if self.state == 2:
                    self.doState2()
                if self.state == 3:
                    self.doState3()

               
                
            except KeyboardInterrupt:
                print("Press Ctrl-C to terminate while statement")
                break
            pass
        print("latter")

                

      
    def doState0(self):
        if self.firstInState:
            # cmd = yarp.Bottle()
            # cmd.addVocab32(VOCAB_START_SUPERQUADRICS)
            # response = yarp.Bottle()
            # print(cmd.toString())
            # self.rpcClientGetGraspingPoses.write(cmd, response)
            # print(cmd.toString())
            # self.rpcClientGetGraspingPoses.write(cmd, response)
            # yarp.delay(5.0)
            
            if self.use_right_arm:
                if (self.checkJointsPosition(self.initTrunkJointsPosition, self.trunkIEncoders, self.numTrunkJoints) and self.checkJointsPosition(self.initRightArmJointsPosition2, self.rightArmIEncoders, self.numRightArmJoints)):
                    print("State 0: Trunk and right arm are in the init position")
                    self.state = 1
                    self.firstInState = True
                    self.sayOnce = True
                else:
                    # self.moveJointsInsideBounds()
                    initTrunkAndRightArmPosition = self.initTrunkJointsPosition + self.initRightArmJointsPosition2
                    print("i:", initTrunkAndRightArmPosition)
                    self.jointsTrajectory = []
                    found, self.jointsTrajectory = self.computeTrajectoryToJointsPosition(self.use_right_arm, initTrunkAndRightArmPosition)
                    if found:
                        self.numPointTrajectory = 0
                        # Lets plot the trajectory
                        print(len(self.jointsTrajectory))
                        print(self.jointsTrajectory)
                        nPoints = 0
                        if len(self.jointsTrajectory) < 100:
                            nPoints = 400
                        else:
                            nPoints = 1000
                        self.smoothJointsTrajectoryTrunk, self.smoothJointsTrajectoryRightArm = self.computeSmoothJointsTrajectory(nPoints)
                        self.adjustedTrajectoryTrunk, self.adjustedTrajectoryArm = self.adjust_trajectory_to_velocity_limits(self.smoothJointsTrajectoryTrunk,
                                                                                                       self.smoothJointsTrajectoryRightArm, 
                                                                                                       self.max_velocities, self.period)
                        self.plotTrajectories(self.jointsTrajectory, self.smoothJointsTrajectoryTrunk,self.smoothJointsTrajectoryRightArm, 
                                              self.adjustedTrajectoryTrunk, self.adjustedTrajectoryArm, self.period)
                        self.firstInState = False
            else: #Use left arm
                if (self.checkJointsPosition(self.initTrunkJointsPosition, self.trunkIEncoders, self.numTrunkJoints) and self.checkJointsPosition(self.initLeftArmJointsPosition2, self.leftArmIEncoders, self.numLeftArmJoints)):
                    print("State 0: Trunk and left arm are in the init position")
                    self.state = 1
                    self.firstInState = True
                    self.sayOnce = True
                else:
                    # self.moveJointsInsideBounds()
                    initTrunkAndLeftArmPosition = self.initTrunkJointsPosition + self.initLeftArmJointsPosition2
                    print("i:", initTrunkAndLeftArmPosition)
                    self.jointsTrajectory = []
                    found, self.jointsTrajectory = self.computeTrajectoryToJointsPosition(self.use_right_arm, initTrunkAndLeftArmPosition)
                    
                    if found:
                        self.numPointTrajectory = 0
                        # Lets plot the trajectory
                        print(len(self.jointsTrajectory))
                        print(self.jointsTrajectory)
                        nPoints = 0
                        if len(self.jointsTrajectory) < 100:
                            nPoints = 400
                        else:
                            nPoints = 1000
                        self.smoothJointsTrajectoryTrunk, self.smoothJointsTrajectoryLeftArm = self.computeSmoothJointsTrajectory(nPoints)
                        self.adjustedTrajectoryTrunk, self.adjustedTrajectoryArm = self.adjust_trajectory_to_velocity_limits(self.smoothJointsTrajectoryTrunk,
                                                                                                       self.smoothJointsTrajectoryLeftArm, 
                                                                                                       self.max_velocities, self.period)
                        
                        self.plotTrajectories(self.jointsTrajectory, self.smoothJointsTrajectoryTrunk,self.smoothJointsTrajectoryLeftArm, 
                                              self.adjustedTrajectoryTrunk, self.adjustedTrajectoryArm, self.period)
                        self.firstInState = False
        else:
            if self.use_right_arm:
                if (self.followJointsTrajectory(self.use_right_arm, self.adjustedTrajectoryTrunk, self.adjustedTrajectoryArm)):
                    self.state = 1
                    self.firstInState = True
                    self.sayOnce = True
            else:
                if (self.followJointsTrajectory(self.use_right_arm, self.adjustedTrajectoryTrunk, self.adjustedTrajectoryArm)):
                    self.state = 1
                    self.firstInState = True
                    self.sayOnce = True
    
    def doState1(self):
                    
        if available_head_controlboard:
            for joint in range(self.numHeadJoints):
                self.headIPositionControl.positionMove(joint, self.initHeadJointsPosition[joint])
                print("joint: ", joint, "value command: ",self.initHeadJointsPosition[joint] )
            if (self.checkJointsPosition(self.initHeadJointsPosition, self.headIEncoders, self.numHeadJoints)):
                print("State 1: Head joints are in the init position.")
                self.state = 2
                self.firstInState = True
                self.sayOnce = True
                yarp.delay(1.0)
        else:
            if self.firstInState:
                print("State 1: Continue the head joints motion towards the init position.")
                self.firstInState = False
            else:
                self.state = 2
                self.firstInState = True
    
    def doState2(self):
        if self.firstInState:
          cmd = yarp.Bottle()                    
          cmd.addVocab32('ggp')
          cmd.addInt8(2)
          response = yarp.Bottle()
          print(cmd.toString())
          self.rpcClientGetGraspingPoses.write(cmd, response)
          print(response.toString())
          
          bGraspingPoses = response
          print(bGraspingPoses.size())
          # bGraspingPoses = allb.get(1).asList()
          print(bGraspingPoses.size())
          if robot == "/teo":
              self.reachingDistance = 0.05
          elif robot == "/teoSim":
              self.reachingDistance = 0.05
          feasible, self.graspingPose, self.graspingQ, self.reachingPose, self.reachingQ = self.getFeasibleGraspingPose(self.use_right_arm, bGraspingPoses, self.reachingDistance)
          
          
          print("Feasible: ", feasible)
          print("Grasping pose: ",self.graspingPose)
          print("Reaching pose: ",self.reachingPose)
          print("Grasping Q: ",self.graspingQ)
          print("Reaching Q: ",self.reachingQ)
                  # feasible, self.graspingPose, self.reachingPose = self.getFeasibleGraspingPose(self.rightArm, bGraspingPoses, self.reachingDistance)
                                      
          if feasible:
              print("Grasping pose: ",self.graspingPose)
              print("Reaching pose: ",self.reachingPose)
              
              # We have a feasible grasping pose so lets publish in a ros2 topic to visualize it in rviz
              
              vector_rotation = PyKDL.Vector(self.graspingPose[3], self.graspingPose[4], self.graspingPose[5])
              grasp_pose_frame = PyKDL.Frame(PyKDL.Rotation.Rot(vector_rotation, vector_rotation.Norm()),
                                             PyKDL.Vector(self.graspingPose[0], self.graspingPose[1], self.graspingPose[2]))
              
              unit_x = grasp_pose_frame.M.UnitX()
              unit_y = grasp_pose_frame.M.UnitY()
              unit_z = grasp_pose_frame.M.UnitZ()
              
              print("unit_x: ", unit_x)
              print("unit_y: ", unit_y)
              print("unit_z: ", unit_z)
              
              marker_array = MarkerArray()
              marker = Marker()
              marker.header.frame_id = "waist"
              marker.header.stamp = self.ros2_publisher_node.get_clock().now().to_msg()

              marker.action = Marker.DELETEALL
              marker_array.markers.append(marker)
              
              self.ros2_publisher_node.publisher_maker_array.publish(marker_array)



              lengthArrow = 0.1
              auxV1 = unit_z * lengthArrow
              marker.id = 0
              marker.type = Marker.ARROW
              marker.action = Marker.ADD
    # geometry_msgs::msg::Point pointRos;
    # pointRos.x = frame.p[0] - auxV1[0];
    # pointRos.y = frame.p[1] - auxV1[1];
    # pointRos.z = frame.p[2] - auxV1[2];
    # marker.points.clear();
    # marker.points.push_back(pointRos);
    # pointRos.x = frame.p[0];
    # pointRos.y = frame.p[1];
    # pointRos.z = frame.p[2];
    # marker.pose.orientation.w = 1.0;
    # marker.points.push_back(pointRos);
    # marker.scale.x = 0.005;
    # marker.scale.y = 0.02;
    # marker.scale.z = 0.02;
    # marker.color.a = 1.0; // Don't forget to set the alpha!
    # marker.color.r = 0.0;
    # marker.color.g = 0.0;
    # marker.color.b = 1.0;
    # markerArray.markers.push_back(marker);

              
              self.jointsTrajectory = []
              found, self.jointsTrajectory = self.computeTrajectoryToJointsPosition(self.use_right_arm, self.reachingQ)

              if found:
                  self.numPointTrajectory = 0
                  # Lets plot the trajectory
                  print(len(self.jointsTrajectory))
                  nPoints = 0
                  if len(self.jointsTrajectory) < 100:
                      nPoints = 400
                  else:
                      nPoints = 1000 
                  self.smoothJointsTrajectoryTrunk, self.smoothJointsTrajectoryArm = self.computeSmoothJointsTrajectory(nPoints)
                  print("Smooth trajectory: ", len(self.smoothJointsTrajectoryTrunk), len(self.smoothJointsTrajectoryArm))

                  self.adjustedTrajectoryTrunk, self.adjustedTrajectoryArm = self.adjust_trajectory_to_velocity_limits(self.smoothJointsTrajectoryTrunk,
                                                                                                       self.smoothJointsTrajectoryArm, 
                                                                                                       self.max_velocities, self.period)
                  self.plotTrajectories(self.jointsTrajectory, self.smoothJointsTrajectoryTrunk,self.smoothJointsTrajectoryArm, 
                                              self.adjustedTrajectoryTrunk, self.adjustedTrajectoryArm, self.period)
                  self.firstInState = False
              else:
                  print("Solution NOT found")
                  self.state = -1
                  self.firstInState = True
          else:
              print("NOT feasible")
              self.state = -1
              self.firstInState = True
        else:
            print("Following the trajectory")
            if (self.followJointsTrajectory(self.use_right_arm, self.adjustedTrajectoryTrunk, self.adjustedTrajectoryArm)):
                self.state = 3
                self.firstInState = True


                
    
    def doState3(self):
        trunkCurrentQ = yarp.DVector(self.numTrunkJoints)
        if (not self.trunkIEncoders.getEncoders(trunkCurrentQ)):
            print("Unable to get trunk encoders position")
            raise SystemExit

        if self.use_right_arm:
            armCurrentQ = yarp.DVector(self.numRightArmJoints)
            if not self.rightArmIEncoders.getEncoders(armCurrentQ):
                print("Unable to get arm encoders position")
                raise SystemExit
        else:
            armCurrentQ = yarp.DVector(self.numLeftArmJoints)
            if not self.leftArmIEncoders.getEncoders(armCurrentQ):
                print("Unable to get arm encoders position")
                raise SystemExit

        numArmJoints = 0
        if self.use_right_arm:
            numArmJoints = self.numRightArmJoints
        else:
            numArmJoints = self.numLeftArmJoints

        current_Q = yarp.DVector(numArmJoints+self.numTrunkJoints)
        desire_Q = yarp.DVector(numArmJoints+self.numTrunkJoints)
        aproachingQs = []
        goal_q = []
                        
        for j in range(self.numTrunkJoints):
            current_Q[j] = trunkCurrentQ[j]
            goal_q.append(trunkCurrentQ[j])
        

        for j in range(0, numArmJoints):
            current_Q[j+2] = armCurrentQ[j]
            goal_q.append(armCurrentQ[j])
                      
        self.reachingPose = yarp.DVector(6)
        
        if self.use_right_arm:
            self.trunkRightArmICartesianSolver.fwdKin(current_Q, self.reachingPose)
        else:
            self.trunkLeftArmICartesianSolver.fwdKin(current_Q, self.reachingPose)
                  
        frame_reaching_base = self.vectorToFrame(self.reachingPose)
                        
        # First we need to get the encoders for the first postion
        aproachingQs.append(goal_q)
        self.d = 0
        while self.d <= 0.95*self.reachingDistance:
            print(self.d)
            self.d = self.d + self.reachingDistance/100.0

            frame_aux_reaching = PyKDL.Frame()
            frame_aux_reaching.p.z(self.d)

            frame_aux_base = frame_reaching_base*frame_aux_reaching

            aux_pose = self.frameToVector(frame_aux_base)
                            
            x_vector = yarp.DVector(6)
            for i in range(len(x_vector)):
                x_vector[i] = aux_pose[i]
              
                                
            if self.use_right_arm:
                if(self.trunkRightArmICartesianSolver.invKin(x_vector, current_Q, desire_Q)):
                    goal_q = []
                    for i in range(0,self.numTrunkJoints+self.numRightArmJoints):
                        goal_q.append(desire_Q[i])
                    aproachingQs.append(goal_q)
                    print("current_Q: ", current_Q)
                    print("desire_Q: ", current_Q)
                    print("distance: ", self.d)
                    current_Q = desire_Q
                    print("Aproaching Qs: ", aproachingQs)
            else:
                if(self.trunkLeftArmICartesianSolver.invKin(x_vector, current_Q, desire_Q)):
                    goal_q = []
                    for i in range(0,self.numTrunkJoints+self.numLeftArmJoints):
                        goal_q.append(desire_Q[i])
                    aproachingQs.append(goal_q)
                    print("current_Q: ", current_Q)
                    print("desire_Q: ", current_Q)
                    print("distance: ", self.d)
                    current_Q = desire_Q
                    print("Aproaching Qs: ", aproachingQs)
                            
        # Lets smooth the trajectory
        self.jointsTrajectory = aproachingQs
        self.smoothJointsTrajectoryTrunk, self.smoothJointsTrajectoryRightArm = self.computeSmoothJointsTrajectory(100)
        
        print("Smooth trajectory: ", len(self.smoothJointsTrajectoryTrunk), len(self.smoothJointsTrajectoryRightArm))
        self.adjustedTrajectoryTrunk, self.adjustedTrajectoryArm = self.adjust_trajectory_to_velocity_limits(self.smoothJointsTrajectoryTrunk,
                                                                                                       self.smoothJointsTrajectoryRightArm, 
                                                                                                       self.max_velocities, self.period)
        self.plotTrajectories(self.jointsTrajectory, self.smoothJointsTrajectoryTrunk,self.smoothJointsTrajectoryRightArm, 
                              self.adjustedTrajectoryTrunk, self.adjustedTrajectoryArm, self.period)
                        
        if (self.followJointsTrajectory(self.use_right_arm, self.adjustedTrajectoryTrunk, self.adjustedTrajectoryArm)):
            self.state = 4
            self.firstInState = True
                            
    def getMostRightObjectOfTheSameCategory(self, detections, asrData):
        x_min = 100
        final_category = None
        for i in range(detections.size()):
            print("Detection ",i)
            dictDetection = detections.get(i)
            category_strict = dictDetection.find("category").asString()
            x = dictDetection.find("X").asFloat32()
            if category_strict.find(asrData.toString()) != -1:
                if(x <x_min):
                    x_min = x
                    final_category = category_strict
        return final_category
    
            
    def updateModule(self):
       print("Demo sharon running")

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

    def computeTrajectoryToJointsPosition(self, rightArm, reachingJointsPosition):
        if rightArm:
            print("Selected right arm.")
        else:
            print("Selected left arm.")

        cmd = yarp.Bottle()
        cmd.addString('cpgj')
        goal = cmd.addList()

        for jointPosition in reachingJointsPosition:
            goal.addFloat32(jointPosition)
        print(cmd.toString())
        response = yarp.Bottle()
        jointsTrajectory = []
        if rightArm:
            self.rpcClientTrajectoryGenerationRight.write(cmd, response)
        else:
            self.rpcClientTrajectoryGenerationLeft.write(cmd, response)
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
                    jointsPosition.append(bJointsPosition.get(i).asFloat32())
                jointsTrajectory.append(jointsPosition)
            print("JointsTrajectory")
            return True, jointsTrajectory


    def computeTrajectoryToPose(self, rightArm, reachingPose):
        if rightArm:  # Compute the a feasible orientation for the rightArm
            print("Selected right arm.")
        else:
            print("Selected left arm.")

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
        if rightArm:
            self.rpcClientTrajectoryGenerationRight.write(cmd, response)
        else:
            self.rpcClientTrajectoryGenerationLeft.write(cmd, response)

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
                    jointsPosition.append(bJointsPosition.get(i).asFloat32())
                jointsTrajectory.append(jointsPosition)
            print("JointsTrajectory")
            return True, jointsTrajectory

    def computeFeasibleOrientation(self, rightArm, locationTCP, reachingDistance):
        if rightArm:  # Compute the a feasible orientation for the rightArm
            print("Selected right arm.")
        else:
            print("Selected left arm")

        # TODO: Compute orientation.
        frame_goal = PyKDL.Frame()
        print(frame_goal)
        frame_goal.M = frame_goal.M*PyKDL.Rotation.RotY(np.pi/2.0)
        frame_goal.M = frame_goal.M*PyKDL.Rotation.RotZ(-np.pi/2.0)

        for rotAroundAxisZ in np.linspace(-np.pi/8.0, np.pi/8.0, 10):
            for rotAroundAxisY in np.linspace(-np.pi/6.0, np.pi/6.0, 10):
                for rotAroundAxisX in np.linspace(-np.pi/2.0, np.pi/2.0, 30):
                    # rotAroundAxisX = 0
                    # rotAroundAxisY = 0
                    # print(rotAroundAxisX, rotAroundAxisY, rotAroundAxisZ)
                    aux_frame_goal = PyKDL.Frame()
                    aux_frame_goal.M = frame_goal.M * \
                        PyKDL.Rotation.RotX(rotAroundAxisX)
                    aux_frame_goal.M = aux_frame_goal.M * \
                        PyKDL.Rotation.RotY(rotAroundAxisY)
                    aux_frame_goal.M = aux_frame_goal.M * \
                        PyKDL.Rotation.RotZ(rotAroundAxisZ)

                    goal_vector = self.frameToVector(aux_frame_goal)
                    cmd = yarp.Bottle()
                    cmd.addVocab32('chgp')
                    # locationTCP[0] =  0.629151
                    # locationTCP[1] = -0.248893
                    # locationTCP[2] = 0.194516

                    goal = cmd.addList()
                    goal.addDouble(locationTCP[0])
                    goal.addDouble(locationTCP[1])
                    goal.addDouble(locationTCP[2])
                    goal.addDouble(goal_vector[3])
                    goal.addDouble(goal_vector[4])
                    goal.addDouble(goal_vector[5])
                    print("check goal: ", locationTCP[0], locationTCP[1],
                          locationTCP[2], goal_vector[3], goal_vector[4], goal_vector[5])

                    orientationTCP = [goal_vector[3],
                        goal_vector[4], goal_vector[5]]
                    response = yarp.Bottle()
                    jointsTrajectory = []

                    if rightArm:
                        self.rpcClientTrajectoryGenerationRight.write(
                            cmd, response)
                    else:
                        self.rpcClientTrajectoryGenerationLeft.write(
                            cmd, response)
                        reaching_pose = []

                    if response.get(0).asVocab32() == VOCAB_OK:
                        print("Valid final pose")
                        x = [locationTCP[0], locationTCP[1], locationTCP[2],
                            orientationTCP[0], orientationTCP[1], orientationTCP[2]]
                        frame_end_base = self.vectorToFrame(x)

                        frame_reaching_end = PyKDL.Frame()
                        frame_reaching_end.p.z(-reachingDistance)

                        frame_reaching_base = frame_end_base*frame_reaching_end

                        reaching_pose = self.frameToVector(frame_reaching_base)

                        cmd.clear()
                        cmd.addVocab32('cbgp')
                        goal = cmd.addList()
                        for i in range(len(reaching_pose)):
                            goal.addDouble(reaching_pose[i])
                        response.clear()
                        if rightArm:
                            self.rpcClientTrajectoryGenerationRight.write(
                                cmd, response)
                        else:
                            self.rpcClientTrajectoryGenerationLeft.write(
                                cmd, response)

                        if response.get(0).asVocab32() == VOCAB_OK:
                            print("Valid reaching position")
                            return True, orientationTCP, reaching_pose

                        else:
                            print("Not valid reaching position")
        orientationTCP = []
        reaching_pose = []
        return False, orientationTCP, reaching_pose

    def checkJointsPosition(self, desiredJointsPosition, iEncoders, numJoints):
        currentQ = yarp.DVector(numJoints)
        iEncoders.getEncoders(currentQ)
        print(list(currentQ))
        for joint in range(numJoints):
            # print(currentQ[joint])
            if abs(currentQ[joint]-desiredJointsPosition[joint]) > self.jointsPositionError:
                return False
        return True

    def transformFromXYZFromCameraCoordinatesToTrunk(self, objects_data, category_object_to_grasp):
        # we received the detected objects data

        trunkCurrentQ = yarp.DVector(self.numTrunkJoints)
        if available_trunk_controlboard:
            self.trunkIEncoders.getEncoders(trunkCurrentQ)

        headCurrentQ = yarp.DVector(self.numHeadJoints)

        if available_head_controlboard:
            self.headIEncoders.getEncoders(headCurrentQ)
        else:
            headCurrentQ[0] = self.initHeadJointsPosition[0]
            headCurrentQ[1] = self.initHeadJointsPosition[1]

        current_Q = yarp.DVector(self.numTrunkJoints+self.numHeadJoints)
        for j in range(0, self.numHeadJoints):
            current_Q[j+2] = headCurrentQ[j]
        for j in range(self.numTrunkJoints):
            current_Q[j] = trunkCurrentQ[j]

        print(current_Q[0], current_Q[1], current_Q[2], current_Q[3])

        x_head_trunk = yarp.DVector(6)
        self.trunkHeadICartesianSolver.fwdKin(current_Q, x_head_trunk)

        print(x_head_trunk[0], x_head_trunk[1], x_head_trunk[2],
              x_head_trunk[3], x_head_trunk[4], x_head_trunk[5])
        # TODO: transform to kdl frame head_wrt_trunk and object xyz -> object_xyz_wrt_trunk

        frame_head_trunk = self.vectorToFrame(x_head_trunk)

        o = 0
        for o in range(objects_data.size()):
            if category_object_to_grasp == objects_data.get(o).find("category").asString():
                break

        if category_object_to_grasp != objects_data.get(o).find("category").asString():
            print("Object of category ",
                  category_object_to_grasp, "is NOT detected")
        else:
            print("Object of category ", category_object_to_grasp, "is detected")

        if robot == "/teo":
            frame_camera_head = PyKDL.Frame()
            print(frame_camera_head)
            frame_camera_head = frame_camera_head * \
                PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(0, 0, 0.059742))
            frame_camera_head = frame_camera_head * \
                PyKDL.Frame(PyKDL.Rotation.RotZ(-np.pi/2.0),
                            PyKDL.Vector(0, 0, 0))
            frame_camera_head = frame_camera_head * \
                PyKDL.Frame(PyKDL.Rotation.RotX(-np.pi/2.0),
                            PyKDL.Vector(0, 0, 0))
            frame_camera_head = frame_camera_head * \
                PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(-0.018-0.026, 0, 0))
            print(frame_camera_head)

            frame_object_camera = PyKDL.Frame()

            frame_object_camera.p.x(objects_data.get(o).find("mmX").asFloat32())
            frame_object_camera.p.y(objects_data.get(o).find("mmY").asFloat32())
            frame_object_camera.p.z(objects_data.get(o).find("mmZ").asFloat32())

            frame_object_head = frame_camera_head*frame_object_camera
            frame_object_trunk = frame_head_trunk*frame_object_head

            pose_object_trunk = self.frameToVector(frame_object_trunk)
        else:
            frame_object_camera = PyKDL.Frame()

            frame_object_camera.p.x(objects_data.get(o).find("mmZ").asFloat32())
            frame_object_camera.p.y(objects_data.get(o).find("mmX").asFloat32())
            frame_object_camera.p.z(-objects_data.get(o).find("mmY").asFloat32())

            frame_object_trunk = frame_head_trunk*frame_object_camera

            pose_object_trunk = self.frameToVector(frame_object_trunk)

        # print(pose_object_camera[0], pose_object_camera[1], pose_object_camera[2])

        return [pose_object_trunk[0], pose_object_trunk[1], pose_object_trunk[2]]

    def followJointsTrajectory(self, rightArm, jointsTrajectoryTrunk, jointsTrajectoryArm):
        if rightArm:
            print("followJointsTrajectory ", self.numPointTrajectory,
                len(jointsTrajectoryTrunk))
            self.numPointTrajectory = 0
            period = 50
            if not self.trunkIControlMode.setControlModes(yarp.IVector(2, yarp.encode('posd'))):
                print("Unable to set trunk to position direct mode.")
                raise SystemExit
            else:
                print("Trunk set to position direct mode.")
            
            
            if not self.rightArmIControlMode.setControlModes(yarp.IVector(6, yarp.encode('posd'))):
                print("Unable to set right arm  to position direct mode.")
                raise SystemExit
            else:
                print("Right arm set to position direct mode.") 
                            
            start = time.time()

            while self.numPointTrajectory < len(jointsTrajectoryTrunk)-1:
                print(self.numPointTrajectory)
                for j in range(2):
                    self.trunkIPositionDirect.setPosition(j, jointsTrajectoryTrunk[self.numPointTrajectory][j])
                for j in range(6):
                    self.rightArmIPositionDirect.setPosition(j, jointsTrajectoryArm[self.numPointTrajectory][j])
                time.sleep(period * 0.001 - ((time.time() - start) % (period * 0.001)))
                self.numPointTrajectory+=1
            return True
        else:
            print("followJointsTrajectory ", self.numPointTrajectory,len(jointsTrajectoryTrunk))
            self.numPointTrajectory = 0
            period = 50

            if not self.trunkIControlMode.setControlModes(yarp.IVector(2, yarp.encode('posd'))):
                print("Unable to set trunk to position direct mode.")
                raise SystemExit
            else:
                print("Trunk set to position direct mode.")
            
        
            leftArmModes = yarp.IVector(6,yarp.VOCAB_CM_POSITION_DIRECT)
            if not self.leftArmIControlMode.setControlModes(yarp.IVector(6, yarp.encode('posd'))):
                print("Unable to set left arm  to position direct mode.")
                raise SystemExit
            else:
                print("Left arm set to position direct mode.") 
                            
            start = time.time()

            while self.numPointTrajectory < len(jointsTrajectoryTrunk)-1:
                print(self.numPointTrajectory)
                for j in range(2):
                    self.trunkIPositionDirect.setPosition(j, jointsTrajectoryTrunk[self.numPointTrajectory][j])
                for j in range(6):
                    self.leftArmIPositionDirect.setPosition(j, jointsTrajectoryArm[self.numPointTrajectory][j])
                time.sleep(period * 0.001 - ((time.time() - start) % (period * 0.001)))
                self.numPointTrajectory+=1
            return True


    def on_release(self, key):
        if key == keyboard.Key.enter:
            print("enter is pressed")
            if self.state == 7:
                self.enter_is_pressed = True
            if self.state == 3:
                self.enter_is_pressed = True
        elif key == keyboard.Key.esc:
            return False

    def computeSmoothJointsTrajectory(self, nPoints):
        smoothJointsTrajectoryTrunk = []
        smoothJointsTrajectoryRightArm = []
        arr = np.asarray(self.jointsTrajectory)
        t = range(0, len(self.jointsTrajectory))
        s = 0.5
        tck, u = interpolate.splprep([t, arr[:, 0]], s=s)
        print(u[-1])

        # create interpolated lists of points
        tnew, q0new = interpolate.splev(np.linspace(0, 1.0, nPoints), tck)

        tck, u = interpolate.splprep([t, arr[:, 1]], s=s)
        # create interpolated lists of points
        tnew, q1new = interpolate.splev(np.linspace(0, 1.0, nPoints), tck)

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
        
        print(tnew)

        for i in range(0, len(q7new)):
            trunkJointsPosition = [q0new[i], q1new[i]]
            jointsPosition = [q2new[i],
                q3new[i], q4new[i], q5new[i], q6new[i], q7new[i]]
            smoothJointsTrajectoryTrunk.append(trunkJointsPosition)
            smoothJointsTrajectoryRightArm.append(jointsPosition)
        return smoothJointsTrajectoryTrunk, smoothJointsTrajectoryRightArm
      
      
    def adjust_joint_trajectory(self, trajectory, max_changes):
      adjusted_trajectory = [trajectory[0]]  # Start with the initial positions
      n_joints = len(trajectory[0])

      for point_index in range(1, len(trajectory)):
          current_point = trajectory[point_index]
          previous_point = adjusted_trajectory[-1]
          need_adjustment = False
          # Determine if any joint exceeds the max change and by how much
          exceed_factors = []
          for joint_index in range(n_joints):
              change = current_point[joint_index] - previous_point[joint_index]
              exceed_factor = abs(change) / max_changes[joint_index]
              exceed_factors.append(exceed_factor)
              if exceed_factor > 1:
                  need_adjustment = True
          
          if need_adjustment:
              max_exceed_factor = max(exceed_factors)
              # Calculate the number of intermediate points needed
              additional_points = int(np.ceil(max_exceed_factor)) - 1
              for i in range(1, additional_points + 2):  # +2 to include the end point itself
                  interpolated_point = [
                      previous_point[joint_index] + (change / max_exceed_factor) * i
                      for joint_index, change in enumerate([current_point[j] - previous_point[j] for j in range(n_joints)])
                  ]
                  adjusted_trajectory.append(interpolated_point)
          else:
              adjusted_trajectory.append(current_point)

      return adjusted_trajectory


    def adjust_trajectory_to_velocity_limits(self,joints_trajectory_trunk, joints_trajectory_arm, max_velocities, time_interval):
        """
        Adjusts the provided trajectory to ensure that velocity limits are not exceeded.
        
        :param joints_trajectory: A list of trajectories for each joint. Each trajectory is a list of positions.
        :param max_velocities: Maximum allowed velocities for each joint in degrees/second.
        :param time_interval: Time interval between successive trajectory points in seconds.
        :return: Adjusted trajectory as a list of lists, each corresponding to a joint's trajectory.
        """
        
        n_joints_trunk = len(joints_trajectory_trunk[0])
        n_joints_arm = len(joints_trajectory_arm[1])
        n_points = len(joints_trajectory_trunk)
        
        print("n_joints_trunk: ", n_joints_trunk)
        print("n_joints_arm: ", n_joints_arm)
        print("n_points: ", n_points)

        max_changes_trunk = [v * time_interval for v in max_velocities[:n_joints_trunk]]
        max_changes_arm = [v * time_interval for v in max_velocities[n_joints_trunk:]]        


        adjusted_trajectory_trunk = self.adjust_joint_trajectory(joints_trajectory_trunk, max_changes_trunk)
        adjusted_trajectory_arm = self.adjust_joint_trajectory(joints_trajectory_arm, max_changes_arm)

        return adjusted_trajectory_trunk, adjusted_trajectory_arm


    def plotTrajectories(self, jointsTrajectory, smoothJointsTrajectoryTrunk, smoothJointsTrajectoryRightArm, adjusted_trajectory_trunk, adjusted_trajectory_arm, period):
        arr = np.asarray(jointsTrajectory)
        t = np.linspace(0,1, len(jointsTrajectory))

        arr_smooth_trunk = np.asarray(smoothJointsTrajectoryTrunk)
        t_smooth_trunk = np.linspace(0,1, len(smoothJointsTrajectoryTrunk))


        arr_smooth_rightArm = np.asarray(smoothJointsTrajectoryRightArm)
        t_smooth_rightArm =  np.linspace(0,1, len(smoothJointsTrajectoryRightArm))

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
            # ax[i].set_xlim(0, 1)
            # ax[i].yaxis.set_major_locator(MaxNLocator(5))
        ax[0].plot(t, arr[:, 0], label='q0', color='skyblue')
        ax[0].plot(t_smooth_trunk, arr_smooth_trunk[:, 0], label='q0 smooth',
                   color='red', linestyle='dashed')

        ax[1].plot(t, arr[:, 1], label='q1')
        ax[1].plot(t_smooth_trunk, arr_smooth_trunk[:, 1], label='q1 smooth',
                   color='red', linestyle='dashed')

        ax[2].plot(t, arr[:, 2], label='q2')
        ax[2].plot(t_smooth_rightArm, arr_smooth_rightArm[:, 0], label='q2 smooth',
                   color='red', linestyle='dashed')

        ax[3].plot(t, arr[:, 3], label='q3')
        ax[3].plot(t_smooth_rightArm, arr_smooth_rightArm[:, 1], label='q3 smooth',
                   color='red', linestyle='dashed')

        ax[4].plot(t, arr[:, 4], label='q4')
        ax[4].plot(t_smooth_rightArm, arr_smooth_rightArm[:, 2], label='q4 smooth',
                   color='red', linestyle='dashed')

        ax[5].plot(t, arr[:, 5], label='q5')
        ax[5].plot(t_smooth_rightArm, arr_smooth_rightArm[:, 3], label='q5 smooth',
                   color='red', linestyle='dashed')

        ax[6].plot(t, arr[:, 6], label='q6')
        ax[6].plot(t_smooth_rightArm, arr_smooth_rightArm[:, 4], label='q6 smooth',
                   color='red', linestyle='dashed')

        ax[7].plot(t, arr[:, 7], label='Joint trajectory')
        ax[7].plot(t_smooth_rightArm, arr_smooth_rightArm[:, 5],
                   label='Smoothed joint trajectory', color='red', linestyle='dashed')
        
        # ax[7].xaxis.set_ticks(np.linspace(0, 1.0, 100))

        handles, labels = ax[7].get_legend_handles_labels()
        fig.legend(handles, labels, loc='upper right')

        plt.show()
        plt.figure().clear()
        
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

        fig.suptitle("Joints trajectories limited by velocity.")
        for i in range(8):
            ax[i].grid()

        arr_smooth_trunk = np.asarray(smoothJointsTrajectoryTrunk)
        t_smooth_trunk = np.linspace(0,len(smoothJointsTrajectoryTrunk)*period, len(smoothJointsTrajectoryTrunk))

        arr_smooth_rightArm = np.asarray(smoothJointsTrajectoryRightArm)
        t_smooth_rightArm =  np.linspace(0, len(smoothJointsTrajectoryRightArm)*period, len(smoothJointsTrajectoryRightArm))
        
        arr_adjusted_trunk = np.asarray(adjusted_trajectory_trunk)
        t_adjusted_trunk = np.linspace(0,len(adjusted_trajectory_trunk)*period, len(adjusted_trajectory_trunk))
        
        arr_adjusted_arm = np.asarray(adjusted_trajectory_arm)
        t_adjusted_arm = np.linspace(0, len(adjusted_trajectory_arm)*period, len(adjusted_trajectory_arm))
        

        ax[0].plot(t_smooth_trunk, arr_smooth_trunk[:, 0], label='q0 smooth',
                   color='red', linestyle='dashed')
        ax[0].plot(t_adjusted_trunk, arr_adjusted_trunk[:, 0], label='q0 limited by velocity',
                    color='green')

        ax[1].plot(t_smooth_trunk, arr_smooth_trunk[:, 1], label='q1 smooth',
                   color='red', linestyle='dashed')
        ax[1].plot(t_adjusted_trunk, arr_adjusted_trunk[:, 1], label='q1 limited by velocity',
                    color='green')

        ax[2].plot(t_smooth_rightArm, arr_smooth_rightArm[:, 0], label='q2 smooth',
                   color='red', linestyle='dashed')
        
        ax[2].plot(t_adjusted_arm, arr_adjusted_arm[:, 0], label='q2 limited by velocity',
                    color='green')

        ax[3].plot(t_smooth_rightArm, arr_smooth_rightArm[:, 1], label='q3 smooth',
                   color='red', linestyle='dashed')
        ax[3].plot(t_adjusted_arm, arr_adjusted_arm[:, 1], label='q3 limited by velocity',
                    color='green')

        ax[4].plot(t_smooth_rightArm, arr_smooth_rightArm[:, 2], label='q4 smooth',
                   color='red', linestyle='dashed')
        ax[4].plot(t_adjusted_arm, arr_adjusted_arm[:, 2], label='q4 limited by velocity',
                    color='green')

        ax[5].plot(t_smooth_rightArm, arr_smooth_rightArm[:, 3], label='q5 smooth',
                   color='red', linestyle='dashed')
        ax[5].plot(t_adjusted_arm, arr_adjusted_arm[:, 3], label='q5 limited by velocity',
                    color='green')

        ax[6].plot(t_smooth_rightArm, arr_smooth_rightArm[:, 4], label='q6 smooth',
                   color='red', linestyle='dashed')
        ax[6].plot(t_adjusted_arm, arr_adjusted_arm[:, 4], label='q6 limited by velocity',
                    color='green')

        ax[7].plot(t_smooth_rightArm, arr_smooth_rightArm[:, 5],
                   label='q7 smooth', color='red', linestyle='dashed')
        ax[7].plot(t_adjusted_arm, arr_adjusted_arm[:, 5],
                    label='q7 limited by velocity', color='green')
        
        # ax[7].xaxis.set_ticks(np.linspace(0, 1.0, 100))

        handles, labels = ax[7].get_legend_handles_labels()
        fig.legend(handles, labels, loc='upper right')
        plt.show()
        plt.figure().clear()
        plt.close()
        
        


    def getFeasibleGraspingPose(self, rightArm, bGraspingPoses, reachingDistance):
        graspingPose = []
        reaching_pose = []
        grasping_q = []
        reaching_q = []
        print(bGraspingPoses.size())
        numArmJoints = 0
        if rightArm:  # Compute the a feasible orientation for the rightArm
            print("Selected right arm.")
            numArmJoints    = self.numRightArmJoints
        else:
            print("Selected left arm")
            numArmJoints    = self.numLeftArmJoints
        for j in range(1, bGraspingPoses.size()):
            bGraspingPose = bGraspingPoses.get(j).asList()
            graspingPose = []
            for i in range(bGraspingPose.size()):
                graspingPose.append(bGraspingPose.get(i).asFloat32())
            print(graspingPose)

            cmd = yarp.Bottle()
            response = yarp.Bottle()
            cmd.addVocab32('chgp')
            goal = cmd.addList()
            for goal_index in range(6):
                goal.addFloat32(graspingPose[goal_index])
            if rightArm:
                self.rpcClientTrajectoryGenerationRight.write(cmd, response)
            else:
                self.rpcClientTrajectoryGenerationLeft.write(cmd, response)

            if response.get(0).asVocab32() == VOCAB_OK:
                print("Valid final pose")
                print(response.get(1).toString())
                aux_q = response.get(1).asList()
                
                for i in range(self.numTrunkJoints+numArmJoints):
                    grasping_q.append(aux_q.get(i).asFloat32())
                print(grasping_q)

                # if goal_q.get(6).asFloat32() < 0:
                #     return False, graspingPose, reaching_pose
                
                frame_end_base = self.vectorToFrame(graspingPose)

                frame_reaching_end = PyKDL.Frame()
                frame_reaching_end.p.z(-reachingDistance)

                frame_reaching_base = frame_end_base*frame_reaching_end

                reaching_pose = self.frameToVector(frame_reaching_base)
                
                print("Let's check the reaching pose")
                cmd.clear()
                cmd.addVocab32("chgp")
                goal = cmd.addList()
                for i in range(len(reaching_pose)):
                    goal.addFloat32(reaching_pose[i])
                    
                response.clear()
                if rightArm:
                    self.rpcClientTrajectoryGenerationRight.write(cmd, response)
                else:
                    self.rpcClientTrajectoryGenerationLeft.write(cmd, response)

                if response.get(0).asVocab32() == VOCAB_OK:
                    print("Valid reaching position")
                    aux_q = response.get(1).asList()
                    for i in range(self.numTrunkJoints+numArmJoints):
                        reaching_q.append(aux_q.get(i).asFloat32())
                   
                    return True, graspingPose,grasping_q, reaching_pose, reaching_q
                else:
                    print("Not valid reaching position")
            else:
                print("Not valid final pose")
        
        return False, graspingPose,grasping_q, reaching_pose, reaching_q

    def say(self, key):
        text = self.dictonarySpeak[key]
        cmd = yarp.Bottle()
        response = yarp.Bottle()
        cmd.addString("say")
        cmd.addString(text)
        self.rpcClientTts.write(cmd, response)
        print(response.toString())
        
    def moveJointsInsideBounds(self):
        if not self.trunkIControlMode.setControlModes(yarp.IVector(2, yarp.encode('posd'))):
            print("Unable to set trunk to position mode.")
        else:
            print("Trunk set to position mode.")
        
     
        if not self.rightArmIControlMode.setControlModes(yarp.IVector(6, yarp.encode('posd'))):
            print("Unable to set right arm  to position  mode.")
        else:
            print("Right arm set to position  mode.")
            
        trunkCurrentQ = yarp.DVector(self.numTrunkJoints)
        self.trunkIEncoders.getEncoders(trunkCurrentQ)

        armCurrentQ = yarp.DVector(self.numRightArmJoints)
        self.rightArmIEncoders.getEncoders(armCurrentQ)

        current_Q = yarp.DVector(self.numRightArmJoints+self.numTrunkJoints)

        for j in range(0, self.numRightArmJoints):
            current_Q[j+2] = armCurrentQ[j]
        for j in range(self.numTrunkJoints):
            current_Q[j] = trunkCurrentQ[j]
        
        for j, q in enumerate(current_Q):
            print(self.minsTrunkAndRightArm.get(j).asFloat32(),self.maxsTrunkAndRightArm.get(j).asFloat32())
            if j<2:
                if q < self.minsTrunkAndRightArm.get(j).asFloat32():
                    self.trunkIPositionControl.positionMove(j, self.minsTrunkAndRightArm.get(j).asFloat32()+2.0)
                    print(q,"joint", j, "outside min bounds")
                if q > self.maxsTrunkAndRightArm.get(j).asFloat32():
                    print(q,"joint", j, "outside max bounds")
                    self.trunkIPositionControl.positionMove(j, self.maxsTrunkAndRightArm.get(j).asFloat32()-2.0)
                    yarp.delay(3)
            else:
                if q<self.minsTrunkAndRightArm.get(j).asFloat32():
                    self.rightArmIPositionControl.positionMove(j,self.minsTrunkAndRightArm.get(j).asFloat32()+2.0)
                if q>self.maxsTrunkAndRightArm.get(j).asFloat32():
                    self.rightArmIPositionControl.positionMove(j,self.maxsTrunkAndRightArm.get(j).asFloat32()-2.0)

                     
   

if __name__ == "__main__":
    print("main")
    yarp.Network.init()  # connect to YARP network

    if not yarp.Network.checkNetwork():  # let's see if there was actually a reachable YARP network
        print('yarp network is not found!!')
        sys.exit(1)

    demo_sharon = DemoSharon()

    rf = yarp.ResourceFinder()
    rf.setVerbose(True)
    rf.setDefaultContext('demoSharon')
    rf.setDefaultConfigFile('demoSharon.ini')

    rf.configure(sys.argv)
    demo_sharon.runModule(rf)

    sys.exit()


