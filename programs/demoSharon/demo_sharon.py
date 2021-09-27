import yarp
import sys
import PyKDL
import numpy as np
import kinematics_dynamics
from pynput import keyboard



import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate
from matplotlib.ticker import MaxNLocator

robot = '/teoSim'

available_right_hand_controlboard = True
available_left_hand_controlboard = True
available_right_arm_controlboard = True
available_left_arm_controlboard = True
available_head_controlboard = True
available_trunk_controlboard = True


class DemoSharon(yarp.RFModule):
    def __init__(self):
        yarp.RFModule.__init__(self)
        # Right Arm device
        self.rightArmOptions = yarp.Property()
        self.rightArmDevice = None
        self.rightArmIEncoders = None
        self.numRightArmJoints = 0
        self.rightArmIPositionControl = None
        
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
        
        # Head device
        self.headOptions = yarp.Property()
        self.headDevice = None
        self.headIEncoders = None
        self.numHeadJoints = 2
        self.headIPositionControl = None
        self.trunkHeadIControlLimits = None

        # Trajectory Generation client
        self.rpcClientTrajectoryGenerationRight = yarp.RpcClient()
        self.rpcClientTrajectoryGenerationLeft = yarp.RpcClient()

        # # Underactuated Hand rpc client
        # self.rpcClientRightHand = yarp.RpcClient()
        # self.rpcClientLeftHand = yarp.RpcClient()

        # State
        self.state = 0
        self.firstInState = True  # For printing messages just once
        self.jointsPositionError = 2.0


        # Init Joints position for grasping
        self.initTrunkJointsPosition = [0, 14.0]
        self.initBackTrunkJointsPosition = [0, -9.0]
        self.initHeadJointsPosition = [0, 14.0]

        # [-90, 0, 0, 0, 90, 0]
        self.initRightArmJointsPosition = [-50, -50, 40, -70, 30, -20]
        self.initLeftArmJointsPosition = [-50, 50, -40, -70, -30, -20]

        # Object category and position port
        self.objectPositionPort = yarp.BufferedPortBottle()
        self.objectData = yarp.Bottle()

        self.category = None
        self.locationTCP = []
        self.rightArm = True
        self.orientationTCP = []
        self.jointsTrajectory = []
        self.aux = []
        self.numPointTrajectory = 0
        self.d = 0
        self.reachingDistance = 0

        # tcp frame wrt trunk when graspping
        self.frame_tcp_trunk = None
        self.up_distance = 0.15
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
        
        self.rightArmJointsInPosition = [False, False, False, False, False, False]
        self.leftArmJointsInPosition = [False, False, False, False, False, False]
        
        self.enter_is_pressed = False
        
        # For plotting the path
        
        self.smoothJointsTrajectory = []

        self.matching_object_output_port_name = "/matching/object:o"
        self.demo_object_input_port_name = "/demoSharon/object:i"
        
        self.once = True


    def configure(self, rf):
        print("Configuring...")

        self.objectPositionPort.open(self.demo_object_input_port_name)
        yarp.Network.connect(self.matching_object_output_port_name, self.demo_object_input_port_name)

        # Open rightArm device
        if available_right_arm_controlboard:
            self.rightArmOptions.put('device', 'remote_controlboard')
            self.rightArmOptions.put('remote', robot+'/rightArm')
            self.rightArmOptions.put('local', robot+'/rightArm')

            self.rightArmDevice = yarp.PolyDriver(self.rightArmOptions)
            self.rightArmDevice.open(self.rightArmOptions)
            if not self.rightArmDevice.isValid():
                print('Cannot open rightArm device!')
                raise SystemExit
        
        # Open rightHand device
        self.rightHandOptions.put('device', 'remote_controlboard')
        self.rightHandOptions.put('remote', robot+'/rightHand')
        self.rightHandOptions.put('local', robot+'/rightHand')
        
        self.rightHandDevice = yarp.PolyDriver(self.rightHandOptions)
        self.rightHandDevice.open(self.rightHandOptions)
        if available_right_hand_controlboard:
            if not self.rightHandDevice.isValid():
                print('Cannot open rightHand device!')
                raise SystemExit
            
        # Open leftArm device
        
        if available_left_arm_controlboard:
            self.leftArmOptions.put('device', 'remote_controlboard')
            self.leftArmOptions.put('remote', robot+'/leftArm')
            self.leftArmOptions.put('local', robot+'/leftArm')
            self.leftArmDevice = yarp.PolyDriver(self.leftArmOptions)
            self.leftArmDevice.open(self.leftArmOptions)
            if not self.leftArmDevice.isValid():
                print('Cannot open leftArm device!')
                raise SystemExit
        
        # Open leftHand device
        self.leftHandOptions.put('device', 'remote_controlboard')
        self.leftHandOptions.put('remote', robot+'/leftHand')
        self.leftHandOptions.put('local', robot+'/leftHand')
        
        self.leftHandDevice = yarp.PolyDriver(self.leftHandOptions)
        self.leftHandDevice.open(self.leftHandOptions)
        if available_left_hand_controlboard:
            if not self.leftHandDevice.isValid():
                print('Cannot open leftHand device!')
                raise SystemExit


        # Open trunk device
        self.trunkOptions.put('device', 'remote_controlboard')
        self.trunkOptions.put('remote', robot+'/trunk')
        self.trunkOptions.put('local', robot+'/trunk')

        self.trunkDevice = yarp.PolyDriver(self.trunkOptions)
        self.trunkDevice.open(self.trunkOptions)
        if available_trunk_controlboard:
            if not self.trunkDevice.isValid():
                print('Cannot open trunk device!')
                raise SystemExit

        # Open head device
        self.headOptions.put('device', 'remote_controlboard')
        self.headOptions.put('remote', robot+'/head')
        self.headOptions.put('local', robot+'/head')

        self.headDevice = yarp.PolyDriver(self.headOptions)
        self.headDevice.open(self.headOptions)
        if available_head_controlboard:
            if not self.headDevice.isValid():
                print('Cannot open head device!')
                raise SystemExit

        # Interface with right arm encoders
        if available_right_arm_controlboard:
            self.rightArmIEncoders = self.rightArmDevice.viewIEncoders()

            if self.rightArmIEncoders == []:
                print("Right arm Encoder interface NOT available.")
                raise SystemExit
            else:
                print("Right arm Encoder interface available.")

            self.numRightArmJoints = self.rightArmIEncoders.getAxes()

        # # Interface with the emulated right Hand encoders
        
        # if available_right_hand_controlboard:        
        #     self.rightHandIEncoders = self.rightHandDevice.viewIEncoders()
        #     if self.rightHandIEncoders == []:
        #         print("Right Hand Encoder interface NOT available.")
        #         raise SystemExit
        #     else:
        #         print("Right Hand Encoder interface available.")
        #         self.numRightHandJoints = self.rightHandIEncoders.getAxes()

        
        # Right arm position control interface
        if available_right_arm_controlboard:
            self.rightArmIPositionControl = self.rightArmDevice.viewIPositionControl()

            if self.rightArmIPositionControl == []:
                print("Right arm position control interface NOT available")
                raise SystemExit
            else:
                print("Right arm position control interface available.")

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
        # Interface with left arm encoders
        if available_left_arm_controlboard:
            self.leftArmIEncoders = self.leftArmDevice.viewIEncoders()

            if self.leftArmIEncoders == []:
                print("Left arm Encoder interface NOT available.")
                raise SystemExit
            else:
                print("Left arm Encoder interface available.")

            self.numLeftArmJoints = self.leftArmIEncoders.getAxes()

            # Left arm position control interface
            self.leftArmIPositionControl = self.leftArmDevice.viewIPositionControl()

            if self.leftArmIPositionControl == []:
                print("Left arm position control interface NOT available")
                raise SystemExit
            else:
                print("Left arm position control interface available.")

        # Left hand position control interface
        if available_left_hand_controlboard:
            self.leftHandIPositionControl = self.leftHandDevice.viewIPositionControl()

            if self.leftHandIPositionControl == []:
                print("Left hand position control interface NOT available")
                raise SystemExit
            else:
                print("Left hand position control interface available.")

        # Interface with trunk encoders
        if available_trunk_controlboard:
            self.trunkIEncoders = self.trunkDevice.viewIEncoders()

            if self.trunkIEncoders == []:
                print("Trunk Encoder interface NOT available.")
                raise SystemExit
            else:
                print("Trunk Encoder interface available.")

            self.numTrunkJoints = self.trunkIEncoders.getAxes()

            # Trunk position control interface
            self.trunkIPositionControl = self.trunkDevice.viewIPositionControl()

            if self.trunkIPositionControl == []:
                print("Trunk position control interface NOT available")
                raise SystemExit
            else:
                print("Trunk position control interface available.")

        # trunk and right arm solver device
        trunkRightKinPath = rf.findFileByName("teo-trunk-rightArm-fetch.ini")
        self.trunkRightArmSolverOptions.fromConfigFile(trunkRightKinPath)
        self.trunkRightArmSolverOptions.put("device", "KdlSolver")
        self.trunkRightArmSolverOptions.put("ik", "nrjl")

        self.trunkRightArmSolverOptions.fromString("(mins (-40 -10.0 -98.1 -75.5 -80.1 -99.6 -80.4 -115.4))", False)
        self.trunkRightArmSolverOptions.fromString("(maxs (40 20.0 106 22.4 57 98.4 99.6 44.7))", False)
        print("mins")
        print(self.trunkRightArmSolverOptions.find("mins").toString())
        print("maxs")
        print(self.trunkRightArmSolverOptions.find("maxs").toString())
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

        trunkLeftKinPath = rf.findFileByName("teo-trunk-leftArm-fetch.ini")
        self.trunkLeftArmSolverOptions.fromConfigFile(trunkLeftKinPath)
        self.trunkLeftArmSolverOptions.put("device", "KdlSolver")
        self.trunkLeftArmSolverOptions.put("ik", "nrjl")

        self.trunkLeftArmSolverOptions.fromString(
            "(mins (-40 -10.1 -98.1 -75.5 -80.1 -99.6 -80.4 -115.4))", False)
        self.trunkLeftArmSolverOptions.fromString(
            "(maxs (40 20.1 106 22.4 57 98.4 99.6 44.7))", False)
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
        
        # trunk and head solver device
        trunkHeadKinPath = rf.findFileByName("teo-trunk-head.ini")
        self.trunkHeadSolverOptions.fromConfigFile(trunkHeadKinPath)
        self.trunkHeadSolverOptions.put("device", "KdlSolver")
        self.trunkHeadSolverOptions.put("ik", "nrjl")
        self.trunkHeadSolverOptions.fromString(
            "(mins (-20 -10.8 -70 -29))", False)
        self.trunkHeadSolverOptions.fromString(
            "(maxs (20 10.1 70 8.4))", False)
        self.trunkHeadSolverDevice = yarp.PolyDriver(
            self.trunkHeadSolverOptions)  # calls open -> connects
        self.trunkHeadSolverDevice.open(self.trunkHeadSolverOptions)
        
        if self.trunkHeadSolverDevice == []:
            print("trunk head  solver device interface NOT available")
        else:
            print("trunk head solver device interface available.")
        
        # Trunk and head cartesian solver
        self.trunkHeadICartesianSolver = kinematics_dynamics.viewICartesianSolver(
            self.trunkHeadSolverDevice)
        if self.trunkHeadICartesianSolver == []:
            print("Trunk head cartesian solver interface NOT available")
        else:
            print("Trunk head cartesian solver interface available.")


        # Open rpc client for /trajectoryGeneration/trunkAndRightArm
        self.rpcClientTrajectoryGenerationRight.open('/trajectoryGeneration/trunkAndRightArm/rpc:c')
        yarp.Network.connect("/trajectoryGeneration/trunkAndRightArm/rpc:c",
                             "/trajectoryGeneration/trunkAndRightArm/rpc:s")
        
        # Open rpc client for /trajectoryGeneration/trunkAndLeftArm
        self.rpcClientTrajectoryGenerationLeft.open('/trajectoryGeneration/trunkAndLeftArm/rpc:c')
        yarp.Network.connect("/trajectoryGeneration/trunkAndLeftArm/rpc:c",
                             "/trajectoryGeneration/trunkAndLeftArm/rpc:s")

        # Open rpc client for rightHand        
        
        # self.rpcClientRightHand.open(robot+'/rightHand/rpc:o')
        # yarp.Network.connect(robot+'/rightHand/rpc:o', robot+'/rightHand/rpc:i')

        # # Open rpc client for leftHand        
        # self.rpcClientLeftHand.open(robot+'/leftHand/rpc:o')
        # yarp.Network.connect(robot+'/leftHand/rpc:o', robot+'/leftHand/rpc:i')


        self.listener = keyboard.Listener(on_release=self.on_release)
        self.listener.start()

        print("demo_sharon Module initialized...")
        return True

    def interruptModel(self):
        print("Stopping the module")

    def close(self):
        print("close")
        return True

    def getPeriod(self):
        return 0.5

    def updateModule(self):
        try:

            if self.state == 0:  # Put the trunk joints in the init state
                if self.firstInState:
                    if (self.checkJointsPosition(self.initTrunkJointsPosition, self.trunkIEncoders, self.numTrunkJoints) and self.checkJointsPosition(self.initRightArmJointsPosition, self.rightArmIEncoders, self.numRightArmJoints)): 
                        print("State 0: Trunk and right arm are in the init position")
                        self.state = 1
                        self.firstInState = True
                    else:
                        initTrunkAndRightArmPosition = self.initTrunkJointsPosition + self.initRightArmJointsPosition
                        
                        self.jointsTrajectory = []
                        found, self.jointsTrajectory = self.computeTrajectoryToJointsPosition(self.rightArm, initTrunkAndRightArmPosition)
                        if found:
                            self.numPointTrajectory = 0
                            # Lets plot the trajectory
                            print(len(self.jointsTrajectory))
                            self.smoothJointsTrajectory = self.computeSmoothJointsTrajectory()
                            print(len(self.jointsTrajectory))
                            self.plotTrajectories(self.jointsTrajectory, self.smoothJointsTrajectory)
                            self.firstInState = False
                else:
                    if(self.followJointsTrajectory(self.rightArm, self.jointsTrajectory)):
                        self.state = 1
                        self.firstInState = True
            if self.state == 1:
                if available_head_controlboard:
                    for joint in range(self.numHeadJoints):
                        self.headIPositionControl.positionMove(joint, self.initHeadJointsPosition[joint])
                    if (self.checkJointsPosition(self.initHeadJointsPosition, self.headIEncoders, self.numHeadJoints)):
                        print("State 1: Head joints are in the init position.")
                        self.state = 5
                        self.firstInState = True
                        yarp.delay(5.0)
                    else:
                        if self.firstInState:
                            print("State 1: Continue the head joints motion towards the init position.")
                            self.firstInState = False
                else:
                    self.state = 5
                    self.firstInState = True
            
            #     for joint in range(self.numTrunkJoints):
            #         self.trunkIPositionControl.positionMove(joint, self.initBackTrunkJointsPosition[joint])
            #     if (self.checkJointsPosition(self.initBackTrunkJointsPosition, self.trunkIEncoders, self.numTrunkJoints)):
            #         print("State 0: Trunks joints are in the back position.")
            #         self.state = 1
            #         self.firstInState = True
            #         yarp.delay(2.0)
            #     else:
            #         if self.firstInState:
            #             print("State 0: Continue the Trunk joints motion towards the back position.")
            #             self.firstInState = False
            # if self.state == 1: # Put the neck joints in the init state
            #     if available_head_controlboard:
            #         for joint in range(self.numHeadJoints):
            #             self.headIPositionControl.positionMove(joint, self.initHeadJointsPosition[joint])
            #         if (self.checkJointsPosition(self.initHeadJointsPosition, self.headIEncoders, self.numHeadJoints)):
            #             print("State 1: Head joints are in the init position.")
            #             self.state = 2
            #             self.firstInState = True
            #             yarp.delay(1.0)
            #         else:
            #             if self.firstInState:
            #                 print("State 1: Continue the head joints motion towards the init position.")
            #                 self.firstInState = False
            #     else:
            #         self.state = 2
            #         self.firstInState = True
            # if self.state == 2:  # Put the right arm joints in the init state
            #     if available_right_arm_controlboard:
            #         if (self.checkJointsPosition(self.initRightArmJointsPosition, self.rightArmIEncoders, self.numRightArmJoints)):
            #             print("State 2: Right arm joints are in the init position.")
            #             self.state = 3
            #             self.firstInState = True
            #         else:
            #             if self.firstInState:
            #                 print("State 2: Continue the Right arm joints motion towards the init position.")
            #                 self.firstInState = False
            #                 if available_right_hand_controlboard and robot == '/teoSim':
            #                     self.rightHandIPositionControl.positionMove(0, 1200)
            #                 if available_right_hand_controlboard and robot == '/teo':
            #                     self.rightHandIPWMControl.setRefDutyCycle(0,100)
            #                     self.rightHandIPWMControl.setRefDutyCycle(0,100)
            #                     self.rightHandIPWMControl.setRefDutyCycle(0,100)

            #                 if available_left_hand_controlboard:
            #                     self.leftHandIPositionControl.positionMove(0, 1200)
            #                 self.aux = [0, -50, 40, 0, 0, 0]
            #             # Move right arm joints 1 2
            #             if(not self.rightArmJointsInPosition[0] and not self.rightArmJointsInPosition[1]):
                            
            #                 if not self.checkJointsPosition(self.aux, self.rightArmIEncoders, self.numRightArmJoints):
            #                     print("State 2: Moving right arm joints 1 and 2")
            #                     if self.once :
            #                         for joint in range(self.numRightArmJoints):
            #                             self.rightArmIPositionControl.positionMove(joint, self.aux[joint])
            #                         self.once = False
            #                 else:
            #                     self.aux = [-50, -50, 40, 0, 0, 0]                    
            #                     self.rightArmJointsInPosition[1] = True
            #                     self.rightArmJointsInPosition[2] = True
            #                     self.once = True
                                
            #             # Move right arm joint 0   
            #             if(not self.rightArmJointsInPosition[0] and self.rightArmJointsInPosition[1]):
            #                 if not self.checkJointsPosition(self.aux, self.rightArmIEncoders, self.numRightArmJoints):
            #                     print("State 2: Moving right arm joint 0")
            #                     if self.once :
            #                         for joint in range(self.numRightArmJoints):
            #                             self.rightArmIPositionControl.positionMove(joint, self.aux[joint])
            #                         self.once = False
            #                 else:
            #                     self.rightArmJointsInPosition[0] = True
            #                     aux = [-50, -50, 40, 70, 0, 0]
            #                     self.once = True
                
            #             if(self.rightArmJointsInPosition[0] and not self.rightArmJointsInPosition[3]):
            #                 if not self.checkJointsPosition(self.aux, self.rightArmIEncoders, self.numRightArmJoints):
            #                     print("State 2: Moving right arm joint 3")
            #                     if self.once :
            #                         for joint in range(self.numRightArmJoints):
            #                             self.rightArmIPositionControl.positionMove(joint, self.aux[joint])
            #                         self.once = False
            #                 else:
            #                     self.rightArmJointsInPosition[3] = True
            #                     self.aux = self.initRightArmJointsPosition
            #                     self.once = True
                
                            
                        # if (self.rightArmJointsInPosition[3]):
                        #     if not self.checkJointsPosition(self.initRightArmJointsPosition, self.rightArmIEncoders, self.numRightArmJoints):
                        #         print("State 2: Moving Right arm joints 4 and 5.")
                        #         if self.once :
                        #             for joint in range(self.numRightArmJoints):
                        #                 self.rightArmIPositionControl.positionMove(joint, self.aux[joint])
                        #             self.once = False
                        #     else:
                        #         print("State 2: Right arm joints are in the init position")
                        #         self.state = 3
                        #         self.once = True
                        # if robot=="/teoSim":
                        #     yarp.delay(2.0)        
                        
                # else:
                #     self.once = True
                #     self.state = 3
                #     self.firstInState = True
            # if self.state == 3:  # Put the left arm joints in the init state
            #     for i  in range(len(self.rightArmJointsInPosition)):
            #         self.rightArmJointsInPosition[i] = False
            #     if available_left_arm_controlboard:
            #         # for joint in range(self.numLeftArmJoints):
            #         #     self.leftArmIPositionControl.positionMove(joint, self.initLeftArmJointsPosition[joint])
            #         if (self.checkJointsPosition(self.initLeftArmJointsPosition, self.leftArmIEncoders, self.numLeftArmJoints)):
            #             print("State 3: Left arm joints are in the init position.")
            #             self.state = 4
            #             self.firstInState = True
            #         else:
            #             if self.firstInState:
            #                 print("State 3: Continue the left arm joints motion towards the init position.")
            #                 self.firstInState = False
            #                 self.once = True
            #                 if available_left_hand_controlboard and robot == '/teoSim':
            #                     self.leftHandIPositionControl.positionMove(0, 1200)
            #                 if available_left_hand_controlboard and robot == '/teo':
            #                     self.leftHandIPWMControl.setRefDutyCycle(0,100)
            #                     self.leftHandIPWMControl.setRefDutyCycle(0,100)
            #                     self.leftHandIPWMControl.setRefDutyCycle(0,100)
            #                 self.aux = [0, 50, -40, 0, 0, 0]
            #             # Move right arm joints 1 2
            #             if(not self.rightArmJointsInPosition[0] and not self.rightArmJointsInPosition[1]):
                     
            #                 if not self.checkJointsPosition(self.aux, self.leftArmIEncoders, self.numRightArmJoints):
            #                     print("State 3: Moving left arm joints 1 and 2")
            #                     if self.once :
            #                         print("command")
            #                         for joint in range(self.numRightArmJoints):
            #                             self.leftArmIPositionControl.positionMove(joint, self.aux[joint])
            #                         self.once = False
            #                 else:
            #                     self.aux = [-50, 50, -40, 0, 0, 0]                    
            #                     self.rightArmJointsInPosition[1] = True
            #                     self.rightArmJointsInPosition[2] = True
            #                     self.once = True
                                
            #             # Move right arm joint 0   
            #             if(not self.rightArmJointsInPosition[0] and self.rightArmJointsInPosition[1]):
            #                 if not self.checkJointsPosition(self.aux, self.leftArmIEncoders, self.numRightArmJoints):
            #                     print("State 3: Moving left arm joint 0")
            #                     if self.once:
            #                         for joint in range(self.numRightArmJoints):
            #                             self.leftArmIPositionControl.positionMove(joint, self.aux[joint])
            #                         self.once = False
            #                 else:
            #                     self.rightArmJointsInPosition[0] = True
            #                     aux = [-50, 50, -40, -70, 0, 0]
            #                     self.once = True
                
            #             if(self.rightArmJointsInPosition[0] and not self.rightArmJointsInPosition[3]):
            #                 if not self.checkJointsPosition(self.aux, self.leftArmIEncoders, self.numRightArmJoints):
            #                     print("State 3: Moving left arm joint 3")
            #                     if self.once :
            #                         for joint in range(self.numRightArmJoints):
            #                             self.leftArmIPositionControl.positionMove(joint, self.aux[joint])
            #                         self.once = False
            #                 else:
            #                     self.rightArmJointsInPosition[3] = True
            #                     self.aux = self.initLeftArmJointsPosition
            #                     self.once = True
                
                            
            #             if (self.rightArmJointsInPosition[3]):
            #                 if not self.checkJointsPosition(self.initLeftArmJointsPosition, self.leftArmIEncoders, self.numRightArmJoints):
            #                     print("State 3: Moving Left arm joints 4 and 5.")
            #                     if self.once :
            #                         for joint in range(self.numRightArmJoints):
            #                             self.leftArmIPositionControl.positionMove(joint, self.aux[joint])
            #                         self.once = False
            #                 else:
            #                     print("State 3: Left arm joints are in the init position")
            #                     self.state = 4
            #                     self.once = True
            #     else:
            #         print("State 3: leftArm controlboard is not selected.")
            #         self.state = 4
            #         self.firstInState = True
            # if self.state == 4:  # Put the trunk joints in the init state

            #     for joint in range(self.numTrunkJoints):
            #         self.trunkIPositionControl.positionMove(
            #             joint, self.initTrunkJointsPosition[joint])
            #     if (self.checkJointsPosition(self.initTrunkJointsPosition, self.trunkIEncoders, self.numTrunkJoints)):
            #         print("State 4: Trunks joints are in the init position.")
            #         self.state = 5
            #         self.firstInState = True
            #         yarp.delay(5.0)
            #     else:
            #         if self.firstInState:
            #             print("State 4: Continue the Trunk joints motion towards the init position.")
            #             self.firstInState = False
            if self.state == 5:
                if self.firstInState:
                    if not yarp.Network.isConnected(self.matching_object_output_port_name, self.demo_object_input_port_name):
                        print("Try to connect to",self.matching_object_output_port_name)
                        print("Check if the mathing.py script is running.")
                        yarp.Network.connect(self.matching_object_output_port_name, self.demo_object_input_port_name)

                    else:
                        print("State 5: Waiting for object location ...")
                        objectData = self.objectPositionPort.read(False)
                        print(objectData)

                        if objectData:
                            #TODO: object coordinates wrt trunk
                            yarp.Network.disconnect(self.matching_object_output_port_name, self.demo_object_input_port_name)
                            self.objectData = objectData
                            category = self.objectData.get(0).find('category').asString()
                            self.locationTCP = self.transformFromXYZFromCameraCoordinatesToTrunk(self.objectData, category)
                            self.locationTCP[1] = self.locationTCP[1]- 0.01
                            if self.locationTCP[1]<=0.1:
                                self.rightArm = True    
                            else:
                                self.rightArm = False
                            
                            print("State 5: We are ready to move near the object ", category, "in location: ", self.locationTCP)
                            self.reachingDistance = 0.1
                            feasible, self.orientationTCP, self.reachingPose = self.computeFeasibleOrientation(
                                self.rightArm, self.locationTCP, self.reachingDistance)
                            if feasible:
                                self.jointsTrajectory = []
                                found, self.jointsTrajectory = self.computeTrajectoryToPose(
                                    self.rightArm, self.reachingPose)
                                if found:
                                    self.numPointTrajectory = 0
                                    # Lets plot the trajectory
                                    print(len(self.jointsTrajectory))
                                    self.smoothJointsTrajectory = self.computeSmoothJointsTrajectory()
                                    print(len(self.jointsTrajectory))
                                    self.plotTrajectories(self.jointsTrajectory, self.smoothJointsTrajectory)

                                    self.followJointsTrajectory(self.rightArm, self.smoothJointsTrajectory)
                                    self.firstInState = False
                                else:
                                    print("Solution NOT found")
                                    self.state = 5
                                    self.firstInState = True
                            else:
                                print("solution not found")
                                self.state = 5
                                self.firstInState = True
                else:
                    
                    print("State 5: Following path to the reaching pose.")
                    # Finished following the joints trajectory
                    if(self.followJointsTrajectory(self.rightArm, self.jointsTrajectory)):
                        self.d = 0
                        self.state = 6
                        self.firstInState = True
            if self.state == 6:
                if self.firstInState:
                    if available_right_hand_controlboard and robot == '/teoSim':
                        self.rightHandIPositionControl.positionMove(0, 1200)
                    if available_right_hand_controlboard and robot == '/teo':
                        self.rightHandIPWMControl.setRefDutyCycle(0,100)
                    yarp.delay(5.0)
                    print("State 6: Hand Open")
                    self.firstInState = False
                    
                else:
                    print("State 6: Move to towards object.")
                    while self.d <= 0.85*self.reachingDistance:
                        print(self.d)
                        self.d = self.d + self.reachingDistance/40.0

                        frame_reaching_base = self.vectorToFrame(self.reachingPose)

                        frame_aux_reaching = PyKDL.Frame()
                        frame_aux_reaching.p.z(self.d)

                        frame_aux_base = frame_reaching_base*frame_aux_reaching

                        aux_pose = self.frameToVector(frame_aux_base)
                        x_vector = yarp.DVector(6)
                        for i in range(len(x_vector)):
                            x_vector[i] = aux_pose[i]

                        #print(x_vector[0], x_vector[1], x_vector[2], x_vector[3], x_vector[4], x_vector[5])
                        trunkCurrentQ = yarp.DVector(self.numTrunkJoints)
                        self.trunkIEncoders.getEncoders(trunkCurrentQ)

                        armCurrentQ = yarp.DVector(self.numRightArmJoints)
                        self.rightArmIEncoders.getEncoders(armCurrentQ)

                        current_Q = yarp.DVector(
                            self.numRightArmJoints+self.numTrunkJoints)
                        desire_Q = yarp.DVector(
                            self.numRightArmJoints+self.numTrunkJoints)

                        for j in range(0, self.numRightArmJoints):
                            current_Q[j+2] = armCurrentQ[j]
                        for j in range(self.numTrunkJoints):
                            current_Q[j] = trunkCurrentQ[j]

                        if self.rightArm:
                            if(self.trunkRightArmICartesianSolver.invKin(x_vector, current_Q, desire_Q)):
                                for joint in range(0, self.numRightArmJoints, 1):
                                    self.rightArmIPositionControl.positionMove(joint, desire_Q[joint+2])
                                for joint in range(self.numTrunkJoints):
                                        self.trunkIPositionControl.positionMove(joint, desire_Q[joint])
                                yarp.delay(0.5)
                        else:
                            if(self.trunkLeftArmICartesianSolver.invKin(x_vector, current_Q, desire_Q)):
                                for joint in range(0, self.numLeftArmJoints, 1):
                                    self.leftArmIPositionControl.positionMove(joint, desire_Q[joint+2])
                                for joint in range(self.numTrunkJoints):
                                    self.trunkIPositionControl.positionMove(joint, desire_Q[joint])
                                if self.d == 0:
                                    yarp.delay(5.0)
                                else:
                                    yarp.delay(0.5)
                    self.state = 7
                    self.firstInState = True
            if self.state == 7:
                if self.firstInState:
                    print("State 7: TCP in object position")
                    if self.rightArm:
                        if available_right_hand_controlboard and robot == '/teoSim':
                            self.rightHandIPositionControl.positionMove(0, -300)
                        elif available_right_hand_controlboard and robot == '/teo':
                            self.rightHandIPWMControl.setRefDutyCycle(0, -50)             
                        yarp.delay(8.0)
                        print("State 7: Lets move up")
                        # Get current pose of the tcp
                        trunkCurrentQ = yarp.DVector(self.numTrunkJoints)
                        self.trunkIEncoders.getEncoders(trunkCurrentQ)
            
                        rightArmCurrentQ = yarp.DVector(self.numRightArmJoints)
                        self.rightArmIEncoders.getEncoders(rightArmCurrentQ)
            
                        current_Q = yarp.DVector(self.numTrunkJoints+self.numRightArmJoints)       
                        for j in range(0, self.numRightArmJoints):
                            current_Q[j+2] = rightArmCurrentQ[j]
                        for j in range(self.numTrunkJoints):
                            current_Q[j] = trunkCurrentQ[j]
            
                        print(current_Q[0],current_Q[1], current_Q[2], current_Q[3], current_Q[4], current_Q[5], current_Q[6], current_Q[7])
            
                        x_tcp_trunk = yarp.DVector(6)
                        self.trunkRightArmICartesianSolver.fwdKin(current_Q, x_tcp_trunk)
                        self.frame_tcp_trunk = self.vectorToFrame(x_tcp_trunk)
                        # Block the demo here just grasping
                        self.firstInState = False
                    else:
                        if available_left_hand_controlboard and robot == '/teoSim':
                            self.leftHandIPositionControl.positionMove(0, -600)
                        if available_left_hand_controlboard and robot == '/teo':
                            self.leftHandIPWMControl.setRefDutyCycle(0, -50)             
                        print("State 7: Wait 8 seconds...")
                        yarp.delay(8.0)
                        print("State 7: Lets move up")
                        # TODO CHANGE UP MOTION
                        if available_left_hand_controlboard:
                            self.leftArmIPositionControl.positionMove(2, -40)
                        
                        self.firstInState = False
                    self.d = 0
                else:          
                    
                    self.d = self.d+self.up_distance/40.0
                    if self.d > self.up_distance:
                        self.firstInState = True
                        self.state = 8
                    else:
                        v = PyKDL.Vector(self.frame_tcp_trunk.p)
                        v.z(v[2]+self.d)
                        rot = self.frame_tcp_trunk.M
                        frame_up_trunk = PyKDL.Frame(rot, v)

                        up_pose = self.frameToVector(frame_up_trunk)
                        x_vector = yarp.DVector(6)
                        for i in range(len(x_vector)):
                            x_vector[i] = up_pose[i]

                        trunkCurrentQ = yarp.DVector(self.numTrunkJoints)
                        self.trunkIEncoders.getEncoders(trunkCurrentQ)
                        current_Q = yarp.DVector(self.numRightArmJoints+self.numTrunkJoints)
                        desire_Q = yarp.DVector(self.numRightArmJoints+self.numTrunkJoints)
                        
                        for j in range(self.numTrunkJoints):
                            current_Q[j] = trunkCurrentQ[j]
                        if self.rightArm:
                            # Get current Q
                            armCurrentQ = yarp.DVector(self.numRightArmJoints)
                            self.rightArmIEncoders.getEncoders(armCurrentQ)
                            for j in range(0, self.numRightArmJoints):
                                current_Q[j+2] = armCurrentQ[j]
                            # print(current_Q)
                            
                            if(self.trunkRightArmICartesianSolver.invKin(x_vector, current_Q, desire_Q)):
                                for joint in range(0, self.numRightArmJoints, 1):
                                    self.rightArmIPositionControl.positionMove(joint, desire_Q[joint+2])
                                for joint in range(self.numTrunkJoints):
                                    self.trunkIPositionControl.positionMove(joint, desire_Q[joint])
                     
                            if available_right_hand_controlboard and robot=="/teo":
                                self.rightHandIPWMControl.setRefDutyCycle(0, -50)
                            # if available_right_hand_controlboard and robot="/teoSim":
                        else:
                            if available_left_hand_controlboard and robot=="/teo":
                                self.leftHandIPWMControl.setRefDutyCycle(0,-50)
                            armCurrentQ = yarp.DVector(self.numLeftArmJoints)
                            self.LeftArmIEncoders.getEncoders(armCurrentQ)
                            for j in range(0, self.numLeftArmJoints):
                                current_Q[j+2] = armCurrentQ[j]
                            # print(current_Q)
                            
                            if(self.trunkLeftArmICartesianSolver.invKin(x_vector, current_Q, desire_Q)):
                                for joint in range(0, self.numLeftArmJoints, 1):
                                    self.leftArmIPositionControl.positionMove(joint, desire_Q[joint+2])
                                for joint in range(self.numTrunkJoints):
                                    self.trunkIPositionControl.positionMove(joint, desire_Q[joint])
            if self.state == 8:
                if(self.firstInState):
                    print("State 8: Object is up and graspped. Press [enter] to open the hand and release the object")
                    with keyboard.Events() as events:
                        # Block at most one second
                        event = events.get(0.1)
                    if event is None:
                        if self.rightArm and available_right_hand_controlboard and robot == '/teo':
                            self.rightHandIPWMControl.setRefDutyCycle(0, -50)
                        if not self.rightArm and available_left_hand_controlboard and robot == '/teo':
                            self.leftHandIPWMControl.setRefDutyCycle(0,-50)
                        pass
                    elif event.key == keyboard.Key.enter:
                        print('Received event {}'.format(event))
                        print("State 8: Waiting 8 seconds to open the hand. ")
                        yarp.delay(8.0)
                        if self.rightArm:
                            if available_right_hand_controlboard  and robot == '/teoSim':
                                self.rightHandIPositionControl.positionMove(0, 1200)
                            elif available_right_hand_controlboard and robot == '/teo':
                                self.rightHandIPWMControl.setRefDutyCycle(0,100)
                            print("State 7: Wait the hand to be open...")
                            yarp.delay(8.0)
                        else:
                            if available_left_hand_controlboard:
                                self.leftHandIPositionControl.positionMove(0, 1200)
                            print("State 7: Wait left hand to be open...")
                            yarp.delay(8.0)
                        self.state = 9
            if self.state == 9:
                print("State 9: Position the object again in the table. Then, Press [enter] to start again the demo.")
                with keyboard.Events() as events:
                    # Block at most one second
                    event = events.get(1.0)
                    if event is None:
                        pass
                    elif event.key == keyboard.Key.enter:
                        self.state = 0
                        self.firstInState = True
                        print("Start again the demo.")

        except KeyboardInterrupt:
            print("Press Ctrl-C to terminate while statement")
        pass                                                       
        return True

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
        cmd.addString("Compute trajectory joints position")
        goal = cmd.addList()
        
        for jointPosition in reachingJointsPosition:
            goal.addDouble(jointPosition)
        
        response = yarp.Bottle()
        jointsTrajectory = []
        if rightArm:
            self.rpcClientTrajectoryGenerationRight.write(cmd, response)
        else:
            self.rpcClientTrajectoryGenerationLeft.write(cmd, response)
            
        if response.get(0).asString() == 'Goal state NOT valid':
            print(response.get(0).asString())
        elif response.get(0).asString() == 'Solution Found':
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
        elif response.get(0).asString() =='Solution NOT found':
            return False, jointsTrajectory

        return False, jointsTrajectory
           
            

    def computeTrajectoryToPose(self, rightArm, reachingPose):
        if rightArm:  # Compute the a feasible orientation for the rightArm
            print("Selected right arm.")
        else:
            print("Selected left arm.")
            
        cmd = yarp.Bottle()
        cmd.addString("Compute trajectory pose")
        goal = cmd.addList()
        goal.addDouble(reachingPose[0])
        goal.addDouble(reachingPose[1])
        goal.addDouble(reachingPose[2])
        goal.addDouble(reachingPose[3])
        goal.addDouble(reachingPose[4])
        goal.addDouble(reachingPose[5])
        response = yarp.Bottle()
        jointsTrajectory = []
        if rightArm:
            self.rpcClientTrajectoryGenerationRight.write(cmd, response)
        else:
            self.rpcClientTrajectoryGenerationLeft.write(cmd, response)
            
        if response.get(0).asString() == 'Goal state NOT valid':
            print(response.get(0).asString())
        elif response.get(0).asString() == 'Solution Found':
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
        elif response.get(0).asString() =='Solution NOT found':
            return False, jointsTrajectory

        return False, jointsTrajectory

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

        for rotAroundAxisZ in np.linspace(-np.pi/8.0, np.pi/8.0,10):
            for rotAroundAxisY in np.linspace(-np.pi/6.0, np.pi/6.0, 10):
                for rotAroundAxisX in np.linspace(-np.pi/2.0, np.pi/2.0, 30):
                    # rotAroundAxisX = 0
                    # rotAroundAxisY = 0
                    # print(rotAroundAxisX, rotAroundAxisY, rotAroundAxisZ)
                    aux_frame_goal = PyKDL.Frame()
                    aux_frame_goal.M = frame_goal.M * PyKDL.Rotation.RotX(rotAroundAxisX)
                    aux_frame_goal.M = aux_frame_goal.M * PyKDL.Rotation.RotY(rotAroundAxisY)
                    aux_frame_goal.M = aux_frame_goal.M * PyKDL.Rotation.RotZ(rotAroundAxisZ)

                    goal_vector = self.frameToVector(aux_frame_goal)
                    cmd = yarp.Bottle()
                    cmd.addString("Check goal pose")
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
                    print("check goal: ",locationTCP[0], locationTCP[1], locationTCP[2], goal_vector[3],goal_vector[4],goal_vector[5])

                    orientationTCP = [goal_vector[3],goal_vector[4], goal_vector[5]]
                    response = yarp.Bottle()
                    jointsTrajectory = []
                    
                    if rightArm:    
                        self.rpcClientTrajectoryGenerationRight.write(cmd, response)
                    else:
                        self.rpcClientTrajectoryGenerationLeft.write(cmd, response)
                        reaching_pose = []

                    if response.get(0).asString() == 'Valid':
                        print("Valid final pose")
                        x = [locationTCP[0], locationTCP[1], locationTCP[2], orientationTCP[0], orientationTCP[1], orientationTCP[2]]
                        frame_end_base = self.vectorToFrame(x)

                        frame_reaching_end = PyKDL.Frame()
                        frame_reaching_end.p.z(-reachingDistance)

                        frame_reaching_base = frame_end_base*frame_reaching_end

                        reaching_pose = self.frameToVector(frame_reaching_base)

                        cmd.clear()
                        cmd.addString("Check goal pose")
                        goal = cmd.addList()
                        for i in range(len(reaching_pose)):
                            goal.addDouble(reaching_pose[i])
                        response.clear()
                        if rightArm:
                            self.rpcClientTrajectoryGenerationRight.write(cmd, response)
                        else:
                            self.rpcClientTrajectoryGenerationLeft.write(cmd, response)
                            
                        if response.get(0).asString() == 'Valid':
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
        for joint in range(numJoints):
            if abs(currentQ[joint]-desiredJointsPosition[joint]) > self.jointsPositionError:
                return False
        return True

    def transformFromXYZFromCameraCoordinatesToTrunk(self,objects_data, category_object_to_grasp):
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
        
        print(current_Q[0],current_Q[1], current_Q[2], current_Q[3])
        
        x_head_trunk = yarp.DVector(6)
        self.trunkHeadICartesianSolver.fwdKin(current_Q, x_head_trunk)
        
        print(x_head_trunk[0], x_head_trunk[1], x_head_trunk[2], x_head_trunk[3], x_head_trunk[4], x_head_trunk[5])
        # TODO: transform to kdl frame head_wrt_trunk and object xyz -> object_xyz_wrt_trunk
        
        frame_head_trunk = self.vectorToFrame(x_head_trunk)
        
        

        
        
        
        o = 0
        for o in range(objects_data.size()):
            if category_object_to_grasp == objects_data.get(o).find("category").asString():
                break
            
        if  category_object_to_grasp != objects_data.get(o).find("category").asString():
            print("Object of category ", category_object_to_grasp, "is NOT detected")
        else:
            print("Object of category ", category_object_to_grasp, "is detected")


       
        if robot=="/teo":
            frame_camera_head = PyKDL.Frame()
            print(frame_camera_head)
            frame_camera_head = frame_camera_head*PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(0,0,0.146))
            frame_camera_head = frame_camera_head*PyKDL.Frame(PyKDL.Rotation.RotZ(-np.pi/2.0), PyKDL.Vector(0,0,0))
            frame_camera_head = frame_camera_head*PyKDL.Frame(PyKDL.Rotation.RotX(-np.pi/2.0), PyKDL.Vector(0,0,0))
            frame_camera_head = frame_camera_head*PyKDL.Frame(PyKDL.Rotation(), PyKDL.Vector(-0.018-0.026,0,0))
            print(frame_camera_head)

            frame_object_camera = PyKDL.Frame()
            
            frame_object_camera.p.x(objects_data.get(o).find("mmX").asDouble())
            frame_object_camera.p.y(objects_data.get(o).find("mmY").asDouble())
            frame_object_camera.p.z(objects_data.get(o).find("mmZ").asDouble())
            
            frame_object_head = frame_camera_head*frame_object_camera
            frame_object_trunk = frame_head_trunk*frame_object_head
            
            pose_object_trunk = self.frameToVector(frame_object_trunk)
        else:
            frame_object_camera = PyKDL.Frame()

            frame_object_camera.p.x(objects_data.get(o).find("mmZ").asDouble())
            frame_object_camera.p.y(objects_data.get(o).find("mmX").asDouble())
            frame_object_camera.p.z(-objects_data.get(o).find("mmY").asDouble())

            frame_object_trunk = frame_head_trunk*frame_object_camera

            pose_object_trunk = self.frameToVector(frame_object_trunk)
        
        # print(pose_object_camera[0], pose_object_camera[1], pose_object_camera[2])
        
        return [pose_object_trunk[0], pose_object_trunk[1], pose_object_trunk[2]]

    def followJointsTrajectory(self, rightArm, jointsTrajectory):
        print("followJointsTrajectory ", self.numPointTrajectory, len(self.jointsTrajectory))
        if self.numPointTrajectory < len(self.jointsTrajectory)-1:    
            jointsPosition = jointsTrajectory[self.numPointTrajectory]
            trunkJointsPosition = [jointsPosition[0], jointsPosition[1]]
            armJointsPosition = [jointsPosition[2], jointsPosition[3], jointsPosition[4], jointsPosition[5], jointsPosition[6], jointsPosition[7]]
            
            if rightArm:
                #if (self.checkJointsPosition(trunkJointsPosition, self.trunkIEncoders, self.numTrunkJoints) and self.checkJointsPosition(armJointsPosition, self.rightArmIEncoders, self.numRightArmJoints)):
                self.numPointTrajectory = self.numPointTrajectory + 1
                #else:
                for joint in range(self.numTrunkJoints):
                    if joint ==0:
                        self.trunkIPositionControl.positionMove(joint, trunkJointsPosition[joint])                
                    else:
                        self.trunkIPositionControl.positionMove(joint, trunkJointsPosition[joint])    
                for joint in range(self.numRightArmJoints):
                    self.rightArmIPositionControl.positionMove(joint, armJointsPosition[joint])
            else:
                #if (self.checkJointsPosition(trunkJointsPosition, self.trunkIEncoders, self.numTrunkJoints) and self.checkJointsPosition(armJointsPosition, self.leftArmIEncoders, self.numLeftArmJoints)):
                self.numPointTrajectory = self.numPointTrajectory + 1
                #else:
                for joint in range(self.numLeftArmJoints):
                    self.leftArmIPositionControl.positionMove(joint, armJointsPosition[joint])
                for joint in range(self.numTrunkJoints):
                    self.trunkIPositionControl.positionMove(joint, trunkJointsPosition[joint])      
  
            return False
        else:
            jointsPosition = jointsTrajectory[self.numPointTrajectory]
            trunkJointsPosition = [jointsPosition[0], jointsPosition[1]]
            armJointsPosition = [jointsPosition[2], jointsPosition[3], jointsPosition[4], jointsPosition[5], jointsPosition[6], jointsPosition[7]]
            if rightArm:            
                if (self.checkJointsPosition(trunkJointsPosition, self.trunkIEncoders, self.numTrunkJoints) and self.checkJointsPosition(armJointsPosition, self.rightArmIEncoders, self.numRightArmJoints)):
                    print("followJointsTrajectory in goal position", self.numPointTrajectory, len(self.jointsTrajectory))
                    return True
                else:
                    for joint in range(self.numTrunkJoints):
                        self.trunkIPositionControl.positionMove(joint, trunkJointsPosition[joint])
               
                    for joint in range(self.numRightArmJoints):
                        self.rightArmIPositionControl.positionMove(joint, armJointsPosition[joint])
            else:
                if (self.checkJointsPosition(trunkJointsPosition, self.trunkIEncoders, self.numTrunkJoints) and self.checkJointsPosition(armJointsPosition, self.leftArmIEncoders, self.numRightArmJoints)):
                    return True
                else:
                    for joint in range(self.numTrunkJoints):
                        self.trunkIPositionControl.positionMove(joint, trunkJointsPosition[joint])
                    for joint in range(self.numRightArmJoints):
                        self.leftArmIPositionControl.positionMove(joint, armJointsPosition[joint])
            return False
    
    def on_release(self,key):
        if key == keyboard.Key.enter:
            print("enter is pressed")
            if self.state == 7:
                self.enter_is_pressed = True
        elif key == keyboard.Key.esc:
            return False
    
    def computeSmoothJointsTrajectory(self):
        smoothJointsTrajectory = []
        arr = np.asarray(self.jointsTrajectory)
        t = range(0,len(self.jointsTrajectory))

        tck, u = interpolate.splprep([t,arr[:,0]], s=0.8)
        #create interpolated lists of points
        tnew, q0new = interpolate.splev(u,tck)
        
        tck, u = interpolate.splprep([t,arr[:,1]], s=0.8)
        #create interpolated lists of points
        tnew, q1new = interpolate.splev(u,tck)
        
        tck, u = interpolate.splprep([t, arr[:,2]], s=0.8)
        #create interpolated lists of points
        tnew, q2new = interpolate.splev(u,tck)
        
        tck, u = interpolate.splprep([t, arr[:,3]], s=0.8)
        #create interpolated lists of points
        tnew, q3new = interpolate.splev(u,tck)
        
        tck, u = interpolate.splprep([t, arr[:,4]], s=0.8)
        #create interpolated lists of points
        tnew, q4new = interpolate.splev(u,tck)
        
        tck, u = interpolate.splprep([t,arr[:,5]], s=0.8)
        #create interpolated lists of points
        tnew, q5new = interpolate.splev(u,tck)
        
        tck, u = interpolate.splprep([t, arr[:,6]], s=0.8)
        #create interpolated lists of points
        tnew, q6new = interpolate.splev(u,tck)
        
        tck, u = interpolate.splprep([t, arr[:,7]], s=0.8)
        #create interpolated lists of points
        tnew, q7new = interpolate.splev(u,tck)
        
        for i in range(0, len(tnew)):
            jointsPosition = [q0new[i], q1new[i], q2new[i], q3new[i], q4new[i], q5new[i], q6new[i], q7new[i]]
            smoothJointsTrajectory.append(jointsPosition)
        return smoothJointsTrajectory

        

                

    def plotTrajectories(self, jointsTrajectory, smoothJointsTrajectory):
        arr = np.asarray(jointsTrajectory)
        t = range(0,len(jointsTrajectory))
        
        arr_smooth = np.asarray(smoothJointsTrajectory)
        t_smooth = range(0,len(smoothJointsTrajectory))

        fig, ax = plt.subplots(8, sharex=True)
        plt.subplots_adjust(left=0.15, right=0.95, top=0.95, bottom=0.05)        
        #set axes labels
        # ax[0].set_xlabel("num point")
        ax[0].set_ylabel("Axial Trunk [deg]", rotation=0, ha="right")
        ax[1].set_ylabel("Frontal Trunk [deg]",  rotation=0, ha="right")
        if self.rightArm:
            ax[2].set_ylabel("Frontal Right Shoulder [deg]", rotation=0, ha="right")
            ax[3].set_ylabel("Sagittal Right Shoulder [deg]", rotation=0, ha="right")
            ax[4].set_ylabel("Axial Right Shoulder [deg]", rotation=0, ha="right")
            ax[5].set_ylabel("Frontal Right Elbow [deg]", rotation=0, ha="right")
            ax[6].set_ylabel("Axial Right Wrist [deg]", rotation=0, ha="right")
            ax[7].set_ylabel("Frontal Right Wrist [deg]", rotation=0, ha="right")
        else:
            ax[2].set_ylabel("Frontal Left Shoulder [deg]", rotation=0, ha="right")
            ax[3].set_ylabel("Sagittal Left Shoulder [deg]", rotation=0, ha="right")
            ax[4].set_ylabel("Axial Left Shoulder [deg]", rotation=0, ha="right")
            ax[5].set_ylabel("Frontal Left Elbow [deg]",  rotation='horizontal', labelpad = 15)
            ax[6].set_ylabel("Axial Left Wrist [deg]",  rotation='horizontal', labelpad = 15)
            ax[7].set_ylabel("Frontal Left Wrist [deg]",  rotation='horizontal', labelpad = 15)         

        fig.suptitle("Joints trajectories.")
        for i in range(8):
            ax[i].grid()
            ax[i].set_xlim(0, len(jointsTrajectory))
            ax[i].yaxis.set_major_locator(MaxNLocator(5)) 
        ax[0].plot(t, arr[:,0], label='q0', color='skyblue')
        ax[0].plot(t_smooth, arr_smooth[:,0], label='q0 smooth', color='red', linestyle='dashed')

        ax[1].plot(t, arr[:,1], label='q1')
        ax[1].plot(t_smooth, arr_smooth[:,1], label='q1 smooth', color='red', linestyle='dashed')

            
        ax[2].plot(t, arr[:,2], label='q2')
        ax[2].plot(t_smooth, arr_smooth[:,2], label='q2 smooth', color='red', linestyle='dashed')

        ax[3].plot(t, arr[:,3], label='q3')
        ax[3].plot(t_smooth, arr_smooth[:,3], label='q3 smooth', color='red', linestyle='dashed')

        ax[4].plot(t, arr[:,4], label='q4')
        ax[4].plot(t_smooth, arr_smooth[:,4], label='q4 smooth', color='red', linestyle='dashed')

        ax[5].plot(t, arr[:,5], label='q5')
        ax[5].plot(t_smooth, arr_smooth[:,5], label='q5 smooth', color='red', linestyle='dashed')

        ax[6].plot(t, arr[:,6], label='q6')
        ax[6].plot(t_smooth, arr_smooth[:,6], label='q6 smooth', color='red', linestyle='dashed')

        ax[7].plot(t, arr[:,7], label='Joint trajectory')
        ax[7].plot(t_smooth, arr_smooth[:,7], label='Smoothed joint trajectory', color='red', linestyle='dashed')
        ax[7].xaxis.set_ticks(np.arange(0, len(jointsTrajectory), 5))

        handles, labels = ax[7].get_legend_handles_labels()
        fig.legend(handles, labels, loc='upper right')
        
        plt.show()
        plt.figure().clear()
        plt.close()
    


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
