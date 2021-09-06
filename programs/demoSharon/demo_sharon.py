import yarp
import sys
import PyKDL
import numpy as np
import kinematics_dynamics


robot = '/teoSim'


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
        # Trunk device
        self.trunkOptions = yarp.Property()
        self.trunkDevice = None
        self.trunkIEncoders = None
        self.numTrunkJoints = 0
        self.trunkIPositionControl = None
        
        # Head device
        self.headOptions = yarp.Property()
        self.headDevice = None
        self.headIEncoders = None
        self.numHeadJoints = 0
        self.headIPositionControl = None
        self.trunkHeadIControlLimits = None

        # Trajectory Generation client
        self.rpcClientTrajectoryGenerationRight = yarp.RpcClient()
        self.rpcClientTrajectoryGenerationLeft = yarp.RpcClient()

        # Underactuated Hand rpc client
        self.rpcClientRightHand = yarp.RpcClient()
        self.rpcClientLeftHand = yarp.RpcClient()

        # State
        self.state = 0
        self.firstInState = True  # For printing messages just once
        self.jointsPositionError = 5.0

        # Init Joints position for grasping
        self.initTrunkJointsPosition = [0, 9.0]
        self.initHeadJointsPosition = [0, 14.5]

        # [-90, 0, 0, 0, 90, 0]
        self.initRightArmJointsPosition = [-50, -50, 40, -70, 30, -20]
        self.initLeftArmJointsPosition = [-50, 50, -40, -70, -30, -20]

        # Object category and position port
        self.objectPositionPort = yarp.Port()
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
        self.up_distance = 0.4


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

    def configure(self, rf):
        print("Configuring...")

        self.objectPositionPort.open("/demoSharon/object:i")
        yarp.Network.connect("/matching/object:o", "/demoSharon/object:i")

        # Open rightArm device
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
        if not self.rightHandDevice.isValid():
            print('Cannot open rightHand device!')
            raise SystemExit
        
        # Open leftArm device
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
        if not self.leftHandDevice.isValid():
            print('Cannot open leftHand device!')
            raise SystemExit

        # Open trunk device
        self.trunkOptions.put('device', 'remote_controlboard')
        self.trunkOptions.put('remote', robot+'/trunk')
        self.trunkOptions.put('local', robot+'/trunk')

        self.trunkDevice = yarp.PolyDriver(self.trunkOptions)
        self.trunkDevice.open(self.trunkOptions)
        if not self.trunkDevice.isValid():
            print('Cannot open trunk device!')
            raise SystemExit

        # Open head device
        self.headOptions.put('device', 'remote_controlboard')
        self.headOptions.put('remote', robot+'/head')
        self.headOptions.put('local', robot+'/head')

        self.headDevice = yarp.PolyDriver(self.headOptions)
        self.headDevice.open(self.headOptions)
        if not self.headDevice.isValid():
            print('Cannot open head device!')
            raise SystemExit

        # Interface with right arm encoders
        self.rightArmIEncoders = self.rightArmDevice.viewIEncoders()

        if self.rightArmIEncoders == []:
            print("Right arm Encoder interface NOT available.")
            raise SystemExit
        else:
            print("Right arm Encoder interface available.")

        self.numRightArmJoints = self.rightArmIEncoders.getAxes()

        # Interface with the emulated right Hand encoders
        
        self.rightHandIEncoders = self.rightHandDevice.viewIEncoders()
        if self.rightHandIEncoders == []:
            print("Right Hand Encoder interface NOT available.")
            raise SystemExit
        else:
            print("Right Hand Encoder interface available.")
        
        self.numRightHandJoints = self.rightHandIEncoders.getAxes()

        
        # Right arm position control interface
        self.rightArmIPositionControl = self.rightArmDevice.viewIPositionControl()

        if self.rightArmIPositionControl == []:
            print("Right arm position control interface NOT available")
            raise SystemExit
        else:
            print("Right arm position control interface available.")

        # Right hand position control interface
        self.rightHandIPositionControl = self.rightHandDevice.viewIPositionControl()

        if self.rightHandIPositionControl == []:
            print("Right hand position control interface NOT available")
            raise SystemExit
        else:
            print("Right hand position control interface available.")

        # Interface with left arm encoders
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
        self.leftHandIPositionControl = self.leftHandDevice.viewIPositionControl()

        if self.leftHandIPositionControl == []:
            print("Left hand position control interface NOT available")
            raise SystemExit
        else:
            print("Left hand position control interface available.")

        # Interface with trunk encoders
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

        self.trunkRightArmSolverOptions.fromString(
            "(mins (-20 -10.8 -98.1 -75.5 -80.1 -99.6 -80.4 -115.4))", False)
        self.trunkRightArmSolverOptions.fromString(
            "(maxs (20 10.1 106 22.4 57 98.4 99.6 44.7))", False)
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
            "(mins (-20 -10.8 -98.1 -75.5 -80.1 -99.6 -80.4 -115.4))", False)
        self.trunkLeftArmSolverOptions.fromString(
            "(maxs (20 10.1 106 22.4 57 98.4 99.6 44.7))", False)
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
        self.rpcClientRightHand.open(robot+'/rightHand/rpc:o')
        yarp.Network.connect(robot+'/rightHand/rpc:o', robot+'/rightHand/rpc:i')

        # Open rpc client for leftHand        
        self.rpcClientLeftHand.open(robot+'/leftHand/rpc:o')
        yarp.Network.connect(robot+'/leftHand/rpc:o', robot+'/leftHand/rpc:i')

        print("demo_sharon Module initialized...")
        return True

    def interruptModel(self):
        print("Stopping the module")

    def close(self):
        return True

    def getPeriod(self):
        return 0.6

    def updateModule(self):

        if self.state == 0:  # Put the trunk joints in the init state

            for joint in range(self.numTrunkJoints):
                self.trunkIPositionControl.positionMove(
                    joint, self.initTrunkJointsPosition[joint])
            if (self.checkJointsPosition(self.initTrunkJointsPosition, self.trunkIEncoders, self.numTrunkJoints)):
                print("State 0: Trunks joints are in the init position.")
                self.state = 1
                self.firstInState = True
                yarp.delay(2.0)
            else:
                if self.firstInState:
                    print("State 0: Continue the Trunk joints motion towards the init position.")
                    self.firstInState = False
        if self.state == 1: # Put the neck joints in the init state
            for joint in range(self.numHeadJoints):
                self.headIPositionControl.positionMove(joint, self.initHeadJointsPosition[joint])
            if (self.checkJointsPosition(self.initHeadJointsPosition, self.headIEncoders, self.numHeadJoints)):
                print("State 1: Head joints are in the init position.")
                self.state = 2
                self.firstInState = True
                yarp.delay(1.0)
            else:
                if self.firstInState:
                    print("State 1: Continue the head joints motion towards the init position.")
                    self.firstInState = False
        if self.state == 2:  # Put the right arm joints in the init state
            if (self.checkJointsPosition(self.initRightArmJointsPosition, self.rightArmIEncoders, self.numRightArmJoints)):
                print("State 2: Right arm joints are in the init position.")
                self.state = 3
                self.firstInState = True
            else:
                if self.firstInState:
                    print("State 2: Continue the Right arm joints motion towards the init position.")
                    self.firstInState = False
                    self.rightHandIPositionControl.positionMove(0, 1200)
                    self.leftHandIPositionControl.positionMove(0, 1200)
                    self.aux = [0, -50, 40, 0, 0, 0]
                # Move right arm joints 1 2
                if(not self.rightArmJointsInPosition[0] and not self.rightArmJointsInPosition[1]):
                    if not self.checkJointsPosition(self.aux, self.rightArmIEncoders, self.numRightArmJoints):
                        print("State 2: Moving right arm joints 1 and 2")
                    else:
                        self.aux = [-50, -50, 40, 0, 0, 0]                    
                        self.rightArmJointsInPosition[1] = True
                        self.rightArmJointsInPosition[2] = True
                        
                # Move right arm joint 0   
                if(not self.rightArmJointsInPosition[0] and self.rightArmJointsInPosition[1]):
                    if not self.checkJointsPosition(self.aux, self.rightArmIEncoders, self.numRightArmJoints):
                        print("State 2: Moving right arm joint 0")
                    else:
                        self.rightArmJointsInPosition[0] = True
                        aux = [-50, -50, 40, 70, 0, 0]
        
                if(self.rightArmJointsInPosition[0] and not self.rightArmJointsInPosition[3]):
                    if not self.checkJointsPosition(self.aux, self.rightArmIEncoders, self.numRightArmJoints):
                        print("State 2: Moving right arm joint 3")
                    else:
                        self.rightArmJointsInPosition[3] = True
                        self.aux = self.initRightArmJointsPosition
                    
                if (self.rightArmJointsInPosition[3]):
                    if not self.checkJointsPosition(self.initRightArmJointsPosition, self.rightArmIEncoders, self.numRightArmJoints):
                        print("State 2: Moving Right arm joints 4 and 5.")
                    else:
                        print("State 2: Right arm joints are in the init position")
                        self.state = 3
                for joint in range(self.numRightArmJoints):
                    self.rightArmIPositionControl.positionMove(joint, self.aux[joint])

        if self.state == 3:  # Put the left arm joints in the init state
            for joint in range(self.numLeftArmJoints):
                self.leftArmIPositionControl.positionMove(
                    joint, self.initLeftArmJointsPosition[joint])
            if (self.checkJointsPosition(self.initLeftArmJointsPosition, self.leftArmIEncoders, self.numLeftArmJoints)):
                print("State 3: Left arm joints are in the init position.")
                self.state = 4
                self.firstInState = True
            else:
                if self.firstInState:
                    print("State 3: Continue the Left arm joints motion towards the init position.")
                    self.firstInState = False
        if self.state == 4:
            if self.firstInState:
                objectData = yarp.Bottle()
                self.objectPositionPort.read(objectData) 
                if objectData:
                    #TODO: object coordinates wrt trunk
                    self.objectData = objectData
                category = self.objectData.get(0).find('category').asString()
                self.locationTCP = self.transformFromXYZFromCameraCoordinatesToTrunk(self.objectData, category)
                
                self.rightArm = True
                print("State 4: We are ready to move near the object ", category, "in location: ", self.locationTCP)
                self.reachingDistance = 0.2
                feasible, self.orientationTCP, self.reachingPose = self.computeFeasibleOrientation(
                    self.rightArm, self.locationTCP, self.reachingDistance)
                if feasible:
                    found, self.jointsTrajectory = self.computeTrajectoryToPose(
                        self.rightArm, self.reachingPose)
                    if found:
                        self.numPointTrajectory = 0
                        self.followJointsTrajectory(
                            self.rightArm, self.jointsTrajectory)
                        self.firstInState = False
                    else:
                        print("Solution NOT found")
                        self.state = -1
                else:
                    print("solution not found")
                    self.state = -1
            else:
                print("State 4: Following path to the reaching pose.")
                # Finished following the joints trajectory
                if(self.followJointsTrajectory(self.rightArm, self.jointsTrajectory)):
                    self.state = 5
                    self.firstInState = True
        if self.state == 5:
            print("State 5: Move to towards object.")
            while self.d <= self.reachingDistance:
                print(self.d)
                self.d = self.d + self.reachingDistance/20.0

                frame_reaching_base = self.vectorToFrame(self.reachingPose)

                frame_aux_reaching = PyKDL.Frame()
                frame_aux_reaching.p.z(self.d)

                frame_aux_base = frame_reaching_base*frame_aux_reaching

                aux_pose = self.frameToVector(frame_aux_base)
                x_vector = yarp.DVector(6)
                for i in range(len(x_vector)):
                    x_vector[i] = aux_pose[i]

                print(x_vector[0], x_vector[1], x_vector[2], x_vector[3], x_vector[4], x_vector[5])
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
                print(current_Q)

                if self.rightArm:
                    if(self.trunkRightArmICartesianSolver.invKin(x_vector, current_Q, desire_Q)):
                        for joint in range(0, self.numRightArmJoints, 1):
                            self.rightArmIPositionControl.positionMove(joint, desire_Q[joint+2])
                        for joint in range(self.numTrunkJoints):
                            self.trunkIPositionControl.positionMove(joint, desire_Q[joint])
                        if self.d == 0:
                            yarp.delay(5.0)
                        else:
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
            self.state = 6
            self.firstInState = True
        if self.state == 6:
            if self.firstInState:
                print("State 6: TCP in object position")
                if self.rightArm:
                    self.rightHandIPositionControl.positionMove(0, -120)
                    print("State 6: Wait 8 seconds...")
                    yarp.delay(8.0)
                    print("State 6: Lets move up")
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
                    self.firstInState = False
                else:
                    self.leftHandIPositionControl.positionMove(0, -100)
                    print("State 6: Wait 8 seconds...")
                    yarp.delay(8.0)
                    print("State 6: Lets move up")
                    self.leftArmIPositionControl.positionMove(2, -40)
                    self.firstInState = False
                self.d = 0
            else:
                self.d = self.d+self.up_distance/40.0
                if self.d > self.up_distance:
                    self.firstInState = True
                    self.state = 7
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
                        print(current_Q)
                        
                        if(self.trunkRightArmICartesianSolver.invKin(x_vector, current_Q, desire_Q)):
                            for joint in range(0, self.numRightArmJoints, 1):
                                self.rightArmIPositionControl.positionMove(joint, desire_Q[joint+2])
                            for joint in range(self.numTrunkJoints):
                                self.trunkIPositionControl.positionMove(joint, desire_Q[joint])
                        
                    else:
                        armCurrentQ = yarp.DVector(self.numLeftArmJoints)
                        self.LeftArmIEncoders.getEncoders(armCurrentQ)
                        for j in range(0, self.numLeftArmJoints):
                            current_Q[j+2] = armCurrentQ[j]
                        print(current_Q)
                        
                        if(self.trunkLeftArmICartesianSolver.invKin(x_vector, current_Q, desire_Q)):
                            for joint in range(0, self.numLeftArmJoints, 1):
                                self.leftArmIPositionControl.positionMove(joint, desire_Q[joint+2])
                            for joint in range(self.numTrunkJoints):
                                self.trunkIPositionControl.positionMove(joint, desire_Q[joint])
                        

                    
                    
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

    def computeTrajectoryToPose(self, rightArm, reachingPose):
        if rightArm:  # Compute the a feasible orientation for the rightArm
            print("Selected right arm.")
        else:
            print("Selected left arm.")
            
        cmd = yarp.Bottle()
        cmd.addString("Compute trajectory")
        goal = cmd.addList()
        goal.addDouble(reachingPose[0])
        goal.addDouble(reachingPose[1])
        goal.addDouble(reachingPose[2])
        print("Compute trajectory to reaching pose")
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

        for rotAroundAxisZ in np.linspace(-np.pi/2.0, np.pi/2.0,20):
            for rotAroundAxisY in np.linspace(-np.pi/2.0, np.pi/2.0, 20):
                for rotAroundAxisX in np.linspace(-np.pi/2.0, np.pi/2.0, 20):
                    #print(rotAroundAxisX, rotAroundAxisY, rotAroundAxisZ)
                    aux_frame_goal = PyKDL.Frame()
                    aux_frame_goal.M = frame_goal.M * PyKDL.Rotation.RotX(rotAroundAxisX)
                    aux_frame_goal.M = aux_frame_goal.M * PyKDL.Rotation.RotY(rotAroundAxisY)
                    aux_frame_goal.M = aux_frame_goal.M * PyKDL.Rotation.RotZ(rotAroundAxisZ)

                    goal_vector = self.frameToVector(aux_frame_goal)
                    cmd = yarp.Bottle()
                    cmd.addString("Check goal")
                    goal = cmd.addList()
                    goal.addDouble(locationTCP[0])
                    goal.addDouble(locationTCP[1])
                    goal.addDouble(locationTCP[2])
                    goal.addDouble(goal_vector[3])
                    goal.addDouble(goal_vector[4])
                    goal.addDouble(goal_vector[5])

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
                        cmd.addString("Check goal")
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
        self.trunkIEncoders.getEncoders(trunkCurrentQ)
        
        headCurrentQ = yarp.DVector(self.numHeadJoints)
        self.headIEncoders.getEncoders(headCurrentQ)
        
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


        # frame_object_trunk = PyKDL.Frame()
        # frame_object_trunk.p.x(objects_data.get(o).find("x").asDouble())
        # frame_object_trunk.p.y(objects_data.get(o).find("y").asDouble())
        # frame_object_trunk.p.z(objects_data.get(o).find("z").asDouble())
        
        # frame_object_camera =frame_head_trunk.Inverse()*frame_object_trunk
        # pose_object_camera = self.frameToVector(frame_object_camera)
        
        # print(pose_object_camera[0], pose_object_camera[1], pose_object_camera[2])
        
        frame_object_camera = PyKDL.Frame()
        
        frame_object_camera.p.x(objects_data.get(o).find("x").asDouble())
        frame_object_camera.p.y(objects_data.get(o).find("y").asDouble())
        frame_object_camera.p.z(objects_data.get(o).find("z").asDouble())
        
        frame_object_trunk = frame_head_trunk*frame_object_camera
        
        pose_object_trunk = self.frameToVector(frame_object_trunk)
        
        return [pose_object_trunk[0], pose_object_trunk[1], pose_object_trunk[2]]

    def followJointsTrajectory(self, rightArm, jointsTrajectory):
        print("followJointsTrajectory ", self.numPointTrajectory,
              len(self.jointsTrajectory))
        if self.numPointTrajectory < len(self.jointsTrajectory):
            jointsPosition = jointsTrajectory[self.numPointTrajectory]
            # print(bJointsPosition.get(0).asDouble(), bJointsPosition.get(1).asDouble(), bJointsPosition.get(2).asDouble(), bJointsPosition.get(3).asDouble(), bJointsPosition.get(4).asDouble(), bJointsPosition.get(5).asDouble(), bJointsPosition.get(6).asDouble(), bJointsPosition.get(7).asDouble())
            trunkJointsPosition = [jointsPosition[0], jointsPosition[1]]
            armJointsPosition = [jointsPosition[2], jointsPosition[3],
                                 jointsPosition[4], jointsPosition[5], jointsPosition[6], jointsPosition[7]]
            for joint in range(self.numTrunkJoints):
                self.trunkIPositionControl.positionMove(
                    joint, trunkJointsPosition[joint])
            if rightArm:
                for joint in range(self.numRightArmJoints):
                    self.rightArmIPositionControl.positionMove(
                        joint, armJointsPosition[joint])
                if (self.checkJointsPosition(trunkJointsPosition, self.trunkIEncoders, self.numTrunkJoints) and self.checkJointsPosition(armJointsPosition, self.rightArmIEncoders, self.numRightArmJoints)):
                    self.numPointTrajectory = self.numPointTrajectory + 1
            else:
                for joint in range(self.numLeftArmJoints):
                    self.leftArmIPositionControl.positionMove(joint, armJointsPosition[joint])
                if (self.checkJointsPosition(trunkJointsPosition, self.trunkIEncoders, self.numTrunkJoints) and self.checkJointsPosition(armJointsPosition, self.leftArmIEncoders, self.numLeftArmJoints)):
                    self.numPointTrajectory = self.numPointTrajectory + 1
            return False
        else:
            return True


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
