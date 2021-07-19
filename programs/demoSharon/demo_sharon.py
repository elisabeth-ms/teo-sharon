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

        
        # Left Arm device
        self.leftArmOptions = yarp.Property()
        self.leftArmDevice = None
        self.leftArmIEncoders = None
        self.numLeftArmJoints = 0
        self.leftArmIPositionControl = None
        
        #Trunk device
        self.trunkOptions = yarp.Property()
        self.trunkDevice = None
        self.trunkIEncoders = None
        self.numTrunkJoints = 0
        self.trunkIPositionControl = None
        
        #Trajectory Generation client
        self.rpcClientTrajectoryGeneration = yarp.RpcClient()


        # State
        self.state = 0
        self.firstInState = True # For printing messages just once
        self.jointsPositionError = 0.05
        
        # Init Joints position for grasping
        self.initTrunkJointsPosition = [0,10]
        self.initRightArmJointsPosition = [-30, -50, 40, -70, 30, -20]
        self.initLeftArmJointsPosition = [-30, 50, -40, -70, -30, -20]
        
        # Object category and position port
        self.objectPositionPort = yarp.Port()
        self.objectData = yarp.Bottle()
        
        self.category = None
        self.locationTCP = []
        self.rightArm  = True
        self.orientationTCP = []
        self.jointsTrajectory = []
        self.numPointTrajectory = 0
        self.d = 0
        self.reachingDistance = 0
        
        
        
        self.trunkRightArmSolverOptions = yarp.Property()
        self.trunkRightArmSolverDevice = None
        
    
    def configure(self, rf):
        print("Configuring...")
        
        self.objectPositionPort.open("/demoSharon/object:i")
        yarp.Network.connect("/matching/object:o", "/demoSharon/object:i")

        
        # Open rightArm device 
        self.rightArmOptions.put('device','remote_controlboard')
        self.rightArmOptions.put('remote',robot+'/rightArm')
        self.rightArmOptions.put('local', robot+'/rightArm')
        
        self.rightArmDevice = yarp.PolyDriver(self.rightArmOptions) 
        self.rightArmDevice.open(self.rightArmOptions)
        if not self.rightArmDevice.isValid():
            print('Cannot open rightArm device!')
            raise SystemExit

        
        # Open leftArm device 
        self.leftArmOptions.put('device','remote_controlboard')
        self.leftArmOptions.put('remote',robot+'/leftArm')
        self.leftArmOptions.put('local', robot+'/leftArm')
        
        self.leftArmDevice = yarp.PolyDriver(self.leftArmOptions) 
        self.leftArmDevice.open(self.leftArmOptions)
        if not self.leftArmDevice.isValid():
            print('Cannot open leftArm device!')
            raise SystemExit
    
    
        #Open trunk device    
        self.trunkOptions.put('device','remote_controlboard')
        self.trunkOptions.put('remote',robot+'/trunk')
        self.trunkOptions.put('local',robot+'/trunk')

        self.trunkDevice = yarp.PolyDriver(self.trunkOptions)
        self.trunkDevice.open(self.trunkOptions)
        if not self.trunkDevice.isValid():
            print('Cannot open trunk device!')
            raise SystemExit

        # Interface with right arm encoders
        self.rightArmIEncoders = self.rightArmDevice.viewIEncoders()

        if self.rightArmIEncoders ==[]:
            print("Right arm Encoder interface NOT available.")
            raise SystemExit
        else:
            print("Right arm Encoder interface available.")

        self.numRightArmJoints = self.rightArmIEncoders.getAxes()


        # Right arm position control interface
        self.rightArmIPositionControl = self.rightArmDevice.viewIPositionControl()

        if self.rightArmIPositionControl == []:
            print("Right arm position control interface NOT available")
            raise SystemExit
        else:
            print("Right arm position control interface available.")

        # Interface with left arm encoders
        self.leftArmIEncoders = self.leftArmDevice.viewIEncoders()

        if self.leftArmIEncoders ==[]:
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
            
        
        # Interface with trunk encoders
        self.trunkIEncoders = self.trunkDevice.viewIEncoders()

        if self.trunkIEncoders ==[]:
            print("Trunk Encoder interface NOT available.")
            raise SystemExit
        else:
            print("Trunk Encoder interface available.")

        self.numTrunkJoints = self.trunkIEncoders.getAxes()


        # Left arm position control interface
        self.trunkIPositionControl = self.trunkDevice.viewIPositionControl()

        if self.trunkIPositionControl == []:
            print("Trunk position control interface NOT available")
            raise SystemExit
        else:
            print("Trunk position control interface available.")
        
        
        trunkRightKinPath = rf.findFileByName("teo-trunk-rightArm-fetch.ini")
        self.trunkRightArmSolverOptions.fromConfigFile(trunkRightKinPath)
        self.trunkRightArmSolverOptions.put("device", "KdlSolver")
        self.trunkRightArmSolverOptions.put("ik","nrjl")

        self.trunkRightArmSolverOptions.fromString("(mins (-20 -10.8 -98.1 -75.5 -80.1 -99.6 -80.4 -115.4))", False)
        self.trunkRightArmSolverOptions.fromString("(maxs (10 10.1 106 22.4 57 98.4 99.6 44.7))", False)
        print("mins")
        print(self.trunkRightArmSolverOptions.find("mins").toString())
        print("maxs")
        print(self.trunkRightArmSolverOptions.find("maxs").toString())
        self.trunkRightArmSolverDevice = yarp.PolyDriver(self.trunkRightArmSolverOptions)  # calls open -> connects
        self.trunkRightArmSolverDevice.open(self.trunkRightArmSolverOptions)

        if self.trunkRightArmSolverDevice == []:
            print("trunk right arm solver device interface NOT available")
        else:
            print("trunk right arm solver device interface available.")
            
        
        
        self.trunkRightArmICartesianSolver = kinematics_dynamics.viewICartesianSolver(self.trunkRightArmSolverDevice)
        if self.trunkRightArmICartesianSolver == []:
            print("Right arm cartesian solver interface NOT available")
        else:
            print("Right arm cartesian solver interface available.")
        
        
        
        # Open rpc client for trajectoryGeneration
        self.rpcClientTrajectoryGeneration.open('/trajectoryGeneration/rpc:c')
        yarp.Network.connect("/trajectoryGeneration/rpc:c", "/trajectoryGeneration/rpc:s")

        
        print("demo_sharon Module initialized...")
        return True
    
    def interruptModel(self):
        print("Stopping the module")
    
    
    def close(self):
        return True

    def getPeriod(self):
        return 0.05
    


    
    def updateModule(self):
        
        objectData = yarp.Bottle()
        self.objectPositionPort.read(objectData)
        if objectData:
            self.objectData = objectData
        
        if self.state == 0: # Put the trunk joints in the init state
            for joint in range(self.numTrunkJoints):
                self.trunkIPositionControl.positionMove(joint, self.initTrunkJointsPosition[joint])
            if (self.checkJointsPosition(self.initTrunkJointsPosition, self.trunkIEncoders, self.numTrunkJoints)):
                print("State 0: Trunks joints are in the init position.")
                self.state = 1
                self.firstInState = True
            else:
                if self.firstInState:
                    print("State 0: Continue the Trunk joints motion towards the init position.")
                    self.firstInState = False
        if self.state == 1: # Put the right arm joints in the init state
            for joint in range(self.numRightArmJoints):
                self.rightArmIPositionControl.positionMove(joint, self.initRightArmJointsPosition[joint])
            if (self.checkJointsPosition(self.initRightArmJointsPosition, self.rightArmIEncoders, self.numRightArmJoints)):
                print("State 1: Right arm joints are in the init position.")
                self.state = 2
            else:
                if self.firstInState:
                    print("State 1: Continue the Right arm joints motion towards the init position.")
                    self.firstInState = False
        if self.state == 2: # Put the left arm joints in the init state
            for joint in range(self.numLeftArmJoints):
                self.leftArmIPositionControl.positionMove(joint, self.initLeftArmJointsPosition[joint])
            if (self.checkJointsPosition(self.initLeftArmJointsPosition, self.leftArmIEncoders, self.numLeftArmJoints)):
                print("State 2: Left arm joints are in the init position.")
                self.state = 3
                self.firstInState = True
            else:
                if self.firstInState:
                    print("State 2: Continue the Left arm joints motion towards the init position.")
                    self.firstInState = False
        if self.state == 3:
            if self.firstInState:
                self.category = self.objectData.get(0).find('category').asString()
                self.locationTCP = [self.objectData.get(0).find("x").asDouble(), self.objectData.get(0).find("y").asDouble(), self.objectData.get(0).find("z").asDouble()]
                self.rightArm  = True
                print("State 3: We are ready to move near the object ", self.category, "in location: ", self.locationTCP)
                self.reachingDistance = 0.08
                feasible, self.orientationTCP, self.reachingPose = self.computeFeasibleOrientation(self.rightArm, self.locationTCP, self.reachingDistance)
                if feasible:
                    found,self.jointsTrajectory = self.computeTrajectoryToPose(self.rightArm, self.reachingPose)
                    if found:
                        self.numPointTrajectory = 0
                        self.followJointsTrajectory(self.rightArm, self.jointsTrajectory)
                        self.firstInState = False
            else:
                print("State 3: Following path to the reaching pose.")
                if(self.followJointsTrajectory(self.rightArm, self.jointsTrajectory)): # Finished following the joints trajectory
                    self.state = 4
                    self.firstInState = True
        if self.state == 4:
            print("State 4: Move to towards object.")
            while self.d <= self.reachingDistance:
                print(self.d)
                self.d = self.d + self.reachingDistance/10

                frame_reaching_base = self.vectorToFrame(self.reachingPose)
                
                frame_aux_reaching = PyKDL.Frame()
                frame_aux_reaching.p.z(self.d)
                
                frame_aux_base = frame_reaching_base*frame_aux_reaching
                
                aux_pose = self.frameToVector(frame_aux_base)
                x_vector = yarp.DVector(6)
                for i in range(len(x_vector)):
                    x_vector[i] = aux_pose[i]
                trunkCurrentQ = yarp.DVector(self.numTrunkJoints)
                self.trunkIEncoders.getEncoders(trunkCurrentQ)
                
                armCurrentQ = yarp.DVector(self.numRightArmJoints)
                self.rightArmIEncoders.getEncoders(armCurrentQ)
                
                current_Q = yarp.DVector(self.numRightArmJoints+self.numTrunkJoints)
                desire_Q = yarp.DVector(self.numRightArmJoints+self.numTrunkJoints)

                for j in range(0, self.numRightArmJoints):
                    current_Q[j+2] = armCurrentQ[j]
                for j in range(self.numTrunkJoints):
                    current_Q[j] = trunkCurrentQ[j]
                print(current_Q)
                
                if(self.trunkRightArmICartesianSolver.invKin(x_vector, current_Q, desire_Q)):
                    for joint in range(0,self.numRightArmJoints,1):
                        self.rightArmIPositionControl.positionMove(joint,desire_Q[joint+2])
                    for joint in range(self.numTrunkJoints):
                #       print(numTrunkJoints, joint, desireQ[joint])
                        self.trunkIPositionControl.positionMove(joint, desire_Q[joint])
                    yarp.delay(1.0)
            self.state = 5
                
                
        
                
                # if feasible:
                #     print("Final TCP feasible pose found")
                #     self.reachingPose = self.computeReachingPose(self.rightArm, self.locationTCP)
                
                
                
        #         if feasible:
        #             self.firstInState = False
        #             print("State 3: Lets go to the reaching pose.")
        #             # self.reachingPose = computeReachingPose(self.orientationTCP, self.locationTCP)
        #             self.numPointTrajectory = 0
        #             self.followJointsTrajectory(self.rightArm, self.jointsTrajectory)
        #         else:
        #             print("Imposible to go to the location of the object. TODO!!")
        #     else:
        #         if(self.followJointsTrajectory(self.rightArm, self.jointsTrajectory)): # Finished following the joints trajectory
        #             self.state = 4
        #             self.firstInState = True
        # if self.state == 4:
        #     if self.firstInState:
        #         print("State 4: TCP in reaching pose")
        #         self.firstInState = False
                    
                
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
        if rightArm: # Compute the a feasible orientation for the rightArm
            print("Selected right arm.")
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
            self.rpcClientTrajectoryGeneration.write(cmd,response)
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
        else:
            print("Selected left arm. TODO!")
            
        return False, jointsTrajectory
            
    def computeFeasibleOrientation(self, rightArm, locationTCP, reachingDistance):
        if rightArm: # Compute the a feasible orientation for the rightArm
            print("Selected right arm.")
            cmd = yarp.Bottle()
            cmd.addString("Check goal")
            goal = cmd.addList()
            goal.addDouble(locationTCP[0])
            goal.addDouble(locationTCP[1])
            goal.addDouble(locationTCP[2])
            print("TODO: Compute Feasible orientation. Right now using a fix orientation")
            goal.addDouble(-1.60316)
            goal.addDouble(1.15825)
            goal.addDouble(-0.572439)
            
            orientationTCP = [-1.60316, 1.15825, -0.572439]
            response = yarp.Bottle()
            jointsTrajectory = []
            self.rpcClientTrajectoryGeneration.write(cmd,response)
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
                self.rpcClientTrajectoryGeneration.write(cmd,response)
                if response.get(0).asString() == 'Valid':
                    print("Valid reaching position")
                else:
                    print("Not valid reaching position")
                
                
                return True, orientationTCP, reaching_pose
            else:
                return False, orientationTCP, reaching_pose
            
            
        else:
            print("Selected left arm. TODO!")
            
        orientationTCP = []
        return False, orientationTCP
    # def computeFeasibleOrientation(self, rightArm, locationTCP):
    #     if rightArm: # Compute the a feasible orientation for the rightArm
    #         print("Selected right arm.")
    #         cmd = yarp.Bottle()
    #         cmd.addDouble(locationTCP[0])
    #         cmd.addDouble(locationTCP[1])
    #         cmd.addDouble(locationTCP[2])
    #         print("TODO: Compute Feasible orientition. Right now using a fix orientation")
    #         cmd.addDouble(-1.60316)
    #         cmd.addDouble(1.15825)
    #         cmd.addDouble(-0.572439)
    #         orientationTCP = [-1.60316, 1.15825, -0.572439]
    #         response = yarp.Bottle()
    #         jointsTrajectory = []
    #         self.rpcClientTrajectoryGeneration.write(cmd,response)
    #         if response.get(0).asString() == 'Goal state NOT valid':
    #             print(response.get(0).asString())
    #         elif response.get(0).asString() == 'Solution Found':
    #             bJointsTrajectory = response.get(1).asList()
    #             print('Solution Found with ', bJointsTrajectory.size(), "points.")
    #             for i in range(bJointsTrajectory.size()):
    #                 bJointsPosition = bJointsTrajectory.get(i).asList()
    #                 jointsPosition = []
    #                 for i in range(bJointsPosition.size()):
    #                     jointsPosition.append(bJointsPosition.get(i).asDouble())
    #                 jointsTrajectory.append(jointsPosition)
    #             print("JointsTrajectory")
    #             return True, orientationTCP, jointsTrajectory
    #     else:
    #         print("Selected left arm. TODO!")
            
    #     orientationTCP = []
    #     return False, orientationTCP, jointsTrajectory

    def checkJointsPosition(self, desiredJointsPosition, iEncoders, numJoints):
        currentQ = yarp.DVector(numJoints)
        iEncoders.getEncoders(currentQ)
        for joint in range(numJoints):
            if abs(currentQ[joint]-desiredJointsPosition[joint])>self.jointsPositionError:
                 return False
        return True

    def followJointsTrajectory(self, rightArm, jointsTrajectory):
        print("followJointsTrajectory ", self.numPointTrajectory, len(self.jointsTrajectory))
        if self.numPointTrajectory < len(self.jointsTrajectory):
            jointsPosition = jointsTrajectory[self.numPointTrajectory]
            # print(bJointsPosition.get(0).asDouble(), bJointsPosition.get(1).asDouble(), bJointsPosition.get(2).asDouble(), bJointsPosition.get(3).asDouble(), bJointsPosition.get(4).asDouble(), bJointsPosition.get(5).asDouble(), bJointsPosition.get(6).asDouble(), bJointsPosition.get(7).asDouble())
            trunkJointsPosition = [jointsPosition[0], jointsPosition[1]]
            armJointsPosition = [jointsPosition[2], jointsPosition[3], jointsPosition[4], jointsPosition[5], jointsPosition[6], jointsPosition[7]]
            for joint in range(self.numTrunkJoints):
                self.trunkIPositionControl.positionMove(joint, trunkJointsPosition[joint])
            if rightArm:
                for joint in range(self.numRightArmJoints):
                    self.rightArmIPositionControl.positionMove(joint, armJointsPosition[joint])
                if (self.checkJointsPosition(trunkJointsPosition, self.trunkIEncoders, self.numTrunkJoints) and self.checkJointsPosition(armJointsPosition, self.rightArmIEncoders, self.numRightArmJoints)):
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