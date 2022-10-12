import yarp
import PyKDL
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate
from matplotlib.ticker import MaxNLocator
import time
import kinematics_dynamics


VOCAB_OK = yarp.createVocab32('o', 'k')
VOCAB_FAIL = yarp.createVocab32('f', 'a', 'i', 'l')
robot = '/teoSim'
prefix = '/demoSharon'



class TestTrajectoryGeneration():
    def __init__(self):

        # Right Arm device
        self.rightArmOptions = yarp.Property()
        self.rightArmDevice = None
        self.rightArmIEncoders = None
        self.numRightArmJoints = 0
        self.rightArmIPositionControl = None
        self.rightArmIPositionControl = None
        self.rightArmIControlMode = None
        self.rightArmIPositionDirect = None
        self.rightArmIRemoteVariables = False
        
        
        # Trunk device
        self.trunkOptions = yarp.Property()
        self.trunkDevice = None
        self.trunkIEncoders = None
        self.numTrunkJoints = 2
        self.trunkIPositionControl = None
        self.trunkIControlMode = None
        self.trunkIPositionDirect = None
        self.trunkIRemoteVariables = False
        
        self.trunkRightArmSolverOptions = yarp.Property()
        self.trunkRightArmSolverDevice = None

        # Trajectory Generation client
        self.rpcClientTrajectoryGenerationRight = yarp.RpcClient()

        # Get Grasping poses client
        self.rpcClientGetGraspingPoses = yarp.RpcClient()

        self.rpcClientGetGraspingPoses.open("/testTrajectoryGeneration/getGraspingPoses/rpc:c")
        yarp.Network.connect("/testTrajectoryGeneration/getGraspingPoses/rpc:c",
                             "/getGraspingPoses/rpc:s")

        self.rpcClientTrajectoryGenerationRight.open('/trajectoryGeneration/trunkAndRightArm/rpc:c')
        yarp.Network.connect("/trajectoryGeneration/trunkAndRightArm/rpc:c",
                             "/trajectoryGeneration/trunkAndRightArm/rpc:s")

        self.bGraspingPoses = yarp.Bottle()
        self.rightArm = True
        self.reachingDistance = 0.09
        self.numPointTrajectory = 0
        self.numTrunkJoints = 2
        self.numRightArmJoints = 6
        self.jointsPositionError = 2.5
        self.jointsDistandeFromBounds = 2

        
        self.smoothJointsTrajectoryTrunk = []
        self.smoothJointsTrajectoryRightArm = []
        
        
        self.rightArmOptions.put('device', 'remote_controlboard')
        self.rightArmOptions.put('remote', robot+'/rightArm')
        self.rightArmOptions.put('local', prefix+robot+'/rightArm')
        # self.rightArmOptions.put('writeStrict', 'on')

        self.rightArmDevice = yarp.PolyDriver(self.rightArmOptions)
        self.rightArmDevice.open(self.rightArmOptions)
        if not self.rightArmDevice.isValid():
            print('Cannot open rightArm device!')
            raise SystemExit
        
        # Open trunk device
        self.trunkOptions.put('device', 'remote_controlboard')
        self.trunkOptions.put('remote', robot+'/trunk')
        self.trunkOptions.put('local', prefix+robot+'/trunk')
        # self.trunkOptions.put('writeStrict', 'on')

        self.trunkDevice = yarp.PolyDriver(self.trunkOptions)
        self.trunkDevice.open(self.trunkOptions)
        if not self.trunkDevice.isValid():
            print('Cannot open trunk device!')
            raise SystemExit
        
        self.rightArmIEncoders = self.rightArmDevice.viewIEncoders()

        if self.rightArmIEncoders == []:
            print("Right arm Encoder interface NOT available.")
            raise SystemExit
        else:
            print("Right arm Encoder interface available.")

        self.numRightArmJoints = self.rightArmIEncoders.getAxes()

        self.rightArmIPositionDirect = self.rightArmDevice.viewIPositionDirect()
        if self.rightArmIPositionDirect == []:
            print("Right arm position direct interface NOT available.")
            raise SystemExit
        else:
            print("Right arm position direct interface available")

        # rightArm position control interface
        self.rightArmIPositionControl = self.rightArmDevice.viewIPositionControl()

        if self.rightArmIPositionControl == []:
            print("rightArm position control interface NOT available")
            raise SystemExit
        else:
            print("rightArm position control interface available.")

        self.rightArmIControlMode = self.rightArmDevice.viewIControlMode()
        if self.rightArmIControlMode == []:
            print("Right arm control mode interface NOT available.")
            raise SystemExit
        else:
            print("Right arm control mode interface available.")

        armCurrentQ = yarp.DVector(self.numRightArmJoints)
        self.rightArmIEncoders.getEncoders(armCurrentQ)
        for j in range(0, self.numRightArmJoints):
            print(armCurrentQ[j])



        self.rightArmIRemoteVariables = self.rightArmDevice.viewIRemoteVariables()
        if self.rightArmIRemoteVariables == []:
            print("Right arm remote variables interface NOT available.")
            raise SystemExit
        else:
            print("Right arm remote variables interface available.")
            
        rightArmModes = yarp.IVector(6, yarp.VOCAB_CM_POSITION_DIRECT)
        if not self.rightArmIControlMode.setControlModes(rightArmModes):
            print("Unable to set right arm  to position direct mode.")
            raise SystemExit
        else:
            print("Right arm set to position direct mode.")
            
            
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

        self.trunkIPositionDirect = self.trunkDevice.viewIPositionDirect()
        if self.trunkIPositionDirect == []:
            print("Trunk position direct interface NOT available.")
            raise SystemExit
        else:
            print("Trunk position direct interface available")

        self.trunkIControlMode = self.trunkDevice.viewIControlMode()
        if self.trunkIControlMode == []:
            print("Trunk control mode interface NOT available.")
            raise SystemExit
        else:
            print("Trunk control mode interface available.")

        self.trunkIRemoteVariables = self.trunkDevice.viewIRemoteVariables()
        if self.trunkIRemoteVariables == []:
            print("Trunk remote variables interface NOT available.")
            raise SystemExit
        else:
            print("Trunk remote variables interface available.")
    
        trunkModes = yarp.IVector(2, yarp.VOCAB_CM_POSITION_DIRECT)
        if not self.trunkIControlMode.setControlModes(trunkModes):
            print("Unable to set trunk to position direct mode.")
            raise SystemExit
        else:
            print("Trunk set to position direct mode.")


        # Open rightHand device
        self.rightHandOptions = yarp.Property()

        self.rightHandOptions.put('device', 'remote_controlboard')
        self.rightHandOptions.put('remote', robot+'/rightHand')
        self.rightHandOptions.put('local', robot+'/rightHand')

        self.rightHandDevice = yarp.PolyDriver(self.rightHandOptions)
        self.rightHandDevice.open(self.rightHandOptions)

        if not self.rightHandDevice.isValid():
            print('Cannot open rightHand device!')
            raise SystemExit

        self.rightHandIPositionControl = self.rightHandDevice.viewIPositionControl()

        if self.rightHandIPositionControl == []:
            print("Right hand position control interface NOT available")
            raise SystemExit
        else:
            print("Right hand position control interface available.")
        
        rf = yarp.ResourceFinder()
        rf.setDefaultContext("kinematics")
        trunkRightKinPath = rf.findFileByName("teo-trunk-rightArm-fetch.ini")
        print(trunkRightKinPath)
        self.trunkRightArmSolverOptions.fromConfigFile(trunkRightKinPath)
        self.trunkRightArmSolverOptions.put("device", "KdlSolver")
        self.trunkRightArmSolverOptions.put("ik", "nrjl")
        self.trunkRightArmSolverOptions.put("eps",0.005)
        self.trunkRightArmSolverOptions.put("maxIter",100000)

        self.trunkRightArmSolverOptions.fromString(
            "(mins (-30 -10.0 -98.1 -75.5 -80.1 -99.6 -80.4 -115.4))", False)
        self.trunkRightArmSolverOptions.fromString(
            "(maxs (30 16.5 106 22.4 57 98.4 99.6 44.7))", False)
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
    
    def execute(self):

    
        self.outsideBounds, goalQInsideBounds = self.checkBounds()
        if(self.outsideBounds):
                 
            rightArmModes = yarp.IVector(6, yarp.VOCAB_CM_POSITION)
            if not self.rightArmIControlMode.setControlModes(rightArmModes):
                print("Unable to set right arm  to position  mode.")
                raise SystemExit
            else:
                print("Right arm set to position  mode.")
            trunkModes = yarp.IVector(2, yarp.VOCAB_CM_POSITION)
            if not self.trunkIControlMode.setControlModes(trunkModes):
                print("Unable to set trunk to position  mode.")
                raise SystemExit
            else:
                print("Trunk set to position  mode.")
            for j in range(2):
                self.trunkIPositionControl.positionMove(j, goalQInsideBounds[j])
            for j in range(6):
                self.rightArmIPositionControl.positionMove(j, goalQInsideBounds[j+2])
        
        
        cmd = yarp.Bottle()
        response = yarp.Bottle()
        cmd.addString('rsm')
        self.rpcClientGetGraspingPoses.write(cmd, response)
        yarp.delay(1.0)


        
        cmd = yarp.Bottle()
        response = yarp.Bottle()
        cmd.addString('paus')
        self.rpcClientGetGraspingPoses.write(cmd, response)
        yarp.delay(1.0)

        cmd.clear()
        response.clear()
        cmd.addString('gsup')
        self.rpcClientTrajectoryGenerationRight.write(cmd, response)
        print(response.toString())

        cmd.clear()
        response.clear()
        cmd.addString('gsup')
        self.rpcClientGetGraspingPoses.write(cmd, response)
        print(response.toString())
        print(response.size())

        label = 0
        for j in range(response.size()):
            sup = response.get(j).asDict()
            label = sup.find("label_idx").asInt32()

        cmd.clear()
        response.clear()
        cmd.addString('ggp')
        cmd.addInt32(label)
        self.rpcClientGetGraspingPoses.write(cmd, response)
        
        self.bGraspingPoses = response
        print(response.toString())
        
        feasible, self.graspingPose, self.graspingQ, self.reachingPose, self.reachingQ = self.getFeasibleGraspingPose(self.rightArm, self.bGraspingPoses, self.reachingDistance)

        print(self.reachingPose)
        print(self.reachingQ)
        
        if feasible:
            
            print("Reaching pose: ",self.reachingPose)
                    
            found, self.jointsTrajectory = self.computeTrajectoryToJointsPosition(self.rightArm, self.reachingQ)

            if found:
                self.numPointTrajectory = 0
                        # Lets plot the trajectory
                print(len(self.jointsTrajectory))
                nPoints = 0
                if len(self.jointsTrajectory) < 100:
                    nPoints = 400
                else:
                    nPoints = 1000 
                self.smoothJointsTrajectoryTrunk, self.smoothJointsTrajectoryRightArm = self.computeSmoothJointsTrajectory(nPoints)
                print("done!")
                self.plotTrajectories(self.jointsTrajectory, self.smoothJointsTrajectoryTrunk,self.smoothJointsTrajectoryRightArm)
                
                self.followJointsTrajectory(self.rightArm, self.smoothJointsTrajectoryTrunk, self.smoothJointsTrajectoryRightArm)
        
        print("Open rightHand")
        self.rightHandIPositionControl.positionMove(0, 1200)
        yarp.delay(5.0)

        cmd.clear()
        response.clear()
        cmd.addString('rsup')
        cmd.addInt32(label)
        self.rpcClientGetGraspingPoses.write(cmd, response)
        print(response.toString())
        
        cmd.clear()
        response.clear()
        cmd.addString('gsup')
        self.rpcClientTrajectoryGenerationRight.write(cmd, response)
        print(response.toString())

        yarp.delay(5.0)

        self.moveTowardsObject()


       
        
        print("Close rightHand")
        self.rightHandIPositionControl.positionMove(0, -200)
        yarp.delay(5.0)


        self.outsideBounds, goalQInsideBounds = self.checkBounds()
        if(self.outsideBounds):
                 
            rightArmModes = yarp.IVector(6, yarp.VOCAB_CM_POSITION)
            if not self.rightArmIControlMode.setControlModes(rightArmModes):
                print("Unable to set right arm  to position  mode.")
                raise SystemExit
            else:
                print("Right arm set to position  mode.")
            trunkModes = yarp.IVector(2, yarp.VOCAB_CM_POSITION)
            if not self.trunkIControlMode.setControlModes(trunkModes):
                print("Unable to set trunk to position  mode.")
                raise SystemExit
            else:
                print("Trunk set to position  mode.")
            for j in range(2):
                self.trunkIPositionControl.positionMove(j, goalQInsideBounds[j])
            for j in range(6):
                self.rightArmIPositionControl.positionMove(j, goalQInsideBounds[j+2])
        yarp.delay(1.0)


        self.moveUpObject()
        yarp.delay(1.0)

        if (self.checkJointsPosition([0,0], self.trunkIEncoders, self.numTrunkJoints) and self.checkJointsPosition([0,0,0,0,0,0], self.rightArmIEncoders, self.numRightArmJoints)):
            return True
        else:
            print("Back to home position")
            self.reachingQ = [0,0,0,0,0,0,0,0]
            found, self.jointsTrajectory = self.computeTrajectoryToJointsPosition(self.rightArm, self.reachingQ)

            if found:
                self.numPointTrajectory = 0
                            # Lets plot the trajectory
                print(len(self.jointsTrajectory))
                nPoints = 0
                if len(self.jointsTrajectory) < 100:
                    nPoints = 400
                else:
                    nPoints = 1000 
                self.smoothJointsTrajectoryTrunk, self.smoothJointsTrajectoryRightArm = self.computeSmoothJointsTrajectory(nPoints)
                print("done!")
                self.plotTrajectories(self.jointsTrajectory, self.smoothJointsTrajectoryTrunk,self.smoothJointsTrajectoryRightArm)
                    
                self.followJointsTrajectory(self.rightArm, self.smoothJointsTrajectoryTrunk, self.smoothJointsTrajectoryRightArm)

    def checkBounds(self):
        goalQinsideBounds = yarp.DVector(self.numRightArmJoints+self.numTrunkJoints)
        outsideBounds = False
        print("Aproach to object")
        trunkCurrentQ = yarp.DVector(self.numTrunkJoints)
        if (not self.trunkIEncoders.getEncoders(trunkCurrentQ)):
            print("Unable to get trunk encoders position")
            raise SystemExit

        for j in range(self.numTrunkJoints):
            if(trunkCurrentQ[j]<= self.minsTrunkAndRightArm.get(j).asFloat64()):
                goalQinsideBounds[j] = self.minsTrunkAndRightArm.get(j).asFloat64()+self.jointsDistandeFromBounds
                outsideBounds = True
            elif(trunkCurrentQ[j]>= self.maxsTrunkAndRightArm.get(j).asFloat64()):
                goalQinsideBounds[j] = self.maxsTrunkAndRightArm.get(j).asFloat64()-self.jointsDistandeFromBounds
                outsideBounds = True
            else:
                goalQinsideBounds[j] = trunkCurrentQ[j]

        armCurrentQ = yarp.DVector(self.numRightArmJoints)
        if not self.rightArmIEncoders.getEncoders(armCurrentQ):
            print("Unable to get arm encoders position")
            raise SystemExit
        
        for j in range(self.numRightArmJoints):
            if(armCurrentQ[j]<= self.minsTrunkAndRightArm.get(j+2).asFloat64()):
                goalQinsideBounds[j+2] = self.minsTrunkAndRightArm.get(j+2).asFloat64()+self.jointsDistandeFromBounds
                outsideBounds = True
            elif(armCurrentQ[j]>= self.maxsTrunkAndRightArm.get(j+2).asFloat64()):
                goalQinsideBounds[j+2] = self.maxsTrunkAndRightArm.get(j+2).asFloat64()-self.jointsDistandeFromBounds
                outsideBounds = True
            else:
                goalQinsideBounds[j+2] = armCurrentQ[j]
        
        return outsideBounds, goalQinsideBounds


        
    def plotTrajectories(self, jointsTrajectory, smoothJointsTrajectoryTrunk, smoothJointsTrajectoryRightArm):
        arr = np.asarray(jointsTrajectory)
        t = np.linspace(0,1.0, len(jointsTrajectory))

        arr_smooth_trunk = np.asarray(smoothJointsTrajectoryTrunk)
        t_smooth_trunk = np.linspace(0,1.0, len(smoothJointsTrajectoryTrunk))


        arr_smooth_rightArm = np.asarray(smoothJointsTrajectoryRightArm)
        t_smooth_rightArm =  np.linspace(0,1.0, len(smoothJointsTrajectoryRightArm))

        fig, ax = plt.subplots(8, sharex=True)
        plt.subplots_adjust(left=0.15, right=0.95, top=0.95, bottom=0.05)
        # set axes labels
        # ax[0].set_xlabel("num point")
        ax[0].set_ylabel("Axial Trunk [deg]", rotation=0, ha="right")
        ax[1].set_ylabel("Frontal Trunk [deg]",  rotation=0, ha="right")
        
        if self.rightArm:
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
        
        ax[7].xaxis.set_ticks(np.linspace(0, 1.0, 100))

        handles, labels = ax[7].get_legend_handles_labels()
        fig.legend(handles, labels, loc='upper right')

        plt.show()
        # plt.figure().clear()
        # plt.close()
    
    def followJointsTrajectory(self, rightArm, jointsTrajectoryTrunk, jointsTrajectoryRightArm):

        self.numPointTrajectory = 0
        print("followJointsTrajectory ", self.numPointTrajectory, len(jointsTrajectoryTrunk))
        period = 50
        trunkModes = yarp.IVector(2,yarp.VOCAB_CM_POSITION_DIRECT)
        if not self.trunkIControlMode.setControlModes(trunkModes):
            print("Unable to set trunk to position direct mode.")
            raise SystemExit
        else:
            print("Trunk set to position direct mode.")
        
     
        rightArmModes = yarp.IVector(6,yarp.VOCAB_CM_POSITION_DIRECT)
        if not self.rightArmIControlMode.setControlModes(rightArmModes):
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
                self.rightArmIPositionDirect.setPosition(j, jointsTrajectoryRightArm[self.numPointTrajectory][j])
            time.sleep(period * 0.001 - ((time.time() - start) % (period * 0.001)))
            self.numPointTrajectory+=1
        return True
        
    def getFeasibleGraspingPose(self, rightArm, bGraspingPoses, reachingDistance):
        graspingPose = []
        reaching_pose = []
        grasping_q = []
        reaching_q = []
        print(bGraspingPoses.size())
        if rightArm:  # Compute the a feasible orientation for the rightArm
            print("Selected right arm.")
        else:
            print("Selected left arm")
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

            frame_reaching_end = PyKDL.Frame()
            frame_reaching_end.p.z(-reachingDistance)

            frame_reaching_base = frame_end_base*frame_reaching_end

            reaching_pose = self.frameToVector(frame_reaching_base)
                
            print("Let's check the reaching pose")
            
            cmd.addVocab32("chgp")
            goal = cmd.addList()
            for i in range(len(reaching_pose)):
               goal.addFloat64(reaching_pose[i])
                    
            
            if rightArm:
                self.rpcClientTrajectoryGenerationRight.write(cmd, response)
            else:
                self.rpcClientTrajectoryGenerationLeft.write(cmd, response)

            if response.get(0).asVocab32() == VOCAB_OK:
                print("Valid reaching position")
                aux_q = response.get(1).asList()
                for i in range(self.numTrunkJoints+self.numRightArmJoints):
                    reaching_q.append(aux_q.get(i).asFloat32())
                   
                return True, graspingPose,grasping_q, reaching_pose, reaching_q
            else:
                print("Not valid reaching position")    
        return False, graspingPose,grasping_q, reaching_pose, reaching_q
    
    def computeTrajectoryToJointsPosition(self, rightArm, reachingJointsPosition):
        if rightArm:
            print("Selected right arm.")
        else:
            print("Selected left arm.")

        cmd = yarp.Bottle()
        cmd.addString('cpgj')
        goal = cmd.addList()

        for jointPosition in reachingJointsPosition:
            goal.addFloat64(jointPosition)
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
                    jointsPosition.append(bJointsPosition.get(i).asFloat64())
                jointsTrajectory.append(jointsPosition)
            print("JointsTrajectory")
            return True, jointsTrajectory


    def computeSmoothJointsTrajectory(self, nPoints):
        smoothJointsTrajectoryTrunk = []
        smoothJointsTrajectoryRightArm = []
        arr = np.asarray(self.jointsTrajectory)
        t = range(0, len(self.jointsTrajectory))
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
            trunkJointsPosition = [q0new[i], q1new[i]]
            jointsPosition = [q2new[i],
                q3new[i], q4new[i], q5new[i], q6new[i], q7new[i]]
            smoothJointsTrajectoryTrunk.append(trunkJointsPosition)
            smoothJointsTrajectoryRightArm.append(jointsPosition)
        return smoothJointsTrajectoryTrunk, smoothJointsTrajectoryRightArm
    
    def moveTowardsObject(self):
        print("Aproach to object")
        trunkCurrentQ = yarp.DVector(self.numTrunkJoints)
        if (not self.trunkIEncoders.getEncoders(trunkCurrentQ)):
            print("Unable to get trunk encoders position")
            raise SystemExit

        armCurrentQ = yarp.DVector(self.numRightArmJoints)
        if not self.rightArmIEncoders.getEncoders(armCurrentQ):
            print("Unable to get arm encoders position")
            raise SystemExit

        current_Q = yarp.DVector(self.numRightArmJoints+self.numTrunkJoints)
        desire_Q = yarp.DVector(self.numRightArmJoints+self.numTrunkJoints)
        aproachingQs = []
        goal_q = []
                        
        for j in range(self.numTrunkJoints):
            current_Q[j] = trunkCurrentQ[j]
            goal_q.append(trunkCurrentQ[j])
        for j in range(0, self.numRightArmJoints):
            current_Q[j+2] = armCurrentQ[j]
            goal_q.append(armCurrentQ[j])
       
        self.reachingPose = yarp.DVector(6)
        self.trunkRightArmICartesianSolver.fwdKin(current_Q, self.reachingPose)
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
           
            if(self.trunkRightArmICartesianSolver.invKin(x_vector, current_Q, desire_Q)):
                goal_q = []
                for i in range(0,self.numTrunkJoints+self.numRightArmJoints):
                    goal_q.append(desire_Q[i])
                aproachingQs.append(desire_Q)
                print("current_Q: ", current_Q)
                print("desire_Q: ", current_Q)
                print("distance: ", self.d)
                current_Q = desire_Q
                print("Aproaching Qs: ", aproachingQs)
        # Lets smooth the trajectory
        self.jointsTrajectory = aproachingQs
        if(len(self.jointsTrajectory)>20):
            self.smoothJointsTrajectoryTrunk, self.smoothJointsTrajectoryRightArm = self.computeSmoothJointsTrajectory(200)
            self.plotTrajectories(self.jointsTrajectory, self.smoothJointsTrajectoryTrunk,self.smoothJointsTrajectoryRightArm)             
            self.followJointsTrajectory(self.rightArm, self.smoothJointsTrajectoryTrunk, self.smoothJointsTrajectoryRightArm)
    
    def moveUpObject(self):
        print("Aproach to object")
        trunkCurrentQ = yarp.DVector(self.numTrunkJoints)
        if (not self.trunkIEncoders.getEncoders(trunkCurrentQ)):
            print("Unable to get trunk encoders position")
            raise SystemExit

        armCurrentQ = yarp.DVector(self.numRightArmJoints)
        if not self.rightArmIEncoders.getEncoders(armCurrentQ):
            print("Unable to get arm encoders position")
            raise SystemExit

        current_Q = yarp.DVector(self.numRightArmJoints+self.numTrunkJoints)
        desire_Q = yarp.DVector(self.numRightArmJoints+self.numTrunkJoints)
        aproachingQs = []
        goal_q = []
                        
        for j in range(self.numTrunkJoints):
            current_Q[j] = trunkCurrentQ[j]
            goal_q.append(trunkCurrentQ[j])
        for j in range(0, self.numRightArmJoints):
            current_Q[j+2] = armCurrentQ[j]
            goal_q.append(armCurrentQ[j])
       
        self.reachingPose = yarp.DVector(6)
        self.trunkRightArmICartesianSolver.fwdKin(current_Q, self.reachingPose)
        frame_reaching_base = self.vectorToFrame(self.reachingPose)
                        
        # First we need to get the encoders for the first postion
        aproachingQs.append(goal_q)
        self.d = 0
        frame_aux_base = frame_reaching_base
        while self.d <= 0.95*self.reachingDistance:
            print(self.d)
            self.d = self.d + self.reachingDistance/100.0

            # frame_aux_reaching = PyKDL.Frame()
            # frame_aux_reaching.p.z(self.d)

            frame_aux_base.p.z(self.d)

            aux_pose = self.frameToVector(frame_aux_base)
                            
            x_vector = yarp.DVector(6)
            for i in range(len(x_vector)):
                x_vector[i] = aux_pose[i]
           
            if(self.trunkRightArmICartesianSolver.invKin(x_vector, current_Q, desire_Q)):
                goal_q = []
                for i in range(0,self.numTrunkJoints+self.numRightArmJoints):
                    goal_q.append(desire_Q[i])
                aproachingQs.append(desire_Q)
                print("current_Q: ", current_Q)
                print("desire_Q: ", current_Q)
                print("distance: ", self.d)
                current_Q = desire_Q
                print("Aproaching Qs: ", aproachingQs)
        # Lets smooth the trajectory
        self.jointsTrajectory = aproachingQs
        if(len(self.jointsTrajectory)>20):
            self.smoothJointsTrajectoryTrunk, self.smoothJointsTrajectoryRightArm = self.computeSmoothJointsTrajectory(200)
            self.plotTrajectories(self.jointsTrajectory, self.smoothJointsTrajectoryTrunk,self.smoothJointsTrajectoryRightArm)             
            self.followJointsTrajectory(self.rightArm, self.smoothJointsTrajectoryTrunk, self.smoothJointsTrajectoryRightArm)

    def checkJointsPosition(self, desiredJointsPosition, iEncoders, numJoints):
        currentQ = yarp.DVector(numJoints)
        iEncoders.getEncoders(currentQ)
        print(list(currentQ))
        for joint in range(numJoints):
            # print(currentQ[joint])
            if abs(currentQ[joint]-desiredJointsPosition[joint]) > self.jointsPositionError:
                return False
        return True

yarp.Network.init()

test = TestTrajectoryGeneration()
test.execute()
