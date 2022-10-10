import yarp
import PyKDL
import matplotlib
import matplotlib.pyplot as plt
import numpy as np
from scipy import interpolate
from matplotlib.ticker import MaxNLocator
import time

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
        self.reachingDistance = 0.05
        
        self.numTrunkJoints = 2
        self.numRightArmJoints = 6
        
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
        cmd.addString('ggp')
        cmd.addInt32(1)
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
        print("followJointsTrajectory ", self.numPointTrajectory,
              len(jointsTrajectoryTrunk))
        self.numPointTrajectory = 0
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

yarp.Network.init()

test = TestTrajectoryGeneration()
test.execute()
