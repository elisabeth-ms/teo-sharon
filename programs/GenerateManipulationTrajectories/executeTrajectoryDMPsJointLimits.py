import JointLimitsDMPLib
import yarp
import sys
import csv
import numpy as np

from JointLimitsDMPLib.JointLimitsDMPLib import jointLimitsDMP


robot = '/teoSim'
prefix = '/demoSharon'

available_right_arm_controlboard = True
available_trunk_controlboard = True


class ExecuteTrajectoryJLDMP(yarp.RFModule):
    def __init__(self):
        yarp.RFModule.__init__(self)
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
        self.rigthArmIControlLimits = None


        # Trunk device
        self.trunkOptions = yarp.Property()
        self.trunkDevice = None
        self.trunkIEncoders = None
        self.numTrunkJoints = 0
        self.trunkIPositionControl = None
        self.trunkIControlMode = None
        self.trunkIPositionDirect = None
        self.trunkIRemoteVariables = False
        self.trunkIControlLimits = None


        # tts client
        self.rpcClientTts = yarp.RpcClient()

        # Trunk and Right Arm solver device
        self.trunkRightArmSolverOptions = yarp.Property()
        self.trunkRightArmSolverDevice = None
        
        self.jointsGoalPositionPort = yarp.BufferedPortBottle()
        self.demo_object_input_port_name = "/executeTrajectoryDMP/goalJoints:i"
        
        self.q_min = []
        self.q_max = []
        
        self.dmp= None
        self.dt = 0.001
        self.execution_time = 1.0
        self.n_features = 50


    def configure(self, rf):
        print("Configuring...")

        self.jointsGoalPositionPort.open(self.demo_object_input_port_name)

        # Open rightArm device
        if available_right_arm_controlboard:
            self.rightArmOptions.put('device', 'remote_controlboard')
            self.rightArmOptions.put('remote', robot+'/rightArm')
            self.rightArmOptions.put('local', prefix+robot+'/rightArm')
            

            self.rightArmDevice = yarp.PolyDriver(self.rightArmOptions)
            self.rightArmDevice.open(self.rightArmOptions)
            if not self.rightArmDevice.isValid():
                print('Cannot open rightArm device!')
                raise SystemExit

        # Open trunk device
        self.trunkOptions.put('device', 'remote_controlboard')
        self.trunkOptions.put('remote', robot+'/trunk')
        self.trunkOptions.put('local', prefix+robot+'/trunk')

        self.trunkDevice = yarp.PolyDriver(self.trunkOptions)
        self.trunkDevice.open(self.trunkOptions)
        if available_trunk_controlboard:
            if not self.trunkDevice.isValid():
                print('Cannot open trunk device!')
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
            
            self.rightArmIRemoteVariables = self.rightArmDevice.viewIRemoteVariables()
            if self.rightArmIRemoteVariables == []:
                print("Right arm remote variables interface NOT available.")
                raise SystemExit
            else:
                print("Right arm remote variables interface available.")

            self.rigthArmIControlLimits = self.rightArmDevice.viewIControlLimits()
            if self.rigthArmIControlLimits == []:
                print("Right arm control limits interface NOT available.")
                raise SystemExit
            else:
                print("Right arm control limits interface available.")
            rightArmModes = yarp.IVector(6,yarp.VOCAB_CM_POSITION_DIRECT)
            if not self.rightArmIControlMode.setControlModes(rightArmModes):
                print("Unable to set right arm  to position direct mode.")
                raise SystemExit
            else:
                print("Right arm set to position direct mode.") 
            

                
            # Right arm position control interface
            self.rightArmIPositionControl = self.rightArmDevice.viewIPositionControl()

            if self.rightArmIPositionControl == []:
                print("Right arm position control interface NOT available")
                raise SystemExit
            else:
                print("Right arm position control interface available.")


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

            self.trunkIControlLimits = self.trunkDevice.viewIControlLimits()
            if self.trunkIControlLimits == []:
                print("Trunk control limits interface NOT available.")
                raise SystemExit
            else:
                print("Trunk control limits interface available.")
            
            self.trunkIRemoteVariables = self.trunkDevice.viewIRemoteVariables()
            if self.trunkIRemoteVariables == []:
                print("Trunk remote variables interface NOT available.")
                raise SystemExit
            else:
                print("Trunk remote variables interface available.")
                

            trunkModes = yarp.IVector(2,yarp.VOCAB_CM_POSITION_DIRECT)
            if not self.trunkIControlMode.setControlModes(trunkModes):
                print("Unable to set trunk to position direct mode.")
                raise SystemExit
            else:
                print("Trunk set to position direct mode.")
                
        # Get the number of joints of right arm and trunk        
        self.numRightArmJoints = self.rightArmIEncoders.getAxes()
        self.numTrunkJoints = self.trunkIEncoders.getAxes()
        self.q_min = [None]*(self.numRightArmJoints+self.numTrunkJoints)
        self.q_max = [None]*(self.numRightArmJoints+self.numTrunkJoints)

        
        self.setJointLimits(self.trunkIControlLimits, self.numTrunkJoints, 0)
        self.setJointLimits(self.rigthArmIControlLimits, self.numRightArmJoints, self.numTrunkJoints)
        
        print(self.q_min)
        print(self.q_max)
        
        y_demo = self.getYDemoFromCSVFile(csv_name='trajectories/fake/fake-right-arm-motion-joint-1.csv')
        self.execution_time = 1
        self.dt = self.execution_time/(y_demo.shape[1])          
        self.dmp = jointLimitsDMP.JointLimitsDMP(self.dt, self.execution_time, self.n_features, self.q_min, self.q_max,"trunkAndRightArm", len(self.q_min)+1)
        
        
        # x0 = np.zeros(self.numRightArmJoints+self.numTrunkJoints)
        # for i in range(self.numRightArmJoints+self.numTrunkJoints):
        #     x0[i] = y_demo[i][0]
        # x0 =  np.expand_dims(x0, axis=1)
        # print("x0: ", x0)
        # print(y_demo[:][0])
        # g = np.zeros(self.numRightArmJoints+self.numTrunkJoints)
        # for i in range(self.numRightArmJoints+self.numTrunkJoints):
        #     g[i] = y_demo[i][-1]
        # g =  np.expand_dims(g, axis=1)
        # print("g: ",g)
        
        time = np.linspace(0,self.execution_time,y_demo.shape[1])
        print("time: ", time.shape)
        gamma = []
        for i in range(y_demo.shape[1]):
            print(y_demo[:, i])
            aux = np.append(time[i], [y_demo[:,i]])
            gamma.append(aux)
        gamma = np.array(gamma)
        
        print(gamma.shape)
        
        x0 = gamma[0].copy()
        g = gamma[-1].copy()
        x0[1] = x0[1] -2
        g[1] = g[1] - 8
        
        correct, trajectory = self.dmp.imitateDemoInsideBounds(gamma, x0, g)
        
        self.dmp.plotDMPS(gamma,trajectory)
        
        
        print("demo_sharon Module initialized...")
        return True
    
    def interruptModel(self):
        print("Stopping the module")

    def close(self):
        print("close")
        return True

    def getPeriod(self):
        return 2.0
    
    def updateModule(self):
        print("Demo sharon running")
        return True
       
    def setJointLimits(self, IControlLimits, numJoints, startingIndex):
        for joint in range(numJoints):
            min = yarp.Vector(1)
            max = yarp.Vector(1)
            IControlLimits.getLimits(joint, min.data(), max.data())
            self.q_min[startingIndex+joint] = min.get(0)
            self.q_max[startingIndex+joint] = max.get(0)
        self.q_min[0] = -31.0
        self.q_max[0] = 31.0
        self.q_max[1] = 16.5
    
    def getYDemoFromCSVFile(self, csv_name):
        qTraj = []
        with open(csv_name) as csv_file:
            csv_reader = csv.reader(csv_file, delimiter=',')
            line_count = 0
            for row in csv_reader:
                qTraj.append([float(row[1]), float(row[2]), float(row[3]), float(row[4]), float(row[5]), float(row[6]), float(row[7]), float(row[8])])
                line_count += 1
        qTraj = np.array(qTraj)
        print(qTraj.shape)
        return qTraj.T



if __name__ == "__main__":
    print("main")
    yarp.Network.init()  # connect to YARP network

    if not yarp.Network.checkNetwork():  # let's see if there was actually a reachable YARP network
        print('yarp network is not found!!')
        sys.exit(1)

    executeTrajectory = ExecuteTrajectoryJLDMP()

    rf = yarp.ResourceFinder()
    rf.setVerbose(True)
    rf.setDefaultContext('executeTrajectoryDMPJointLimits')
    rf.setDefaultConfigFile('executeTrajectoryDMPJointLimits.ini')

    rf.configure(sys.argv)
    executeTrajectory.runModule(rf)

    sys.exit()


