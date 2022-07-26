import yarp

import time

robot = "/teoSim"
rightArmOptions = yarp.Property()

rightArmOptions.put('device', 'remote_controlboard')
rightArmOptions.put('remote', robot+'/rightArm')
rightArmOptions.put('local', robot+'/rightArm')

            

rightArmDevice = yarp.PolyDriver(rightArmOptions)
rightArmDevice.open(rightArmOptions)
if not rightArmDevice.isValid():
    print('Cannot open rightArm device!')
    raise SystemExit


rightArmIControlMode = rightArmDevice.viewIControlMode()
if rightArmIControlMode == []:
    print("Right arm control mode interface NOT available.")
    raise SystemExit
else:
    print("Right arm control mode interface available.")


rightArmModes = yarp.IVector(6,yarp.VOCAB_CM_POSITION)
if not rightArmIControlMode.setControlModes(rightArmModes):
    print("Unable to set right arm  to position direct mode.")
    raise SystemExit
else:
    print("Right arm set to position direct mode.") 
    
    