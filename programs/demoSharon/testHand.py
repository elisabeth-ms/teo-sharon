import yarp

yarp.Network.init()

# options = yarp.Property()
# options.put('device', 'remote_controlboard')
# options.put('remote', '/teo/leftHand')
# options.put('local', '/remoteLeftHand')

# device = yarp.PolyDriver(options)
# device.open(options)
# if not device.isValid():
#     print('Cannot open leftHand device!')
#     raise SystemExit
# pwm = device.viewIPWMControl()

optionsRight = yarp.Property()
optionsRight.put('device', 'remote_controlboard')
optionsRight.put('remote', '/teo/rightHand')
optionsRight.put('local', '/remoteRightHand')

deviceRight = yarp.PolyDriver(optionsRight)
deviceRight.open(optionsRight)
if not deviceRight.isValid():
    print('Cannot open rightHand device!')
    raise SystemExit
pwmRight = deviceRight.viewIPWMControl()

pwmRight.setRefDutyCycle(0, 100) # [-100, 100]
pwmRight.setRefDutyCycle(0, 100) # [-100, 100]

yarp.delay(10.0)

# for i in range(100):
#     pwmRight.setRefDutyCycle(0, -100) # [-100, 100]
#     print("i: ", i)
#     yarp.delay(0.05)
pwmRight.setRefDutyCycle(0, -100) # [-100, 100]
yarp.delay(10.0)

print("Now lets wait 8 seconds without sending messages")
pwmRight.setRefDutyCycle(0, 100) # [-100, 100]

yarp.delay(8.0)

# # pwm.setRefDutyCycle(0, 100) # [-100, 100]
# yarp.delay(2.0)



# pwmRight.setRefDutyCycle(0, 100) # [-100, 100]
# yarp.delay(2.0)