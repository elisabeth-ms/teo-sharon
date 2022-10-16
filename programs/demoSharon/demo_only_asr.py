import yarp

def main():
    from SharonLib import sharon
    robot = "/teoSim"
    prefix = "/demo"
    
    demo = sharon.Sharon(robot=robot, prefix=prefix, only_asr=True, use_right_arm=True, 
                         available_right_arm_controlboard=True, available_left_arm_controlboard=True,
                         available_trunk_controlboard=True, reachingDistance=0.09, upDistance=0.1)
    demo.configure()
    
    # demo.state0([0,10])
    # demo.state1([0,10, -50, -50, 40, -70, 30, -20])
    # demo.state2()
    # demo.state3()
    # demo.openHand()
    # yarp.delay(5.0)
    demo.moveTowardsObject()
    yarp.delay(5.0)
    demo.closeHand()
    yarp.delay(5.0)
    demo.moveUpObject()
    

yarp.Network.init()
main()