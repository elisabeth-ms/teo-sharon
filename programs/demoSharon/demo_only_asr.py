import yarp

def main():
    from SharonLib import sharon
    robot = "/teo"
    prefix = "/demo"
    use_right_arm = True
    if use_right_arm:
        demo = sharon.Sharon(robot=robot, prefix=prefix, only_asr=True, use_right_arm=use_right_arm, 
                         available_right_arm_controlboard=True, available_left_arm_controlboard=False,
                         available_trunk_controlboard=True,available_right_hand_controlboard=True,
                         available_left_hand_controlboard=False,reachingDistance=0.07, upDistance=0.1)
    else:
        demo = sharon.Sharon(robot=robot, prefix=prefix, only_asr=True, use_right_arm=use_right_arm, 
                         available_right_arm_controlboard=False, available_left_arm_controlboard=True,
                         available_right_hand_controlboard=False, available_left_hand_controlboard=True,
                         available_trunk_controlboard=True, reachingDistance=0.09, upDistance=0.1)
        
    demo.configure()
    
    demo.state0([0,10])
    yarp.delay(5.0)
    # # demo.openHand()
    # demo.state1([0, 10, -50, -50, 40, -70, 30, -20])
    # # demo.state1([0,10,-50, 50, -40, -70, -30, -20]) # left arm
    demo.state2()
    if demo.state3():
        demo.moveUpObject()

    # demo.openHand()
    # yarp.delay(5.0)
    # demo.moveTowardsObject()
    # yarp.delay(5.0)
    # demo.closeHand()
    # yarp.delay(5.0)
    

yarp.Network.init()
main()