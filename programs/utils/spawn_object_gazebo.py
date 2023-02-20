import yarp
import sys
import random
import numpy as np
import random
from PIL import Image, ImageDraw
import os
import kinematics_dynamics
import PyKDL
import numpy as np
import matplotlib.pyplot as plt
import pickle
import math

# Trunk device
trunkOptions = yarp.Property()
trunkDevice = None
trunkIEncoders = None
numTrunkJoints = 2
trunkIPositionControl = None

# Head device
headOptions = yarp.Property()
headDevice = None
headIEncoders = None
numHeadJoints = 2
headIPositionControl = None
trunkHeadIControlLimits = None

# Trunk and head solver device
trunkHeadSolverOptions = yarp.Property()
trunkHeadSolverDevice = None

simulation_data_directory = "/home/elisabeth/data/simulation_data/"

absolute_width = 640
absolute_height = 480
just_move = False

robot = '/teoSim'


def getBoundingPoints3DwrtWorld(fobjectCenter_world, bounding3Dpoints_center, index):
    fpoint_center = PyKDL.Frame()
    fpoint_center.p.x(bounding3Dpoints_center[index][0])
    fpoint_center.p.y(bounding3Dpoints_center[index][1])
    fpoint_center.p.z(bounding3Dpoints_center[index][2])

    fpoint_world = fobjectCenter_world*fpoint_center

    return fpoint_world


def vectorToFrame(x):

    f = PyKDL.Frame()

    f.p.x(x[0])
    f.p.y(x[1])
    f.p.z(x[2])

    rotvec = PyKDL.Vector(x[3], x[4], x[5])
    f.M = PyKDL.Rotation.Rot(rotvec, rotvec.Norm())
    return f


def frameToVector(frame):
    x = []

    x.append(frame.p.x())
    x.append(frame.p.y())
    x.append(frame.p.z())

    rotVector = frame.M.GetRot()
    x.append(rotVector.x())
    x.append(rotVector.y())
    x.append(rotVector.z())
    return x


def transformObjWrtWorldToWrtCamera(trunkIEncoders, numTrunkJoints, headIEncoders, numHeadJoints, trunkHeadICartesianSolver):
    trunkCurrentQ = yarp.DVector(numTrunkJoints)
    trunkIEncoders.getEncoders(trunkCurrentQ)

    headCurrentQ = yarp.DVector(numHeadJoints)
    headIEncoders.getEncoders(headCurrentQ)

    current_Q = yarp.DVector(numTrunkJoints+numHeadJoints)
    for j in range(0, numHeadJoints):
        current_Q[j+2] = headCurrentQ[j]
    for j in range(numTrunkJoints):
        current_Q[j] = trunkCurrentQ[j]

    print(current_Q[0], current_Q[1], current_Q[2], current_Q[3])

    x_head_trunk = yarp.DVector(6)
    trunkHeadICartesianSolver.fwdKin(current_Q, x_head_trunk)

    print(x_head_trunk[0], x_head_trunk[1], x_head_trunk[2],
          x_head_trunk[3], x_head_trunk[4], x_head_trunk[5])
    # TODO: transform to kdl frame head_wrt_trunk and object xyz -> object_xyz_wrt_trunk

    frame_head_trunk = vectorToFrame(x_head_trunk)

    return frame_head_trunk


def getPixelCoordinates(fpoint_camera, principalPoint, focalLength):
    xpixel = -fpoint_camera.p.y()/fpoint_camera.p.x() * \
        focalLength[0] + principalPoint[0]
    ypixel = -fpoint_camera.p.z()/fpoint_camera.p.x() * \
        focalLength[1] + principalPoint[1]
    if xpixel > absolute_width:
        xpixel = absolute_width
    if ypixel > absolute_height:
        ypixel = absolute_height
    if xpixel < 0:
        xpixel = 0
    if ypixel < 0:
        ypixel = 0
    return [int(xpixel), int(ypixel)]


def drawRectImage(tlx, tly, brx, bry, imageArray):
    im = Image.fromarray(imageArray)
    draw = ImageDraw.Draw(im)
    draw.rectangle([(tlx, tly), (brx, bry)], fill=None, width=5, outline="red")
    imWithRect = np.asarray(im)
    return imWithRect


if __name__ == "__main__":
    print("main")

    if not just_move:
        try:
            category = str(sys.argv[1])
            startImage = int(sys.argv[2])
            label = int(sys.argv[3])
        except:
            print(
                '(category start label) (Category, Number of naming the files, label number')
    else:
        category = str(sys.argv[1])
        print("Just moving the object ", category, " activated...")
    yarp.Network.init()  # connect to YARP network

    if category[-1].isdigit():
        objectCategory = category[:-1]
    else:
        objectCategory = category

    print(objectCategory)
    with open(objectCategory+'.pkl', 'rb') as f:
        bounding3Dpoints_center = pickle.load(f)
        print(bounding3Dpoints_center)

    if not yarp.Network.checkNetwork():  # let's see if there was actually a reachable YARP network
        print('yarp network is not found!!')
        sys.exit(1)

    rf = yarp.ResourceFinder()
    rf.setVerbose(True)
    rf.setDefaultContext('spawn_object_gazebo')
    rf.setDefaultConfigFile('spawn_object_gazebo.ini')

    rf.configure(sys.argv)
    # matching_object_port = yarp.Port()
    # matching_object_port.open('/spawn/matching/object:i')
    # yarp.Network.connect('/matching/object:o', '/spawn/matching/object:i')

    world_interface_client = yarp.RpcClient()
    world_interface_client.open("/test/world_interface")
    world_interface_client.addOutput("/world_interface_port")

    camera_rpc_client = yarp.RpcClient()
    camera_rpc_client.open("/test" + "/teoSim/camera/rpc:i")
    camera_rpc_client.addOutput("/teoSim/camera/rpc:i")

    # Open trunk device
    trunkOptions.put('device', 'remote_controlboard')
    trunkOptions.put('remote', robot+'/trunk')
    trunkOptions.put('local', robot+'/trunk')

    trunkDevice = yarp.PolyDriver(trunkOptions)
    trunkDevice.open(trunkOptions)
    if not trunkDevice.isValid():
        print('Cannot open trunk device!')
        raise SystemExit

    # Open head device
    headOptions.put('device', 'remote_controlboard')
    headOptions.put('remote', robot+'/head')
    headOptions.put('local', robot+'/head')

    headDevice = yarp.PolyDriver(headOptions)
    headDevice.open(headOptions)
    if not headDevice.isValid():
        print('Cannot open head device!')
        raise SystemExit

    trunkIEncoders = trunkDevice.viewIEncoders()

    if trunkIEncoders == []:
        print("Trunk Encoder interface NOT available.")
        raise SystemExit
    else:
        print("Trunk Encoder interface available.")

    numTrunkJoints = trunkIEncoders.getAxes()

    # Trunk position control interface
    trunkIPositionControl = trunkDevice.viewIPositionControl()

    if trunkIPositionControl == []:
        print("Trunk position control interface NOT available")
        raise SystemExit
    else:
        print("Trunk position control interface available.")

    headIEncoders = headDevice.viewIEncoders()

    if headIEncoders == []:
        print("Head Encoder interface NOT available.")
        raise SystemExit
    else:
        print("Head Encoder interface available.")
    numHeadJoints = headIEncoders.getAxes()

    # Head position control interface
    headIPositionControl = headDevice.viewIPositionControl()

    if headIPositionControl == []:
        print("Head position control interface NOT available")
        raise SystemExit
    else:
        print("Head position control interface available.")

     # trunk and head solver device
    trunkHeadKinPath = rf.findFileByName("teo-trunk-head.ini")
    trunkHeadSolverOptions.fromConfigFile(trunkHeadKinPath)
    trunkHeadSolverOptions.put("device", "KdlSolver")
    trunkHeadSolverOptions.put("ik", "nrjl")
    trunkHeadSolverOptions.fromString("(mins (-20 -10.8 -70 -29))", False)
    trunkHeadSolverOptions.fromString("(maxs (20 10.1 70 8.4))", False)
    trunkHeadSolverDevice = yarp.PolyDriver(
        trunkHeadSolverOptions)  # calls open -> connects
    trunkHeadSolverDevice.open(trunkHeadSolverOptions)

    if trunkHeadSolverDevice == []:
        print("trunk head  solver device interface NOT available")
    else:
        print("trunk head solver device interface available.")

    # Trunk and head cartesian solver
    trunkHeadICartesianSolver = kinematics_dynamics.viewICartesianSolver(
        trunkHeadSolverDevice)
    if trunkHeadICartesianSolver == []:
        print("Trunk head cartesian solver interface NOT available")
    else:
        print("Trunk head cartesian solver interface available.")

    frame_camera_trunk = transformObjWrtWorldToWrtCamera(
        trunkIEncoders, numTrunkJoints, headIEncoders, numHeadJoints, trunkHeadICartesianSolver)

    frame_trunk_world = PyKDL.Frame()
    frame_trunk_world.p.z(0.845)

    frame_camera_world = frame_trunk_world * frame_camera_trunk

    # Get camera params
    cmd = yarp.Bottle()
    rep = yarp.Bottle()
    cmd.addString('visr')
    cmd.addString('get')
    cmd.addString('intp')
    camera_rpc_client.write(cmd, rep)

    focalLength = [rep.get(3).find('focalLengthX').asDouble(), rep.get(
        3).find('focalLengthY').asDouble()]

    principalPoint = [rep.get(3).find('principalPointX').asDouble(), rep.get(
        3).find('principalPointY').asDouble()]
    #principalPoint = [319.5, 239.5]
    print("focalLength: ", focalLength)
    print("principalPoint: ", principalPoint)

    if not just_move:
        global start
        start = int(startImage)

        inRgbPort = yarp.BufferedPortImageRgb()
        inRgbPort.open("/rgbImage")
        yarp.Network.connect("/teoSim/camera/rgbImage:o", "/rgbImage")

        inDepthPort = yarp.Port()
        inDepthPort.open("/depthImage")
        yarp.Network.connect("/teoSim/camera/depthImage:o", "/depthImage")

        #color_detection_port_name = "/rgbdDetection/teoSim/camera/state:o"
        #input_port_name = "/fake/rgbdDetection/teoSim/camera/state:o"
        # input_port = yarp.Port()
        # input_port.open(input_port_name)

        #yarp.Network.connect(color_detection_port_name, input_port_name)

        img_array = np.zeros((480, 640, 3), dtype=np.uint8)
        # yarp_image = yarp.ImageRgb()
        # yarp_image.resize(640, 480)
        # yarp_image.setExternal(
        #     img_array.data, img_array.shape[1], img_array.shape[0])

        depthImgArray = np.zeros((480, 640, 1), dtype=np.float64)
        yarpImgDepth = yarp.ImageFloat()
        yarpImgDepth.resize(640, 480)
        yarpImgDepth.setExternal(
            depthImgArray.data, depthImgArray.shape[1], depthImgArray.shape[0])

        color_detection_data = yarp.Bottle()

        num_image = start
    for x in np.arange(0.6, 1.2, 0.15):
        for y in np.arange(-0.3, 0.4, 0.15):
            startRotZ = random.uniform(0, 0.2)
            for rotZ in np.arange(startRotZ, np.pi*2.0, 0.2):
                print("Lets modify the position of the object ",
                      category, "x: ", x, "y: ", y)
                cmd = yarp.Bottle()
                rep = yarp.Bottle()
                cmd.addString('setPose')
                cmd.addString(category)
                cmd.addDouble(x)
                cmd.addDouble(y)
                cmd.addDouble(1.08)
                cmd.addDouble(3.1415)
                cmd.addDouble(0)
                cmd.addDouble(rotZ)
                world_interface_client.write(cmd, rep)
                yarp.delay(0.25)
                if not just_move:
                    cmd = yarp.Bottle()
                    rep = yarp.Bottle()
                    cmd.addString('getPose')
                    cmd.addString(category)

                    world_interface_client.write(cmd, rep)

                    fobjectCenter_world = PyKDL.Frame()
                    fobjectCenter_world.p.x(rep.get(0).asDouble())
                    fobjectCenter_world.p.y(rep.get(1).asDouble())
                    fobjectCenter_world.p.z(rep.get(2).asDouble())
                    fobjectCenter_world.M = PyKDL.Rotation.RPY(
                        rep.get(3).asDouble(), rep.get(4).asDouble(), rep.get(5).asDouble())
                    tlx = absolute_width
                    tly = absolute_height
                    brx = 0
                    bry = 0
                    print("Center object wrt world:",
                          frameToVector(fobjectCenter_world))
                    for i in range(len(bounding3Dpoints_center)):
                        fpoint_world = PyKDL.Frame()
                        fpoint_world = getBoundingPoints3DwrtWorld(
                            fobjectCenter_world, bounding3Dpoints_center, i)
                        fpoint_camera = frame_camera_world.Inverse()*fpoint_world
                        print("Point ", i, ": ", frameToVector(fpoint_world))
                        pixel = getPixelCoordinates(
                            fpoint_camera, principalPoint, focalLength)
                        print("pixel point ", i, pixel)
                        if pixel[0] < tlx:
                            tlx = pixel[0]
                        if pixel[1] < tly:
                            tly = pixel[1]
                        if pixel[0] > brx:
                            brx = pixel[0]
                        if pixel[1] > bry:
                            bry = pixel[1]
                        print(tlx, tly, brx, bry)

                    print("End points: ", tlx, tly, brx, bry)

                    # inRgbPort.read(yarp_image)
                    while np.array_equal(img_array,  np.zeros((480, 640, 3), dtype=np.uint8)):
                        yarp_image = inRgbPort.read(False)
                        if yarp_image:
                            yarp_image.setExternal(
                                img_array.data, img_array.shape[1], img_array.shape[0])
                        yarp.delay(0.1)

                        print("image: ", np.array_equal(
                            img_array,  np.zeros((480, 640, 3), dtype=np.uint8)))

                    inDepthPort.read(yarpImgDepth)
                    im = Image.fromarray(img_array)

                    # outImage = drawRectImage(tlx, tly, brx, bry, img_array)
                    # plt.imshow(outImage)
                    # plt.show()

                    strImageFile = str(num_image)
                    print(strImageFile)

                    # if not os.path.exists("rgbImage_o/"+strImageFile+'.ppm'):
                    print("Save rgbImage: ", "rgbImage_o/"+strImageFile, ".ppm")
                    im.save(simulation_data_directory +
                            "rgbImage_o/" + strImageFile + '.ppm')
                    print("Save depthImage: ", "depthImage_o/" +
                          strImageFile, ".float")
                    yarp.write(yarpImgDepth, simulation_data_directory +
                               "depthImage_o/"+strImageFile + '.float')
                    labels_file = open(
                        simulation_data_directory+"labels/"+strImageFile+'.txt', "w+")
                    x_label = (brx+tlx)/(2.0*absolute_width)
                    y_label = (bry+tly)/(2.0*absolute_height)
                    width = (brx-tlx)/absolute_width
                    height = (bry-tly)/absolute_height
                    labels_file.write(str(
                        label)+" "+str(x_label)+" "+str(y_label)+" " + str(width)+" "+str(height)+"\n")
                    labels_file.close()
                    num_image += 1
        # else:
        #     for y in np.arange(-0.25, 0.5, 0.15):
        #         startRotZ = random.uniform(0, 0.2)
        #         print(startRotZ)
        #         for rotZ in np.arange(startRotZ, np.pi*2.0, 0.2):
        #             print("Lets modify the position of the object ",
        #                   category, "x: ", x, "y: ", y)
        #             cmd = yarp.Bottle()
        #             rep = yarp.Bottle()
        #             cmd.addString('setPose')
        #             cmd.addString(category)
        #             cmd.addDouble(x)
        #             cmd.addDouble(y)
        #             cmd.addDouble(1)
        #             cmd.addDouble(0)
        #             cmd.addDouble(0)
        #             cmd.addDouble(rotZ)
        #             world_interface_client.write(cmd, rep)
        #             yarp.delay(1.0)

        #             if not just_move:
        #                 inRgbPort.read(yarp_image)
        #                 inDepthPort.read(yarpImgDepth)

        #                 brx = color_detection_data.get(
        #                     0).find("brx").asDouble()
        #                 bry = color_detection_data.get(
        #                     0).find("bry").asDouble()

        #                 tlx = color_detection_data.get(
        #                     0).find("tlx").asDouble()
        #                 tly = color_detection_data.get(
        #                     0).find("tly").asDouble()

        #                 print(brx, bry, tlx, tly)

        #                 im = Image.fromarray(img_array)

        #                 strImageFile = str(num_image)
        #                 # print(strImageFile)

        #                 # if not os.path.exists("rgbImage_o/"+strImageFile+'.ppm'):
        #                 print("Save rgbImage: ", "rgbImage_o/" +
        #                       strImageFile, ".ppm")
        #                 im.save(simulation_data_directory +
        #                         "rgbImage_o/" + strImageFile + '.ppm')
        #                 print("Save depthImage: ", "depthImage_o/" +
        #                       strImageFile, ".float")
        #                 yarp.write(yarpImgDepth, simulation_data_directory +
        #                            "depthImage_o/"+strImageFile + '.float')
        #                 labels_file = open(
        #                     simulation_data_directory+"labels/"+strImageFile+'.txt', "w+")
        #                 x_label = (brx+tlx)/(2.0*absolute_width)
        #                 y_label = (bry+tly)/(2.0*absolute_height)
        #                 width = (brx-tlx)/absolute_width
        #                 height = (bry-tly)/absolute_height
        #                 labels_file.write(str(
        #                     label)+" "+str(x_label)+" "+str(y_label)+" " + str(width)+" "+str(height)+"\n")
        #                 labels_file.close()
        #                 num_image += 1

    print("Closing ports")
    inDepthPort.close()
    inRgbPort.close()
    # object_data = yarp.Bottle()
    # matching_object_port.read(object_data)
    # category = object_data.get(0).find('category').asString()
    # locationTCP = [object_data.get(0).find("x").asDouble(), object_data.get(0).find("y").asDouble(), object_data.get(0).find("z").asDouble()]
    # print(category, locationTCP)

    # Check if we already have a model of the object in Gazebo
    # model_present = False

    # cmd = yarp.Bottle()
    # rep = yarp.Bottle()
    # cmd.addString("getPose")
    # cmd.addString(category)
    # world_interface_client.write(cmd, rep)
    # model_present = not all(rep.get(i).asDouble() == 0 for i in range(6))
    # print(model_present)

    # if not model_present:
    #     if category == 'water':
    #         cmd = yarp.Bottle()
    #         rep = yarp.Bottle()
    #         cmd.addString("makeCylinder")
    #         cmd.addDouble(0.04) # radius [mt]
    #         cmd.addDouble(0.320)  # lenght x  [mt]
    #         cmd.addDouble(locationTCP[0])  # pos x  [mt]
    #         cmd.addDouble(locationTCP[1])  # pos y

    #         cmd.addDouble(locationTCP[2]+0.845)  # pos z
    #         cmd.addDouble(0.0)  # pose.roll  [rad]
    #         cmd.addDouble(0.0)  # pose.pitch [rad]
    #         cmd.addDouble(0.0)  # pose.yaw   [rad]
    #         cmd.addInt(0)       # color.r
    #         cmd.addInt(0)     # color.g
    #         cmd.addInt(255)       # color.b
    #         cmd.addString('')
    #         cmd.addString('water')
    #     elif category == 'milk':
    #         cmd = yarp.Bottle()
    #         rep = yarp.Bottle()
    #         cmd.addString("makeBox")
    #         cmd.addDouble(0.195) # radius [mt]
    #         cmd.addDouble(0.08)  # lenght x  [mt]
    #         cmd.addDouble(0.340)
    #         cmd.addDouble(locationTCP[0])  # pos x  [mt]
    #         cmd.addDouble(locationTCP[1])  # pos y
    #         cmd.addDouble(locationTCP[2]+0.845)  # pos z
    #         cmd.addDouble(0.0)  # pose.roll  [rad]
    #         cmd.addDouble(0.0)  # pose.pitch [rad]
    #         cmd.addDouble(0.0)  # pose.yaw   [rad]
    #         cmd.addInt(0)       # color.r
    #         cmd.addInt(0)     # color.g
    #         cmd.addInt(255)       # color.b
    #     world_interface_client.write(cmd, rep)
    # else:
