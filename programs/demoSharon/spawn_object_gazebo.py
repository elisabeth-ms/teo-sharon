import yarp
import sys
import random
import numpy as np
import random
from PIL import Image
from pynput import keyboard
import os


simulation_data_directory = "/home/elisabeth/data/simulation_data/"

absolute_width = 640
absolute_height = 480
if __name__ == "__main__":
    print("main")
    
    try:
        startImage = int(sys.argv[1])
        label = int(sys.argv[2])
    except:
        print('(start label) (Number of naming the files, label number')
        
    yarp.Network.init()  # connect to YARP network

    if not yarp.Network.checkNetwork():  # let's see if there was actually a reachable YARP network
        print('yarp network is not found!!')   
        sys.exit(1)
    
    # matching_object_port = yarp.Port()
    # matching_object_port.open('/spawn/matching/object:i')
    # yarp.Network.connect('/matching/object:o', '/spawn/matching/object:i')
    
    world_interface_client = yarp.RpcClient()
    world_interface_client.open("/test/world_interface")
    world_interface_client.addOutput("/world_input_port")
    
    global start
    start = int(startImage)
    
    inRgbPort = yarp.Port()
    inRgbPort.open("/rgbImage")
    yarp.Network.connect("/teoSim/camera/rgbImage:o","/rgbImage")

    inDepthPort = yarp.Port()
    inDepthPort.open("/depthImage")
    yarp.Network.connect("/teoSim/camera/depthImage:o", "/depthImage")
    
    
    color_detection_port_name = "/rgbdDetection/teoSim/camera/state:o"
    input_port_name = "/fake/rgbdDetection/teoSim/camera/state:o"
    input_port = yarp.Port()
    input_port.open(input_port_name)
    
    yarp.Network.connect(color_detection_port_name, input_port_name)
    
    img_array = np.zeros((480, 640, 3), dtype=np.uint8)
    yarp_image = yarp.ImageRgb()
    yarp_image.resize(640, 480)
    yarp_image.setExternal(img_array.data, img_array.shape[1], img_array.shape[0])
    
    depthImgArray = np.random.uniform(0., 255., (480, 640)).astype(np.float32)
    yarpImgDepth = yarp.ImageFloat()
    yarpImgDepth.resize(640,480)
    yarpImgDepth.setExternal(depthImgArray.data,depthImgArray.shape[1], depthImgArray.shape[0])

    
    color_detection_data = yarp.Bottle()

    num_image = start
    for x in np.arange(0.5,1.0, 0.2):
        for y in np.arange(-0.25,0.35,0.2):
            startRotZ = random.uniform(0, 0.2)
            print(startRotZ)
            for rotZ in np.arange(startRotZ,np.pi+0.1, 0.2):
                category = 'milk2'
                print("Lets modify the position of the object ", category, "x: ", x, "y: ", y)
                cmd = yarp.Bottle()
                rep = yarp.Bottle()
                cmd.addString('setPose')
                cmd.addString(category)
                cmd.addDouble(x)
                cmd.addDouble(y)
                cmd.addDouble(1)
                cmd.addDouble(0)
                cmd.addDouble(0)
                cmd.addDouble(rotZ)
                world_interface_client.write(cmd, rep)
                yarp.delay(1.0)
                
                inRgbPort.read(yarp_image)
                inDepthPort.read(yarpImgDepth)
                
                input_port.read(color_detection_data)
                
                brx = color_detection_data.get(0).find("brx").asDouble()
                bry = color_detection_data.get(0).find("bry").asDouble()
                
                tlx = color_detection_data.get(0).find("tlx").asDouble()
                tly = color_detection_data.get(0).find("tly").asDouble()

                print(brx, bry, tlx, tly)

                im = Image.fromarray(img_array)
                
                

                strImageFile = str(num_image)
                #print(strImageFile)
            
                # if not os.path.exists("rgbImage_o/"+strImageFile+'.ppm'):
                print("Save rgbImage: ", "rgbImage_o/"+strImageFile,".ppm")
                im.save(simulation_data_directory+"rgbImage_o/"+ strImageFile + '.ppm')
                print("Save depthImage: ", "depthImage_o/"+strImageFile,".float")
                yarp.write(yarpImgDepth,simulation_data_directory+"depthImage_o/"+strImageFile + '.float')
                labels_file  = open(simulation_data_directory+"labels/"+strImageFile+'.txt', "w+")
                x_label = (brx+tlx)/(2.0*absolute_width)
                y_label = (bry+tly)/(2.0*absolute_height)
                width = (brx-tlx)/absolute_width
                height = (bry-tly)/absolute_height
                labels_file.write(str(label)+" "+str(x_label)+" "+str(y_label)+" " + str(width)+" "+str(height)+"\n")
                labels_file.close()
                num_image+=1
    
    print("Closing ports")
    inDepthPort.close()
    inRgbPort.close()
    input_port.close()
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
    
    