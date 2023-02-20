#!/usr/bin/python3

import yarp
import sys
import numpy as np
from PIL import Image
from pynput import keyboard
import os

def on_release(key):
    if key == keyboard.Key.esc:
        # Stop listener
        return False
    if key == keyboard.Key.enter:
        global start
        start +=1



if __name__ == "__main__":
    # Initialise YARP
    yarp.Network.init()
    try:
        startImage = sys.argv[1]
    except:
        print('(start) Number of naming the files')
    
    global start
    start = int(startImage)
    width = 640
    height = 480
    
    inRgbPort = yarp.Port()
    inRgbPort.open("/rgbImage")
    yarp.Network.connect("/realsense2/rgbImage:o","/rgbImage")

    inDepthPort = yarp.Port()
    inDepthPort.open("/depthImage")
    yarp.Network.connect("/realsense2/depthImage:o", "/depthImage")

    img_array = np.zeros((height, width, 3), dtype=np.uint8)
    yarp_image = yarp.ImageRgb()
    yarp_image.resize(width, height)
    yarp_image.setExternal(img_array.data, img_array.shape[1], img_array.shape[0])
    
    depthImgArray = np.random.uniform(0., 65535., (height, width)).astype(np.float32)
    yarpImgDepth = yarp.ImageFloat()
    yarpImgDepth.resize(width,height)
    yarpImgDepth.setExternal(depthImgArray.data,depthImgArray.shape[1], depthImgArray.shape[0])

    listener = keyboard.Listener(
        on_release=on_release)
    listener.start()
    try:
        while True:
            inRgbPort.read(yarp_image)
            inDepthPort.read(yarpImgDepth)

            
            im = Image.fromarray(img_array)
            # listImageFile = list("00000000")
            # listNumber = str(start)
            # for i in range(len(listNumber)):
            #     listImageFile[len(listImageFile)-1-i] = listNumber[len(listNumber)-1-i]
            strImageFile = str(start)
            
            
            formatted = (depthImgArray/depthImgArray.max()*65535/2.0).astype('uint16')
            # print("depth max: ", depthImgArray.max())
            # print(formatted)
            imDepth = Image.fromarray(formatted)

            #print(strImageFile)
            
            if not os.path.exists("color/"+strImageFile+'.jpg'):
                print("Save rgbImage: ", "color/"+strImageFile,".jpg")
                im.save("color/"+ strImageFile + '.jpg')
                imDepth.save("depth/"+strImageFile + '.png', 'png')
                np.save("depthArray/"+strImageFile + '.npy', depthImgArray)
                #imwrite("depth/test.png", imageF_8UC3)
                print("Save depthImage: ", "depth/"+strImageFile,".png")
                #yarp.write(yarpImgDepth,"depth/"+strImageFile + '.png')
    except KeyboardInterrupt:
        print("Press Ctrl-C to terminate while statement")
    pass
    
    print("Closing ports")
    inDepthPort.close()
    inRgbPort.close()
