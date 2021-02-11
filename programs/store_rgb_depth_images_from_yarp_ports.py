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
    
    inRgbPort = yarp.Port()
    inRgbPort.open("/rgbImage")
    yarp.Network.connect("/xtion/rgbImage:o","/rgbImage")

    inDepthPort = yarp.Port()
    inDepthPort.open("/depthImage")
    yarp.Network.connect("/xtion/depthImage:o", "/depthImage")

    img_array = np.zeros((480, 640, 3), dtype=np.uint8)
    yarp_image = yarp.ImageRgb()
    yarp_image.resize(640, 480)
    yarp_image.setExternal(img_array.data, img_array.shape[1], img_array.shape[0])
    
    depthImgArray = np.random.uniform(0., 255., (480, 640)).astype(np.float32)
    yarpImgDepth = yarp.ImageFloat()
    yarpImgDepth.resize(640,480)
    yarpImgDepth.setExternal(depthImgArray.data,depthImgArray.shape[1], depthImgArray.shape[0])

    listener = keyboard.Listener(
        on_release=on_release)
    listener.start()
    try:
        while True:
            inRgbPort.read(yarp_image)
            inDepthPort.read(yarpImgDepth)

            im = Image.fromarray(img_array)
            listImageFile = list("00000000")
            listNumber = list(str(start))
            for i in range(len(listNumber)):
                listImageFile[len(listImageFile)-1-i] = listNumber[len(listNumber)-1-i]
            strImageFile = "".join(listImageFile) 
            #print(strImageFile)
            
            if not os.path.exists(strImageFile+'.ppm'):
                print("Save rgbImage: ", strImageFile,".ppm")
                im.save(strImageFile + '.ppm')
                print("Save depthImage: ", strImageFile,".float")
                yarp.write(yarpImgDepth,strImageFile + '.float')
    except KeyboardInterrupt:
        print("Press Ctrl-C to terminate while statement")
    pass
    
    print("Closing ports")
    inDepthPort.close()
    inRgbPort.close()