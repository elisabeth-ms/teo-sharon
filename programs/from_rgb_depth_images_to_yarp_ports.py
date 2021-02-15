#!/usr/bin/python3

''' from_rgb_depth_images_to_yarp_ports.py
    This scripts gathers one rgb image and one depth image and publishes them
    through two /rgbImage and /depthImage yarp ports.
    Usage:
        $ python3 from_rgb_depth_images_to_yarp_ports.py path_to_rgb_image.ppm path_to_depth_image.float
    For visulazing the images from the ports:
        $ yarpview --name /yarpview/rgbImage
        $ yarpview --name /yarpview/depthImage
        $ yarp connect /rgbImage /yarpview/rgbImage mjpeg
        $ yarp connect /depthImage /yarpview/depthImage udp+recv.portmonitor+type.dll+file.depthimage
'''

import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import sys
import yarp
import numpy as np
import struct
import copy
from os import listdir
from os.path import isfile, join

def fromRgbImageFileToArray(rgbImageFile):
    # Read rgb image file and create the rgb image array 
    imgFile=mpimg.imread(rgbImageFile)
    print(imgFile.shape)
    img_array = np.zeros((imgFile.shape[0],imgFile.shape[1], imgFile.shape[2]), dtype=np.uint8)
    for i in range(imgFile.shape[0]):
        for j in range(imgFile.shape[1]):
            for k in range(imgFile.shape[2]):
                img_array[i,j,k] = imgFile[i,j,k]
    return  img_array 


def fromDepthImageFileToArray(depthImageFile):
    # Depth image
    # Open the depthImage.float 
    imFile = open(depthImageFile,'rb')
    # Get height of the depth image (the first 8 bytes)
    line = imFile.readline(8)
    h = int.from_bytes(line,"little") # Transform the 8 bytes to little endian int
    # Get the width of the depth image 
    line = imFile.readline(8)
    w = int.from_bytes(line,"little") # Transform the 8 bytes to little endian int

    # Create the yarp image
    yarpImgDepth = yarp.ImageFloat()
    # Resize with the width and height from the image file
    yarpImgDepth.resize(w,h)
    # Create a numpy array of float for storing the data from the depth image file. 
    # Important! This numpy array must have shape (height, width) 
    depthImgArray = np.random.uniform(0., 255., (h, w)).astype(np.float32)
    
    # Fill all the element of the numpy array with the data from the file. 
    # Read data for each pixel (one pixel is getPixelSize() bytes) for filling the array (as float). 
    for j in range(0,h):
        for i in range(0,w):
            depthImgArray[j][i] = struct.unpack('f', imFile.read(yarpImgDepth.getPixelSize()))[0]
    
    return depthImgArray,w,h



if __name__ == "__main__":
    # Initialise YARP
    yarp.Network.init()
    rgbImage = ''
    depthImage = ''
    mode = -1
    try:
        mode = int(sys.argv[1])
        rgbImage = sys.argv[2]
        depthImage = sys.argv[3]
       
    except:
        print('Please pass rgbImage and depthImage')
    
    # Create the yarp ports
    outpuRgbPort = yarp.Port()
    outputDepthPort = yarp.Port()
    outpuRgbPort.open("/rgbImage")
    outputDepthPort.open("/depthImage")

    if mode == 0:
        print('Mode 0 selected, sending only the rgbImage and depthImage through the ports')
        print('Selected rgbImage:', rgbImage)
        print('Selected depthImage:', depthImage)

        depthImgArray,w,h = fromDepthImageFileToArray(depthImage)
        depthYarpImg = yarp.ImageFloat()
        # Resize with the width and height from the image file
        depthYarpImg.resize(w,h)
    
        # Wrap the yarp depth image around the depth image array      
        depthYarpImg.setExternal(depthImgArray.data, depthImgArray.shape[1], depthImgArray.shape[0])

        yarpRgbImg = yarp.ImageRgb()
        imgArray = fromRgbImageFileToArray(rgbImage)
        copyImg = copy.copy(imgArray)
        yarpRgbImg.resize(copyImg.shape[1],copyImg.shape[0])

        yarpRgbImg.setExternal(copyImg.data, copyImg.shape[1], copyImg.shape[0])

        
        try:
            while True:
                print('Send the rgb and depth images through the ports...')
                outpuRgbPort.write(yarpRgbImg)
                outputDepthPort.write(depthYarpImg)
                yarp.delay(0.5)

        except KeyboardInterrupt:
            print('Script interrupted!')
        
    elif mode ==1:
        print('Mode 1 selected. Send all images in the rgb and depth directories through the ports')
        print('RgbImages directory:', rgbImage)
        print('DepthImages directory', depthImage)

        rgbImagesStr = [f for f in listdir(rgbImage) if isfile(join(rgbImage, f))]
        rgbImagesStrOrdered = ["" for i in range(len(rgbImagesStr))]
        for currentRgbImage in rgbImagesStr:
            rgbImagesStrOrdered[(int(currentRgbImage[:-4]))] = currentRgbImage

        depthImagesStr = [f for f in listdir(depthImage) if isfile(join(depthImage, f))]
        depthImagesStrOrdered = ["" for i in range(len(depthImagesStr))]
        for currentDepthImage in depthImagesStr:
            depthImagesStrOrdered[(int(currentDepthImage[:-6]))] = currentDepthImage

        print(rgbImagesStrOrdered)
        print(depthImagesStrOrdered)

        yarp.delay(5.0)
        i = 0
        while i <len(rgbImagesStrOrdered):
            currentRgbImage = rgbImage+"/"+rgbImagesStrOrdered[i]
            currentDepthImage = depthImage+"/"+depthImagesStrOrdered[i]

            print('Selected rgbImage:', currentRgbImage)
            print('Selected depthImage:', currentDepthImage)
            
            depthImgArray,w,h = fromDepthImageFileToArray(currentDepthImage)
            
            depthYarpImg = yarp.ImageFloat()
            # Resize with the width and height from the image file
            depthYarpImg.resize(w,h)
    
            # Wrap the yarp depth image around the depth image array      
            depthYarpImg.setExternal(depthImgArray.data, depthImgArray.shape[1], depthImgArray.shape[0])

            yarpRgbImg = yarp.ImageRgb()
            imgArray = fromRgbImageFileToArray(currentRgbImage)

            yarpRgbImg.resize(imgArray.shape[1],imgArray.shape[0])
            yarpRgbImg.setExternal(imgArray.data, imgArray.shape[1], imgArray.shape[0])

            try:
                print('Send the rgb and depth images through the ports...')
                outpuRgbPort.write(yarpRgbImg)
                outputDepthPort.write(depthYarpImg)
                yarp.delay(1.0)

            except KeyboardInterrupt:
                print('Script interrupted!')
            i+=1

    print('Clossing ports...')
    output_port.close()
    output_depth_port.close()