
import yarp
import sys
import pickle
import matplotlib.pyplot as plt
from PIL import Image 
import numpy as np
import pickle 
import os

### ----------------- Just for the example ----------------------###
cwd = os.getcwd()
pathImages = cwd + '/cup__2021-01-29_04-57-47-e2cc26b5'
fixationsFileStr = pathImages+'/fixations.pkl'

def get_frames_fixations_from_pkl(fixationsFileStr):
    with open(fixationsFileStr, 'rb') as f:
        data = pickle.load(f)
    frames = data['frames']
    fixations = data['fixations']
    return frames, fixations

frames, fixations = get_frames_fixations_from_pkl(fixationsFileStr)
frame_number = 0
### ----------------- Just for the example ----------------------###



class GlassesServerYarp(yarp.RFModule):
    def __init__(self):
        """ 
        param glassesImageResolution: Tuple (width, height) with the resolution of the images 
        """
        yarp.RFModule.__init__(self)
                
        self.imageWidth = 0
        self.imageHeight = 0
        self.glasses_images_port = yarp.BufferedPortImageRgb() # Create a port
        self.glasses_fixation_point_port = yarp.Port()
    
    def configure(self, rf):
        
        self.imageWidth = rf.find("width").asInt32()
        self.imageHeight = rf.find("height").asInt32()
        
        # Open the yarp ports in the yarp network
        self.glasses_images_port.open("/glassesServer/images:o") # Give name to the port in the yarp network
        self.glasses_fixation_point_port.open("/glassesServer/fixationPoint:o") 
        
        print("RFModule configure. Running the model")
        return True
    
    def interruptModel(self):
        print("Stopping the module")
        self.glasses_images_port.interrupt()
        self.glasses_images_port.interrupt()
        return True
    
    def close(self):
        self.glasses_images_port.close()
        self.glasses_fixation_point_port.close()
        return True
    
    def getPeriod(self):
        """
        Module refresh rate
        
        Returns: The period of the module in seconds.
        """
        return 0.5
    
        
    def write_image_yarp_port(self, img_array):
        """ Send the image through the yarp port
        
        param img_array: image data in the form of numpy array [height, width, 3].
            Image to be sent.
        """
        
        yarp_img = self.glasses_images_port.prepare() # Get a place to store the image to be sent
        
        img = yarp.ImageRgb()
            
        img.resize(self.imageHeight, self.imageWidth)
       
        img.setExternal(img_array.data, self.imageHeight, self.imageWidth) # Wrap yarp image around the numpy array  
            
        yarp_img.copy(img)
        print(img_array)
        self.glasses_images_port.write()
    
    def write_fixation_point_yarp_port(self, fixation_point):
        """ Send the fixation point through a yarp port
        
        param fixation_point: Tuple (x,y) with the fixation point in image coordinates
        """   
        bottle_fixation_point = yarp.Bottle()
        bb = bottle_fixation_point.addList()
        bb.addDouble(fixation_point[0])
        bb.addDouble(fixation_point[1])
        self.glasses_fixation_point_port.write(bottle_fixation_point)


    def updateModule(self):
        ''' 
        This function is called periodically every getPeriod() seconds
        '''
        
        # Change img_array (numpy array [height, width, 3]) with the actual image from the glasses server
        global frame_number
        print(frame_number)
        im=Image.open(pathImages+"/"+frames[frame_number])
        img_array = np.asarray(im)
        
        # Send image as a numpy array through yarp port
        glasses_server_yarp.write_image_yarp_port(img_array) 
        
        # Change fixation_point (tuple (fixationX, fixationY)) with the actual fixation point from the glasses server
        fixation_point = (fixations[frame_number][0], fixations[frame_number][1])
        
        # Send fixation point though yarp port
        glasses_server_yarp.write_fixation_point_yarp_port(fixation_point)
        
        frame_number = frame_number + 1
        
        return True
        
if __name__ == "__main__":
    print("main")
    yarp.Network.init()  # connect to YARP network

    if not yarp.Network.checkNetwork():  # let's see if there was actually a reachable YARP network
        print('arp network is not found!!')   
        sys.exit(1)


    print("glasses server")
    glasses_server_yarp = GlassesServerYarp()
    print("glasses server")

    rf = yarp.ResourceFinder()
    rf.setVerbose(True)
    rf.setDefaultContext('glassesServer')
    rf.setDefaultConfigFile('glassesServer.ini')
    
    rf.configure(sys.argv)
    glasses_server_yarp.runModule(rf)
    
    sys.exit()   