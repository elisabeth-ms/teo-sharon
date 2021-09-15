#!/usr/bin/python3

''' generateCropsDatasetfromglassesImageAndFixations.py
    This script is used for visulaizing the images and fixation points from glasses database.
    Press any key to pause/continue the frames visualization.
    For running it:
    python3 generateCropsDatasetfromglassesImageAndFixations.py directory_of_object_with_pkl_and_images/
'''

import sys
import pickle
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from PIL import Image 

path = r"/home/elisabeth/data/siamese_dataset/glasses_only_milks/"
label = "milk2"
pathImages = path+label+"/"
imageName = pathImages + label+"_image_"

continueImages = True
def nextImage(evt=None):
    global continueImages
    continueImages = not continueImages



if __name__ == "__main__":
    directoryObject = ''
    try:
        directoryObject = sys.argv[1]
        print(directoryObject)
    except:
        print('Please pass directory_name')

    fixationsFileStr = directoryObject+'fixations.pkl'
    print(fixationsFileStr)

    with open(fixationsFileStr, 'rb') as f:
        data = pickle.load(f)
        frames = data['frames']
        fixations = data['fixations']
        img = None
        x,y=0,0
        fig, ax = plt.subplots()
        i=0
        crop_width = 200
        crop_height = 200
        index = 0
        while i < (len(frames)):
                print(directoryObject+frames[i])
                imPIL = Image.open(directoryObject+frames[i])
                total_width, total_height = imPIL.size
                left = fixations[i][0] - crop_width/2.0
                right = fixations[i][0] + crop_width/2.0
                top = fixations[i][1] - crop_height/2.0
                bottom = fixations[i][1] + crop_height/2.0
                area = (left, top, right, bottom)

                im1 = imPIL.crop(area)
                im1.save(imageName+str(index) + ".jpg", "JPEG")
                index = index+1
                # prev_x, prev_y = x, y
                # x = fixations[i][0]
                # y = fixations[i][1]
                # if img is None:
                #     temporaryPoint, =ax.plot(x, y,marker='o', color="white")
                #     img = ax.imshow(im)
                # else:
                #     temporaryPoint.set_data(x,y)
                #     img.set_data(im)
                # plt.pause(.01)
                # plt.draw()
                i+=1
