#!/usr/bin/python3

''' glasses_database.py
    This script is used for visulaizing the images and fixation points from glasses database.
    Press any key to pause/continue the frames visualization.
    For running it:
    python3 glasses_database.py directory_of_object_with_pkl_and_images/
'''

import sys
import pickle
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

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
        fig.canvas.mpl_connect("key_press_event", nextImage)    
        i=0    
        while i < (len(frames)):
            if continueImages:
                print(directoryObject+frames[i])
                im=mpimg.imread(directoryObject+frames[i])
                prev_x, prev_y = x, y
                x = fixations[i][0]
                y = fixations[i][1]
                if img is None:
                    temporaryPoint, =ax.plot(x, y,marker='o', color="white")
                    img = ax.imshow(im)
                else:
                    temporaryPoint.set_data(x,y)
                    img.set_data(im)
                plt.pause(.01)
                plt.draw()
                i+=1
            else:
                print("Press any key for continue visualizing the next frames!!")
                plt.pause(1.0)