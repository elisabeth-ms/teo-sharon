#!/usr/bin/python3


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

    fixationsFileStr = directoryObject+'annotation.pkl'
    print(fixationsFileStr)

    with open(fixationsFileStr, 'rb') as f:
        data = pickle.load(f)
        frames = data['frames']
        fixations = data['fixations']
        action_level = data['action_labels']
        labels = data['labels']

        img = None
        x,y=0,0

        fig, ax = plt.subplots()
        fig.canvas.mpl_connect("key_press_event", nextImage)    
        i=0    
        while i < (len(frames)):
            if(action_level[i]==1):
                print(directoryObject+frames[i], action_level[i], labels[i])
                im=mpimg.imread(directoryObject+"Frames/"+frames[i]+".png")
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