from random import shuffle
from shutil import copyfile
import os
from PIL import Image
import pickle
import numpy as np
pathImages = r"/home/elisabeth/Downloads/grasping_database/colors/milk__2021-06-09_07-41-53-0da341fc__1/"
pathResizedImages = r"/home/elisabeth/Downloads/grasping_database/resized/colors/milk__2021-06-09_07-41-53-0da341fc__1/"

imagesList = os.listdir(pathImages)

previous_size = [1088,1080]
size = [725, 720]
for image in imagesList:
    if '.pkl' not in image:
        im = Image.open(pathImages+image)
        im_resized = im.resize(size, Image.ANTIALIAS)
        im_resized.save(pathResizedImages+image, "PNG")


with open(pathImages+'fixations.pkl', 'rb') as f:
    data = pickle.load(f)
    frames = data['frames']
    fixations = data['fixations']
    action_level = data['action_level']
    
print(np.asarray(fixations[0]))


for i in range(len(fixations)):
    print(fixations[i])
    fixations[i][0] = fixations[i][0]*size[0]/previous_size[0]
    fixations[i][1] = fixations[i][1]*size[1]/previous_size[1]
    
dict_sample = dict({'frames': frames, 'fixations': fixations, 'action_level': action_level})
print(dict_sample)

a_file = open(pathResizedImages+'fixations.pkl', "wb")
pickle.dump(dict_sample,a_file)
a_file.close()

        