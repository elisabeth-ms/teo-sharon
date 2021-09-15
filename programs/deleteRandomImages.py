from random import shuffle
from shutil import copyfile
import os
from PIL import Image
import numpy as np
import shutil
from random import randint
path = r"/home/elisabeth/data/siamese_dataset/glasses_only_milks/milk2/"


imagesList = os.listdir(path)
print(imagesList)

numberOfImages = 650
x=np.random.randint(len(imagesList), size=(len(imagesList)-numberOfImages))


for i in x:
    try:
        os.remove(path+imagesList[i])
    except OSError:
        pass
