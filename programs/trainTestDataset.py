from random import shuffle
from shutil import copyfile
import os
from PIL import Image
import numpy as np
import shutil
from random import randint
path = r"/home/elisabeth/data/siamese_dataset/glasses_only_milks/milk2/"
pathTrain = r"/home/elisabeth/data/siamese_dataset/glasses_only_milks/train/milk2/"
pathTest = r"/home/elisabeth/data/siamese_dataset/glasses_only_milks/test/milk2/"


imagesList = os.listdir(path)

numberOfImages = 500
x=np.random.randint(len(imagesList), size=(len(imagesList)-numberOfImages))
listRandomImages = []
i = 0
while i<numberOfImages:
    x = randint(0,len(imagesList)-1)
    while imagesList[x] in listRandomImages:
        x = randint(0,len(imagesList)-1)
    listRandomImages.append(imagesList[x])
    i = i+1
print(len(listRandomImages))

for image in listRandomImages:
    try:
        shutil.copy(path+image,pathTrain+image)
    except OSError:
        pass


imagesListTest = [i for i in imagesList if i not in listRandomImages]

for image in imagesListTest:
    try:
        shutil.copy(path+image,pathTest+image)
    except OSError:
        pass
