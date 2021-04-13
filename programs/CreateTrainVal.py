from random import shuffle
from shutil import copyfile
import os
from PIL import Image


pathObjects = r"/home/elisabeth/repos/darknet/data/obj/"
pathImages = r"/home/elisabeth/data/dataset/images/jpg/"
pathTxt = r"/home/elisabeth/repos/darknet/data/"

imagesList = os.listdir(pathImages)

shuffle(imagesList)


print(imagesList)

trainList = imagesList[0:int(0.9*len(imagesList))]
print("train:",trainList)
print(len(trainList))
valList = imagesList[int(0.9*len(imagesList)):]
print("val:",valList)
print("Total size: ",len(imagesList)," Train size: ", len(trainList), " Val size: ",len(valList))

fileTrain = open(pathTxt+'train.txt', 'w')
fileVal = open(pathTxt+'val.txt', 'w')

for image in trainList:
    fileTrain.write(pathObjects+image+'\n')

for image in valList:
    fileVal.write(pathObjects+image+'\n')   
# pathTrain = path + 'train/'
# try:
#     os.mkdir(pathTrain)
# except OSError:
#     print ("Creation of the directory %s failed" % pathTrain)
# else:
#     print ("Successfully created the directory %s " % pathTrain)
    
# pathTrainImages = pathTrain + "images/"
# try:
#     os.mkdir(pathTrainImages)
# except OSError:
#     print ("Creation of the directory %s failed" % pathTrainImages)
# else:
#     print ("Successfully created the directory %s " % pathTrainImages)

# pathTrainLabels = pathTrain + "labels/"
# try:
#     os.mkdir(pathTrainLabels)
# except OSError:
#     print ("Creation of the directory %s failed" % pathTrainLabels)
# else:
#     print ("Successfully created the directory %s " % pathTrainLabels)
    

# for image in trainList:
#     print("Copying image: ",image,"to train images directory: ", pathTrainImages)
#     trainImageFile = pathTrainImages + image[:-4]
    
#     im = Image.open(pathImages + image)
#     im.save(trainImageFile+".jpg")
#     label = image[:-4] + ".txt"
#     print("Copying label: ",label,"to train labels directory: ",pathTrainLabels)
#     copyfile(pathLabels + label, pathTrainLabels + label)


# pathVal = path + 'val/'
# try:
#     os.mkdir(pathVal)
# except OSError:
#     print ("Creation of the directory %s failed" % pathVal)
# else:
#     print ("Successfully created the directory %s " % pathVal)
    
# pathValImages = pathVal + "images/"
# try:
#     os.mkdir(pathValImages)
# except OSError:
#     print ("Creation of the directory %s failed" % pathValImages)
# else:
#     print ("Successfully created the directory %s " % pathValImages)

# pathValLabels = pathVal + "labels/"
# try:
#     os.mkdir(pathValLabels)
# except OSError:
#     print ("Creation of the directory %s failed" % pathValLabels)
# else:
#     print ("Successfully created the directory %s " % pathValLabels)
    

# for image in valList:
#     print("Copying image: ",image,"to val images directory: ", pathValImages)
#     valImageFile = pathValImages + image[:-4]
    
#     im = Image.open(pathImages + image)
#     im.save(valImageFile+".jpg")
#     label = image[:-4] + ".txt"
#     print("Copying label: ",label,"to val labels directory: ",pathValLabels)
#     copyfile(pathLabels + label, pathValLabels + label)


