from random import shuffle
from shutil import copyfile
import os
from PIL import Image

totalWidth = 640
totalHeight = 480

path = r"/home/elisabeth/data/dataset/"
pathImages = path+"images/jpg/"
pathLabels = path+"labels/"

newPath = r"/home/elisabeth/data/siamese_dataset/teo_variable_sizes/"

imagesList = os.listdir(pathImages)

print(imagesList)

index = [0,0,0,0]

for image in imagesList:
    label = image[:-4] + ".txt"
    fileLabel = open(pathLabels+label,'r')
    line = fileLabel.read()
    content = line.split(" ")
    category = float(content[0])
    centerX = float(content[1])
    centerY = float(content[2])
    width = float(content[3])*totalWidth
    height = float(content[4])*totalHeight
    #width = 200/totalWidth
    #height = 200/totalHeight
    fileLabel.close()
    
    left = centerX*totalWidth - width/2.0
    right = centerX*totalWidth + width/2.0
    top = centerY*totalHeight - height/2.0
    bottom = centerY*totalHeight + height/2.0

    print(category, centerX, centerY, width, height)
    # print(left,right, top, bottom)
    imPIL = Image.open(pathImages+image) 
    area = (left, top, right, bottom)
    # if width*totalWidth >=200 or height*totalHeight>=200:
    #     print(width*totalWidth, height*totalHeight)
    im1 = imPIL.crop(area)
    
    if category == 0: # rice
        imageName = newPath +"rice/"+ "rice_image_"
        im1.save(imageName+str(index[0]) + ".jpg", "JPEG")
        index[0] = index[0]+1
    
    elif category == 1: # milk
        imageName = newPath +"milk_bottle/"+ "milk_bottle_image_"
        im1.save(imageName+str(index[1]) + ".jpg", "JPEG")
        index[1] = index[1]+1
        
    elif category == 2: # cup
        imageName = newPath +"cup/"+ "cup_image_"
        im1.save(imageName+str(index[2]) + ".jpg", "JPEG")
        index[2] = index[2]+1
    
    elif category == 3: # glass
        imageName = newPath +"glass/"+ "glass_image_"
        im1.save(imageName+str(index[3]) + ".jpg", "JPEG")
        index[3] = index[3]+1
        
    
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


