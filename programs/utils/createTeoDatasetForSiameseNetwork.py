from random import shuffle
from shutil import copyfile
import os
from PIL import Image

totalWidth = 640
totalHeight = 480

path = r"/home/elisabeth/data/"
pathImages = path+"teoImages/color/"
pathLabels = path+"sharonXtionDatabase/"
newPath = r"/home/elisabeth/data/siamese_dataset/1/teo/"

labelsList = os.listdir(pathLabels)

print(labelsList)

index = [0,0,0,0]

for labelsFileName in labelsList:
    print(labelsFileName)
    if labelsFileName!= "classes.txt":
        image = labelsFileName[:-4] + ".jpg"
        with open(pathLabels+labelsFileName,'r') as fileLabel:
            
            lines = fileLabel.readlines()
            fileLabel.close()
                
            for line in lines:

                content = line.split(" ")
                category = float(content[0])
                centerX = float(content[1])
                centerY = float(content[2])
                width = float(content[3])*totalWidth
                height = float(content[4])*totalHeight
                    #width = 200/totalWidth
                    #height = 200/totalHeight
                    
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
                                    
                if category == 21: # olive-oil
                    print(category)
                    imageName = newPath +"olive-oil/"+ "olive-oil_image_"
                    im1.save(imageName+str(index[0]) + ".ppm")
                    index[0] = index[0]+1
                elif category == 20: # water
                    imageName = newPath +"water/"+ "water_image_"
                    im1.save(imageName+str(index[1]) + ".ppm")
                    index[1] = index[1]+1
                elif category == 8: # sliced-bread
                    imageName = newPath +"sliced-bread/"+ "sliced-bread_image_"
                    im1.save(imageName+str(index[1]) + ".ppm")
                    index[1] = index[1]+1
        
    # if category == 0: # cereal1
    #     imageName = newPath +"cereal1/"+ "cereal1_image_"
    #     im1.save(imageName+str(index[0]) + ".ppm")
    #     index[0] = index[0]+1
    
    # elif category == 1: # cereal2
    #     imageName = newPath +"cereal2/"+ "cereal2_image_"
    #     im1.save(imageName+str(index[1]) + ".ppm")
    #     index[1] = index[1]+1
        
    # elif category == 2: # cereal3
    #     imageName = newPath +"cereal3/"+ "cereal3_image_"
    #     im1.save(imageName+str(index[1]) + ".ppm")
    #     index[1] = index[1]+1
    # elif category == 3: # milk1
    #     imageName = newPath +"milk1/"+ "milk1_image_"
    #     im1.save(imageName+str(index[1]) + ".ppm")
    #     index[1] = index[1]+1
    # elif category == 4: # milk2
    #     imageName = newPath +"milk2/"+ "milk2_image_"
    #     im1.save(imageName+str(index[1]) + ".ppm")
    #     index[1] = index[1]+1
    # elif category == 5: # nesquick
    #     imageName = newPath +"nesquick/"+ "nesquick_image_"
    #     im1.save(imageName+str(index[1]) + ".ppm")
    #     index[1] = index[1]+1
    # elif category == 6: # water
    #     imageName = newPath +"water/"+ "water_image_"
    #     im1.save(imageName+str(index[1]) + ".ppm")
    #     index[1] = index[1]+1
          
    # elif category == 2: # cup
    #     imageName = newPath +"cup/"+ "cup_image_"
    #     im1.save(imageName+str(index[2]) + ".jpg", "JPEG")
    #     index[2] = index[2]+1
    
    # elif category == 3: # glass
    #     imageName = newPath +"glass/"+ "glass_image_"
    #     im1.save(imageName+str(index[3]) + ".jpg", "JPEG")
    #     index[3] = index[3]+1
        
    
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


