import tensorflow as tf
import matplotlib.pyplot as plt
import matplotlib
import random
import tensorflow_datasets as tfds
import seaborn as sns
print('TensorFlow version:', tf.__version__)
 
import numpy as np         # dealing with arrays
import os                  # dealing with directories
from random import shuffle # mixing up or currently ordered data that might lead our network astray in training.
import h5py
from PIL import Image, ImageDraw
from keras import backend as K
from keras.models import Model
from keras.optimizers import Adam
from keras.layers import Input, Dense, Dropout, Lambda, Convolution2D, MaxPooling2D, Flatten
from keras.losses import categorical_crossentropy
from keras.callbacks import ModelCheckpoint, EarlyStopping
from keras.applications.resnet50 import ResNet50, preprocess_input
from tensorflow.keras.preprocessing.image import ImageDataGenerator
import yarp
import logging
import sys



# TODO configure param: Confidence margin for the detections from yolo


class MatchingGlassesTeoImages(yarp.RFModule):
    
    def __init__(self):
        yarp.RFModule.__init__(self)
        self.xtionResolution = (640,480)
        # Define port variable to recive an image from Teo's camera
        self.xtionImagePort = yarp.BufferedPortImageRgb()
        # Create numpy array to receive the image
        self.xtionImageArray = np.zeros((self.xtionResolution[1], self.xtionResolution[0], 3), dtype=np.uint8)
        
        self.xtionObjectDetectionsPort = yarp.Port()
        
        self.glassesImagePort = yarp.BufferedPortImageRgb()
        self.glassesResolution = (1088,1080)
        self.glassesImageArray =  np.zeros((self.glassesResolution[1],self.glassesResolution[0],3), dtype=np.uint8)
        self.minConfidenceDetection = 0.9
        self.cropsResolution = (200,200)
        
        self.glassesFixationPort = yarp.Port()
        
        self.tripletModel = None
        self.embeddingModel = None
        
        # Output Ports
        self.fixationObjectPort = yarp.Port()
        self.fixationObjectImagePort = yarp.BufferedPortImageRgb()
        self.detections = yarp.Bottle()
        self.bottleFixationPoint = yarp.Bottle()
   


    def configure(self, rf):
        print("Configuring...")
        self.xtionImagePort.open("/matching/xtion/rgbImage:i")
        self.xtionObjectDetectionsPort.open("/matching/opencvDnnObjectDetection2D/xtion/detections:i")
        self.glassesImagePort.open("/matching/glassesServer/images:i")
        self.glassesFixationPort.open("/matching/glassesServer/fixationPoint:i")
        self.fixationObjectPort.open("/matching/fixationObject:o")
        self.fixationObjectImagePort.open("/matching/rgbImage:o")
        #connect up the output port to our input port
        yarp.Network.connect("/xtion/rgbImage:o", "/matching/xtion/rgbImage:i")
        yarp.Network.connect("/opencvDnnObjectDetection2D/xtion/detections:o", "/matching/opencvDnnObjectDetection2D/xtion/detections:i")
        yarp.Network.connect("/glassesServer/images:o", "/matching/glassesServer/images:i")
        yarp.Network.connect("/glassesServer/fixationPoint:o", "/matching/glassesServer/fixationPoint:i")

        gpus = tf.config.list_physical_devices('GPU')

        self.embeddingModel, self.tripletModel = self.getModel()

        checkpoint_path = '/home/elisabeth/repos/teo-sharon/models/training4_130TrainedLayers_128output/cp-0005.ckpt'
 
        self.tripletModel.load_weights(checkpoint_path)
    
        print("MatchingGlassesTeoImages Module initialized...")
        
        return True
    
    def interruptModel(self):
        print("Stopping the module")
        self.xtionImagePort.interrupt()
        self.glassesImagePort.interrupt()
        self.xtionObjectDetectionsPort.interrupt()
        return True
    
    def close(self):
        self.xtionImagePort.close()
        self.glassesImagePort.close()
        self.xtionObjectDetectionsPort.close()
        return True
    
    def getPeriod(self):
        return 0.05
    
    def createLabeledCropsFromDetections(self, detections):
        cropImages = []
        labelsCropImages = []
        tlxCropImages = []
        tlyCropImages = []
        brxCropImages = []
        bryCropImages = []
        for i in range(detections.size()):
            print("Detection ",i)
            dictDetection = detections.get(i)
            tlx = dictDetection.find("tlx").asInt32()
            tly = dictDetection.find("tly").asInt32()
            brx = dictDetection.find("brx").asInt32()
            bry = dictDetection.find("bry").asInt32()
            centerx = int((brx + tlx)/2.0)
            centery = int((bry + tly)/2.0)
            tlyCrop = int(centery-self.cropsResolution[1]/2)
            bryCrop = int(centery+self.cropsResolution[0]/2)
            if tlyCrop <=0:
                tlyCrop = 0
                bryCrop = self.cropsResolution[1]
            if bryCrop>=self.xtionResolution[1]:
                bryCrop = self.xtionResolution[1]
                tlyCrop = bryCrop - self.cropsResolution[1]
                
            tlxCrop = int(centerx - self.cropsResolution[0]/2)
            brxCrop = int(centerx + self.cropsResolution[0]/2)
            
            if tlxCrop <= 0:
                tlxCrop = 0
                brxCrop = self.cropsResolution[0]
            if brxCrop >= self.xtionResolution[0]:
                brxCrop = self.xtionResolution[0]
                tlxCrop = brxCrop - self.cropsResolution[0]
                
            category = dictDetection.find("category").asString()
            confidence = dictDetection.find("confidence").asFloat32()
            if confidence>=self.minConfidenceDetection:
                cropImages.append(self.xtionImageArray[tlyCrop:bryCrop, tlxCrop:brxCrop,:])
                labelsCropImages.append(category)
                brxCropImages.append(brx)
                tlxCropImages.append(tlx)
                tlyCropImages.append(tly)
                bryCropImages.append(bry)
            

        return cropImages, labelsCropImages, tlxCropImages, tlyCropImages, brxCropImages, bryCropImages
    
    def getGlassesCropImageCenterFixation(self,bottleFixationPoint):
        fixationPoint = (bottleFixationPoint.get(0).asList().get(0).asFloat32(), bottleFixationPoint.get(0).asList().get(1).asFloat32())
        tlx = int(fixationPoint[0] - self.cropsResolution[0]/2)
        brx = int(fixationPoint[0] + self.cropsResolution[0]/2)
        if tlx<=0:
            tlx = 0
            brx = self.cropsResolution[0]
        
        if brx>= self.glassesResolution[0]:
            brx = self.glassesResolution[0]
            tlx = brx - self.cropsResolution[0]
        
        tly = int(fixationPoint[1] - self.cropsResolution[1]/2)
        bry = int(fixationPoint[1] + self.cropsResolution[1]/2)
        if tly<=0:
            tly = 0
            bry = self.cropsResolution[1]
        
        if bry>= self.glassesResolution[1]:
            bry = self.glassesResolution[1]
            tly = bry - self.cropsResolution[1]
        
        return self.glassesImageArray[tly:bry, tlx:brx, :]
    
    
    def writeImageYarpPort(self, imgArray):
        """ Send the image through the yarp port
        
        param img_array: image data in the form of numpy array [height, width, 3].
            Image to be sent.
        """
        
        yarpImg = self.fixationObjectImagePort.prepare() # Get a place to store the image to be sent
        
        img = yarp.ImageRgb()
            
        img.resize(self.xtionResolution[0], self.xtionResolution[1])
       
        img.setExternal(imgArray.data, self.xtionResolution[0], self.xtionResolution[1]) # Wrap yarp image around the numpy array  
            
        yarpImg.copy(img)

        self.fixationObjectImagePort.write()  

    def tripletLoss(self,inputs, dist='sqeuclidean', margin='maxplus'):
        anchor, positive, negative = inputs
        positiveDistance = K.square(anchor - positive)
        negativeDistance = K.square(anchor - negative)
        if dist == 'euclidean':
            positiveDistance = K.sqrt(K.sum(positiveDistance, axis=-1, keepdims=True))
            negativeDistance = K.sqrt(K.sum(negativeDistance, axis=-1, keepdims=True))
        elif dist == 'sqeuclidean':
            positiveDistance = K.sum(positiveDistance, axis=-1, keepdims=True)
            negativeDistance = K.sum(negativeDistance, axis=-1, keepdims=True)
        loss = positiveDistance - negativeDistance
        if margin == 'maxplus':
            loss = K.maximum(0.0, 0.5 + loss)
        elif margin == 'softplus':
            loss = K.log(1 + K.exp(loss))
        return K.mean(loss)

    def getModel(self):
        baseModel= tf.keras.applications.ResNet50(include_top=False, weights='imagenet', pooling='max')
        x = baseModel.output
        x = Flatten() (x)
        x = Dropout(0.4)(x)
        x = Dense(128)(x)
        x = Lambda(lambda  x: K.l2_normalize(x,axis=1))(x)
        for i, layer in enumerate(baseModel.layers):
            if i>130:
                layer.trainable = True
            else:
                layer.trainable= False
        
        embeddingModel = Model(baseModel.input, x, name="embedding")
 
        for i, layer in enumerate(embeddingModel.layers):
            print(i, layer.name, layer.trainable)
  
        inputShape = (200, 200, 3)
        anchorInput = Input(inputShape, name='anchor_input')
        positiveInput = Input(inputShape, name='positive_input')
        negativeInput = Input(inputShape, name='negative_input')
        anchorEmbedding = embeddingModel(anchorInput)
        positiveEmbedding = embeddingModel(positiveInput)
        negativeEmbedding = embeddingModel(negativeInput)
 
        inputs = [anchorInput, positiveInput, negativeInput]
        outputs = [anchorEmbedding, positiveEmbedding, negativeEmbedding]
       
        tripletModel = Model(inputs, outputs)
        tripletModel.add_loss(K.mean(self.tripletLoss(outputs)))
 
        return embeddingModel, tripletModel
        
        
    def drawRectImage(self, tlx, tly, brx, bry):
        im = Image.fromarray(self.xtionImageArray)
        draw = ImageDraw.Draw(im)
        draw.rectangle([(tlx, tly), (brx, bry)], fill= None, width=5, outline="red")
        imWithRect = np.asarray(im)
        return imWithRect

        


    def updateModule(self):
        print("Update")
        detectionsCropImages = []
        detectionsCropsLabels = [] 
        
        xtionYarpImage= self.xtionImagePort.read(False)
        if xtionYarpImage:
            xtionYarpImage.setExternal(self.xtionImageArray.data, self.xtionImageArray.shape[1], self.xtionImageArray.shape[0])
        glassesYarpImage = self.glassesImagePort.read(True)
        if glassesYarpImage:
            glassesYarpImage.setExternal(self.glassesImageArray.data, self.glassesImageArray.shape[1], self.glassesImageArray.shape[0])
        
        detections = yarp.Bottle()
        self.xtionObjectDetectionsPort.read(detections)          
        if detections:
            self.detections = detections
        
        bottleFixationPoint = yarp.Bottle()
        self.glassesFixationPort.read(bottleFixationPoint)
        
        if bottleFixationPoint:
            self.bottleFixationPoint = bottleFixationPoint
        fixationPoint = (self.bottleFixationPoint.get(0).asList().get(0).asFloat32(), self.bottleFixationPoint.get(0).asList().get(1).asFloat32())
        print(fixationPoint)
        
        print(np.array_equal(self.xtionImageArray,  np.zeros((self.xtionResolution[1], self.xtionResolution[0], 3), dtype=np.uint8)))
        
        print(np.array_equal(self.glassesImageArray,np.zeros((self.glassesResolution[1],self.glassesResolution[0],3), dtype=np.uint8)))

        if not np.array_equal(self.xtionImageArray,  np.zeros((self.xtionResolution[1], self.xtionResolution[0], 3), dtype=np.uint8)) and not np.array_equal(self.glassesImageArray,np.zeros((self.glassesResolution[1],self.glassesResolution[0],3), dtype=np.uint8)):
            print("We have data from both cameras and detections")
            
            glassesCropImage = self.getGlassesCropImageCenterFixation(self.bottleFixationPoint)
            
            detectionsCropImages, detectionsCropsLabels, tlxCropImages, tlyCropImages, brxCropImages, bryCropImages = self.createLabeledCropsFromDetections(self.detections)
            
            predictEmbeddingGlasses = self.computeEmbeddingsCropImage(glassesCropImage)
            
            iMin = -1
            minDistance = np.array(10000)
            for i,crop in enumerate(detectionsCropImages):
                predictEmbeddingDetection = self.computeEmbeddingsCropImage(crop)

                distance = K.square(predictEmbeddingGlasses - predictEmbeddingDetection)
                distance = K.sqrt(K.sum(distance, axis=-1, keepdims=True))
                distance = distance.numpy()[0][0]            

                if distance < minDistance:
                    minDistance = distance
                    iMin = i
                  
            if iMin>=0:
                print(minDistance)
                fixationObject = yarp.Bottle()
                dictObject = fixationObject.addDict()
                dictObject.put("category",detectionsCropsLabels[iMin])
                dictObject.put("distance", minDistance.item())
                dictObject.put("tlx", tlxCropImages[iMin])
                dictObject.put("tly", tlyCropImages[iMin])
                dictObject.put("brx", brxCropImages[iMin])
                dictObject.put("bry", bryCropImages[iMin])
            
                self.fixationObjectPort.write(fixationObject)
                print("Min distance of ",minDistance, "for object labeled as", detectionsCropsLabels[iMin])
                outImage = self.drawRectImage(tlxCropImages[iMin], tlyCropImages[iMin], brxCropImages[iMin], bryCropImages[iMin])
            
                self.writeImageYarpPort(outImage)
            else:
                self.writeImageYarpPort(self.xtionImageArray)

                
                        
        return True

    def computeEmbeddingsCropImage(self, cropImage):
        image = np.expand_dims(cropImage, axis=0)
        inputImage = preprocess_input(np.array(image))
        predictEmbedding = self.embeddingModel.predict(inputImage)
        return predictEmbedding
            
             

if __name__ == "__main__":
    print("main")
    yarp.Network.init()  # connect to YARP network

    if not yarp.Network.checkNetwork():  # let's see if there was actually a reachable YARP network
        print('arp network is not found!!')   
        sys.exit(1)

    matchingGlassesTeoImages = MatchingGlassesTeoImages()

    rf = yarp.ResourceFinder()
    rf.setVerbose(True)
    rf.setDefaultContext('matchingGlassesTeoImages')
    rf.setDefaultConfigFile('matchingGlassesTeoImages.ini')
    
    rf.configure(sys.argv)
    matchingGlassesTeoImages.runModule(rf)
    
    sys.exit()   

# #tf.debugging.set_log_device_placement(True)
# tf.get_logger().setLevel(logging.ERROR)

# def tripletLoss(inputs, dist='sqeuclidean', margin='maxplus'):
#     anchor, positive, negative = inputs
#     positiveDistance = K.square(anchor - positive)
#     negativeDistance = K.square(anchor - negative)
#     if dist == 'euclidean':
#         positiveDistance = K.sqrt(K.sum(positiveDistance, axis=-1, keepdims=True))
#         negativeDistance = K.sqrt(K.sum(negativeDistance, axis=-1, keepdims=True))
#     elif dist == 'sqeuclidean':
#         positiveDistance = K.sum(positiveDistance, axis=-1, keepdims=True)
#         negativeDistance = K.sum(negativeDistance, axis=-1, keepdims=True)
#     loss = positiveDistance - negativeDistance
#     if margin == 'maxplus':
#         loss = K.maximum(0.0, 0.5 + loss)
#     elif margin == 'softplus':
#         loss = K.log(1 + K.exp(loss))
#     return K.mean(loss)

# def getModel():
#     baseModel= tf.keras.applications.ResNet50(include_top=False, weights='imagenet', pooling='max')
#     x = baseModel.output
#     x = Flatten() (x)
#     x = Dropout(0.4)(x)
#     x = Dense(128)(x)
#     x = Lambda(lambda  x: K.l2_normalize(x,axis=1))(x)
#     for i, layer in enumerate(baseModel.layers):
#         if i>130:
#             layer.trainable = True
#         else:
#             layer.trainable= False
        
#     embeddingModel = Model(baseModel.input, x, name="embedding")
 
#     for i, layer in enumerate(embeddingModel.layers):
#         print(i, layer.name, layer.trainable)
  
#     inputShape = (200, 200, 3)
#     anchorInput = Input(inputShape, name='anchor_input')
#     positiveInput = Input(inputShape, name='positive_input')
#     negativeInput = Input(inputShape, name='negative_input')
#     anchorEmbedding = embeddingModel(anchorInput)
#     positiveEmbedding = embeddingModel(positiveInput)
#     negativeEmbedding = embeddingModel(negativeInput)
 
#     inputs = [anchorInput, positiveInput, negativeInput]
#     outputs = [anchorEmbedding, positiveEmbedding, negativeEmbedding]
       
#     tripletModel = Model(inputs, outputs)
#     tripletModel.add_loss(K.mean(tripletLoss(outputs)))
 
#     return embeddingModel, tripletModel

# def getTestSet(datasetFile):
#     dataset = h5py.File(datasetFile, "r")
#     indices = np.arange(len(dataset['image']))
#     np.random.shuffle(indices)
#     indicesTest = indices
#     testImages = []
#     testLabels = []    
#     for i in indicesTest:
#         testImages.append(dataset['image'][i])
#         testLabels.append(dataset['label'][i])
#     testImages = np.array(testImages)
 
#     return testImages, testLabels

    



# def createTripet(teoImages, teoLabels, glassesImages, glassesLabels):
#     anchorImage = []
#     positiveImage = []
#     negativeImage = []
 
#     idxAnchor = np.random.randint(0, glassesImages.shape[0] - 1)
#     categoryAnchor = glassesLabels[idxAnchor]
#     # Anchor image augmentation
#     image = np.expand_dims(glassesImages[idxAnchor], axis=0)
#     img = aug.flow(image, batch_size=1)
#     print(img)
#     print("Category Anchor:", categoryAnchor)
#     batchAnchor = img.next()
#     anchorImage.append(batchAnchor[0]) # Append the augmented image of the anchor
 
#     # Positive image augmentation
#     idxPositive = np.random.randint(0,teoImages.shape[0]-1)
#     while categoryAnchor!= teoLabels[idxPositive]:
#         idxPositive = np.random.randint(0,teoImages.shape[0]-1)
#     image = np.expand_dims(teoImages[idxPositive], axis=0)
#     img = aug.flow(image, batch_size=1)
#     print(img)
#     batchPositive = img.next()
#     positiveImage.append(batchPositive[0]) # Append the augmented image of the anchor
 
#     idxNegative = np.random.randint(0,teoImages.shape[0]-1)
#     while categoryAnchor == teoLabels[idxNegative]:
#         idxNegative = np.random.randint(0,teoImages.shape[0]-1)
  
#     image = np.expand_dims(teoImages[idxNegative], axis=0)
#     img = aug.flow(image, batch_size=1)
#     print(img)
#     batchNegative = img.next()
#     negativeImage.append(batchNegative[0]) # Append the augmented image of the anchor
 
 
#     fig, ax = plt.subplots(nrows=2, ncols=3)
#     ax[1][0].imshow(batchAnchor[0].astype('uint8'))
#     ax[0][0].imshow(glassesImages[idxAnchor])
#     ax[0][1].imshow(teoImages[idxPositive])
#     ax[1][1].imshow(batchPositive[0].astype('uint8'))
#     ax[0][2].imshow(teoImages[idxNegative])
#     ax[1][2].imshow(batchNegative[0].astype('uint8'))
 
#     plt.show()
 
 
#     ai = preprocess_input(np.array(anchorImage))
#     pi = preprocess_input(np.array(positiveImage))
#     ni = preprocess_input(np.array(negativeImage))
        
#     print("Anchor images:",len(ai))
#     return {'anchor_input': ai, 'positive_input': pi, 'negative_input': ni}, None



# yarp.Network.init()

# xtionImagePort = yarp.BufferedPortImageRgb()
# xtionImagePort.open("/matching/xtion/rgbImage:i")

# #connect up the output port to our input port
# yarp.Network.connect("/xtion/rgbImage:o", "/matching/xtion/rgbImage:i")

# xtionObjectDetectionsPort = yarp.Port()
# xtionObjectDetectionsPort.open("/matching/opencvDnnObjectDetection2D/xtion/detections:i")

# yarp.Network.connect("/opencvDnnObjectDetection2D/xtion/detections:o", "/matching/opencvDnnObjectDetection2D/xtion/detections:i")


# glassesImagePort = yarp.BufferedPortImageRgb()
# glassesImagePort.open("/matching/glassesServer/images")
# yarp.Network.connect("/glassesServer/images", "/matching/glassesServer/images")

# # glassesFixationPort = yarp.Port()
# # glassesFixationPort.open("/matching/glassesServer/fixationPoint")
# # yarp.Network.connect("/glassesServer/fixationPoint", "/matching/glassesServer/fixationPoint")

# cropImageSize = [200,200] # [width, height]

# img_array = np.zeros((480, 640, 3), dtype=np.uint8)
# # Read the data from the port into the image
# yarp_image = yarp.ImageRgb()
# yarp_image.resize(640, 480)

# img_array_glasses = np.zeros((1080,1088,3), dtype=np.uint8)
# yarp_img_glasses = yarp.ImageRgb()
# yarp_img_glasses.resize(img_array_glasses.shape[1], img_array_glasses.shape[0])

# while True:
    

#     yarp_image.setExternal(img_array.data, img_array.shape[1], img_array.shape[0])
#     yarp_image= xtionImagePort.read(False)
    
#     print(img_array)
    
#     yarp_img_glasses.setExternal(img_array_glasses.data, img_array_glasses.shape[1], img_array_glasses.shape[0])
#     yarp_img_glasses=glassesImagePort.read(False)
    
    
    # bottleFixationPoint = yarp.Bottle()
    # glassesFixationPort.read(bottleFixationPoint)
    
    # fixationPoint = [bottleFixationPoint.get(0).asList().get(0).asFloat32(), bottleFixationPoint.get(0).asList().get(1).asFloat32()]
    # print(fixationPoint)
    
    # if not np.array_equal(img_array_glasses, np.zeros((1080,1088,3), dtype=np.uint8)):

    #     cropImgArrayGlasses = img_array_glasses[int(fixationPoint[1]-cropImageSize[1]/2):int(fixationPoint[1]+cropImageSize[1]/2), int(fixationPoint[0]-cropImageSize[0]/2):int(fixationPoint[0]+cropImageSize[0]/2),:]
    
    #     # plt.imshow(cropImgArrayGlasses, interpolation='nearest')
    #     # plt.show()
    
    # detections = yarp.Bottle()
    # xtionObjectDetectionsPort.read(detections)
    # tlx = []
    # tly = []
    # bry = []
    # brx = []
    # category = []
    # confidence = []
     
    # for i in range(detections.size()):
    #     print("Detection ",i)
    #     dictDetection = detections.get(i)
    #     tlx.append(dictDetection.find("tlx").asInt32())
    #     tly.append(dictDetection.find("tly").asInt32())
    #     brx.append(dictDetection.find("brx").asInt32())
    #     bry.append(dictDetection.find("bry").asInt32())
    #     category.append(dictDetection.find("category").asString())
    #     confidence.append(dictDetection.find("confidence").asFloat32())
    #     print("Detections bounding box: (",tlx,", ",tly,", ", brx, ", ", bry)
    #     print("Category: ", category)
    #     print("Confidence: ", confidence)
        
    # # TODO only loop over detections once
    # # print("tlx", tlx)
    # if img_array.size != 0 and not np.array_equal(img_array, np.zeros((480,640,3), dtype=np.uint8)):
    #     for i in range(len(tlx)):
    #         centerx = int((brx[i] + tlx[i])/2.0)
    #         centery = int((bry[i] + tly[i])/2.0)
    #         print(centerx, centery)
    #         tlyCrop = int(centery-cropImageSize[1]/2)
    #         bryCrop = int(centery+cropImageSize[1]/2)
    #         if tlyCrop <=0:
    #             tlyCrop = 0
    #             bryCrop = cropImageSize[1]
    #         if bryCrop>=480:
    #             bryCrop = 480
    #             tlyCrop = bryCrop - cropImageSize[1]
    #         tlxCrop = int(centerx - cropImageSize[0]/2)
    #         brxCrop = int(centerx + cropImageSize[0]/2)
    #         if tlxCrop <= 0:
    #             tlxCrop = 0
    #             brxCrop = cropImageSize[0]
    #         if brxCrop >= 640:
    #             brxCrop = 640
    #             tlxCrop = brxCrop - cropImageSize[0]
    
    #         cropDetectedObject = img_array[tlyCrop:bryCrop, tlxCrop: brxCrop,:]
    #         print(cropDetectedObject)
    #         plt.imshow(cropDetectedObject, interpolation='nearest')
    #         plt.show()
    
    

    #print(img_array_glasses)
    


    # plt.imshow(img_array, interpolation='nearest')
    # plt.show()

# DESDE AQUI VIENE TODO LO DEL MATCHING CON TENSORFLOW
# aug = ImageDataGenerator(
#         rotation_range=30,
#         zoom_range=0.2,
#         width_shift_range=0.1,
#         height_shift_range=0.1,
#         shear_range=0.05,
#         horizontal_flip=True,
#     brightness_range=(0.2,1.3))

# embeddingModel, tripletModel = getModel()

# checkpoint_path = '/home/elisabeth/repos/teo-sharon/models/training4_130TrainedLayers_128output/cp-0005.ckpt'
 
# tripletModel.load_weights(checkpoint_path)

# teoTestImages, teoTestLabels = getTestSet('/home/elisabeth/data/siamese_dataset/teo/test_siamese_teo.hdf5')
# glassesTestImages, glassesTestLabels = getTestSet('/home/elisabeth/data/siamese_dataset/small_glasses/test_siamese_glasses.hdf5')


# for i in range(10):
#     X1,Y1=createTripet(teoTestImages, teoTestLabels, glassesTestImages, glassesTestLabels)
#     print("Embedding anchor")
#      
#     print("Embedding positive")
#     predictEmbeddingPositive = embeddingModel.predict(X1['positive_input'])
#     print("Embedding negative")
#     predictEmbeddingNegative = embeddingModel.predict(X1['negative_input'])

#     print("Compute distances")
#     positiveDistance = K.square(predictEmbeddingAnchor - predictEmbeddingPositive)
#     positiveDistance = K.sqrt(K.sum(positiveDistance, axis=-1, keepdims=True))
#     print(positiveDistance)

#     negativeDistance = K.square(predictEmbeddingAnchor - predictEmbeddingNegative)
#     negativeDistance = K.sqrt(K.sum(positiveDistance, axis=-1, keepdims=True))
#     print(negativeDistance)