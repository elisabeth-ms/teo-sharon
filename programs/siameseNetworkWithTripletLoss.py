import tensorflow as tf
import matplotlib.pyplot as plt
import numpy as np
import random
import tensorflow_datasets as tfds
import seaborn as sns
print('TensorFlow version:', tf.__version__)

import numpy as np         # dealing with arrays
import os                  # dealing with directories
from random import shuffle # mixing up or currently ordered data that might lead our network astray in training.
import h5py
from PIL import Image

IMG_SIZE = 200

from keras import backend as K
from keras.models import Model
from keras.optimizers import Adam
from keras.layers import Input, Dense, Dropout, Lambda, Convolution2D, MaxPooling2D, Flatten
from keras.losses import categorical_crossentropy
from keras.callbacks import ModelCheckpoint, EarlyStopping
from keras.applications.resnet50 import ResNet50, preprocess_input

class SiameseNetwork:
    def __init__(self, imageWidth, imageHeight):
        self.imageWidth = imageWidth
        self.imageHeight = imageHeight
        self.teoTrainImages = []
        self.teoTrainLabels = []
        self.teoValImages = []
        self.teoValLabels = []
        
        self.glassesTrainImages = []
        self.glassesTrainLabels = []
        self.glassesValImages = []
        self.glassesValLabels = []
        
    def loadDataset(self,teo, datasetFile, valSize = 0.2):
        dataset = h5py.File(datasetFile, "r")
        indices = np.arange(len(dataset['image']))
        np.random.shuffle(indices)
        indicesTrain = indices[:int((1-valSize)*len(dataset['image']))]
        indicesVal = indices[int((1-valSize)*len(dataset['image'])):]
        print("Dataset images shape: ", str(dataset['image'].shape))
        
        if teo:
            for i in indicesTrain:
                self.teoTrainImages.append(dataset['image'][i]/255)
                self.teoTrainLabels.append(dataset['label'][i])
            self.teoTrainImages = np.array(self.teoTrainImages)
            print("Teo train images shape:", str(self.teoTrainImages.shape))
            for i in indicesVal:
                self.teoValImages.append(dataset['image'][i]/255)
                self.teoValLabels.append(dataset['label'][i])
            self.teoValImages = np.array(self.teoValImages)
            print("Teo val images shape:", str(self.teoValImages.shape))
        else: # Glasses dataset
            for i in indicesTrain:
                self.glassesTrainImages.append(dataset['image'][i]/255)
                self.glassesTrainLabels.append(dataset['label'][i])
            self.glassesTrainImages = np.array(self.glassesTrainImages)
            print("Glasses train images shape:", str(self.glassesTrainImages.shape))
            for i in indicesVal:
                self.glassesValImages.append(dataset['image'][i]/255)
                self.glassesValLabels.append(dataset['label'][i])
            self.glassesValImages = np.array(self.glassesValImages)
            print("Glasses val images shape:", str(self.glassesValImages.shape))
            
        

        
        
    def createDataset(self, path, name):
        images = []
        labels = []
        label=0
        f = h5py.File(name, "w")
        i = 0
        for (dirpath,dirnames,filenames) in os.walk(path):
            for dirname in dirnames:
                if dirname == 'rice':
                    label = 0
                elif dirname == 'milk_bottle':
                    label = 1
                elif dirname == 'cup':
                    label = 2
                elif dirname =='glass':
                    label = 3
                print(dirname)
                for(direcpath,direcnames,files) in os.walk(path+"/"+dirname):
                    for file in files:
                        i = i + 1
                        labels = np.append(labels,label)

        f.create_dataset("label",(len(labels),),np.uint8)
        images_shape = (len(labels), self.imageWidth, self.imageWidth, 3)
        f.create_dataset("image", images_shape, np.uint8)
        f["label"][...] = labels

        i = 0
        for (dirpath,dirnames,filenames) in os.walk(path):
            for dirname in dirnames:
                if dirname == 'rice':
                    label = 0
                elif dirname == 'milk_bottle':
                    label = 1
                elif dirname == 'cup':
                    label = 2
                elif dirname =='glass':
                    label = 3
                print(dirname)
                for(direcpath,direcnames,files) in os.walk(path+"/"+dirname):
                    for file in files:
                            actual_path=path+"/"+dirname+"/"+file
                            #print(files)
                            # label=label_img(dirname)
                            path1 =path+"/"+dirname+'/'+file
                            im = Image.open(path1).convert('RGB')
                            im = im.resize((200, 200))
                            img = np.array(im, dtype="float32")
                            f["image"][i, ...] = img[None] 
                            i = i + 1
        f.close()
    
    

    def getTripletBatch(self, batchSize, trainData=True):
        while True:
            anchorImage = []
            positiveImage = []
            negativeImage = []
            if trainData:
                XTeo = self.teoTrainImages
                YTeo = self.teoTrainLabels
                XGlasses = self.glassesTrainImages
                YGlasses = self.glassesTrainLabels
            else:
                XTeo = self.teoValImages
                YTeo = self.teoValLabels
                XGlasses = self.glassesValImages
                YGlasses = self.glassesValLabels

            for _ in range(batchSize):
                idxAnchor = np.random.randint(0, XGlasses.shape[0] - 1)
                categoryAnchor = YGlasses[idxAnchor]
                anchorImage.append(XGlasses[idxAnchor])
                idxPositive = np.random.randint(0,XTeo.shape[0]-1)
                while categoryAnchor!= YTeo[idxPositive]:
                    idxPositive = np.random.randint(0,XTeo.shape[0]-1)
            
                idxNegative = np.random.randint(0,XTeo.shape[0]-1)
                while categoryAnchor == YTeo[idxNegative]:
                    idxNegative = np.random.randint(0,XTeo.shape[0]-1)
                positiveImage.append(XTeo[idxPositive])
                negativeImage.append(XTeo[idxNegative])

            ai = preprocess_input(np.array(anchorImage))
            pi = preprocess_input(np.array(positiveImage))
            ni = preprocess_input(np.array(negativeImage))
        
            print("Anchor images:",len(ai))
            yield ({'anchor_input': ai, 'positive_input': pi, 'negative_input': ni}, None)
    def GetModel(self):
        baseModel= tf.keras.applications.ResNet50(
    include_top=False, weights='imagenet', pooling='max')
        x = baseModel.output
        x = Dropout(0.2)(x)
        x = Dense(1024)(x)
        x = Lambda(lambda  x: K.l2_normalize(x,axis=1))(x)
        for i, layer in enumerate(baseModel.layers):
            if i>110:
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
    
    
        
    def evaPlot(self,history, epoch):
        plt.figure(figsize=(20,10))
        plt.figure(figsize=(20,10))
        sns.lineplot(range(1, epoch+1), history.history['loss'], label='Train loss')
        sns.lineplot(range(1, epoch+1), history.history['val_loss'], label='Test loss')
        plt.legend(['train', 'validaiton'], loc='upper left')
        plt.ylabel('loss')
        plt.xlabel('epoch')
        plt.title("Loss Graph")
        plt.show()
    
    
    def tripletLoss(self, inputs, dist='euclidean', margin='maxplus'):
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
            loss = K.maximum(0.0, 0.2 + loss)
        elif margin == 'softplus':
            loss = K.log(1 + K.exp(loss))
        print(loss)
        return K.mean(loss)
    def data_generator(self, batchSize,trainData):
        while True:
            x = self.getTripletBatch(batchSize, trainData)
        yield x
def visualizeTriplets(batchSize, anchorImage, positiveImage, negativeImage):
    fig, ax = plt.subplots(nrows=batchSize, ncols=3)
    for i in range(batchSize):
        ax[i, 0].imshow(anchorImage[i])
        ax[i, 1].imshow(positiveImage[i])
        ax[i, 2].imshow(negativeImage[i])
    plt.show()


pathTeo='/home/elisabeth/data/siamese_dataset/teo/'
pathGlasses='/home/elisabeth/data/siamese_dataset/small_glasses/'

siam = SiameseNetwork(200,200)
siam.createDataset(pathTeo, 'siamese_teo.hdf5')
siam.loadDataset(True,'siamese_teo.hdf5',0.2)
siam.createDataset(pathGlasses, 'siamese_glasses.hdf5')
siam.loadDataset(False, 'siamese_glasses.hdf5',0.2)
print(siam.teoTrainImages[0])



batchSize = 16
embeddingModel, tripletModel = siam.GetModel()

checkpoint_path = "training4/cp-{epoch:04d}.ckpt"
checkpoint_dir = os.path.dirname(checkpoint_path)

# Create a callback that saves the model's weights every 5 epochs
cp_callback = tf.keras.callbacks.ModelCheckpoint(
    filepath=checkpoint_path, 
    verbose=1, 
    save_weights_only=True,
    save_freq=batchSize)


tripletModel.compile(loss=None, optimizer=Adam(0.0001))
tripletModel.save_weights(checkpoint_path.format(epoch=0))

history = tripletModel.fit_generator(siam.getTripletBatch(batchSize,True), 
                           validation_data = siam.getTripletBatch(batchSize,False),
                           epochs=30, 
                           verbose=1, 
                           steps_per_epoch=1000//batchSize, 
                           validation_steps=400//batchSize,
                           callbacks=[cp_callback],
                           use_multiprocessing=False)
print("history")
print(history)
tripletModel.save("models/siamese4",save_format='h5')
embeddingModel.save("models/embedding4", save_format="h5")       

# new_model = tf.keras.models.load_model('models/embedding')
# new_model.summary()

siam.evaPlot(history, 30)




# create_train_data(pathGlasses, 'train_siamese_glasses.npy')

# trainSiameseTeo = np.load('train_siamese_teo.npy')
# trainSiameseGlasses = np.load('train_siamese_glasses.npy')

# print("Teo: ", trainSiameseTeo)
# print("Glasses: ", trainSiameseGlasses)

