# Importing the necessary modules:
from skimage.feature import hog
from skimage.transform import pyramid_gaussian
from skimage.io import imread
import joblib
from sklearn.preprocessing import LabelEncoder
from sklearn.linear_model import SGDClassifier
from sklearn.svm import LinearSVC

from sklearn.metrics import classification_report
from sklearn.model_selection import train_test_split
from skimage import color
from imutils.object_detection import non_max_suppression
import imutils
import numpy as np
import argparse
import cv2
import os
import glob
# This will be used to read/modify images (can be done via OpenCV too)
from PIL import Image
from numpy import *
from skimage import data, exposure
import matplotlib.pyplot as plt
from skimage import img_as_ubyte

from skimage.util import img_as_float
from sklearn.preprocessing import StandardScaler
from sklearn.multiclass import OneVsRestClassifier

from FeaturesDescriptor import FeaturesDescriptor
# define parameters of HOG feature extraction
orientations = 10
pixels_per_cell = (15, 15)
cells_per_block = (10, 10)


# define path to images:

im_rice_path = r"/home/elisabeth/data/hog-svm/rice2/crop-rgb"  # Path rice input dataset
im_rice_mask_path = r""
# Path milk bottle input dataset
im_milk_bottle_path = r"/home/elisabeth/data/hog-svm/milk-bottle1/crop-rgb"

# define the same for negatives
neg_im_path = r"/home/elisabeth/data/hog-svm/background/crop-rgb"

# read the image files:
# it will read all the files in the positive image path (so all the required images)
im_rice_listing = os.listdir(im_rice_path)
im_milk_bottle_listing = os.listdir(im_milk_bottle_path)
neg_im_listing = os.listdir(neg_im_path)
num_rice_samples = size(im_rice_listing)  # Total no. of rice images
num_milk_bottle_samples = size(im_milk_bottle_listing)
num_neg_samples = size(neg_im_listing)
# prints the number value of the no.of samples in rice dataset
print(num_rice_samples)
# prints the number value of the no.of samples in milk bottle dataset
print(num_milk_bottle_samples)
print(num_neg_samples)
data = []
labels = []



featuresDescriptor = FeaturesDescriptor(hogFeatures = True, hueFeatures = False, hogWinSize = (150,150), hogBlockSize =(30,30),
                 hogBlockStride = (15,15),hogCellSize = (15,15), hogNbins = 10, hogDerivAperture = 1, 
                 hogWinSigma = -1, hogHistogramNormType = cv2.HOGDescriptor_L2Hys, hogL2HysThreshold = 0.2,
                 hogGammaCorrection = True, hogNlevels = 64, hogSignedGradient = False, hueRange = (0,180),
                 hueHistize = 16)

# Same for the negative images
for file in neg_im_listing:
    img = Image.open(neg_im_path + '/' + file)  # open the file
    #img = img.resize((64,128))
    cv_image = img_as_ubyte(img)
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    norm_img = np.zeros((150, 150))
    final_img = cv2.equalizeHist(gray)


    image = img_as_float(gray)
    plt.figure()
    plt.imshow(final_img, cmap=plt.cm.gray)
    #plt.show()

    # Now we calculate the HOG for negative features
    #fd,hog_image = hog(image, orientations, pixels_per_cell, cells_per_block,visualize=True, block_norm='L2', feature_vector=True)
    fd = featuresDescriptor.getFeatureVector(gray)
    print(fd)
    fd.reshape(1, -1)

    data.append(fd)
    labels.append(0)

    # fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 4), sharex=True, sharey=True)

    # ax1.axis('off')
    # ax1.imshow(gray, cmap=plt.cm.gray)
    # ax1.set_title('Input image')

    # Rescale histogram for better display
    #hog_image_rescaled = exposure.rescale_intensity(hog_image, in_range=(0, 10))

    # ax2.axis('off')
    # ax2.imshow(hog_image_rescaled, cmap=plt.cm.gray)
    # ax2.set_title('Histogram of Oriented Gradients')
    #plt.show()

# compute HOG features and label them:

for file in im_rice_listing:  # this loop enables reading the files in the pos_im_listing variable one by one
    img = Image.open(im_rice_path + '/' + file)  # open the file
    #img = img.resize((64,128))
    cv_image = img_as_ubyte(img)
    
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    final_img = cv2.equalizeHist(gray)

    image = img_as_float(gray)

    # # calculate HOG for positive features
    # fd, hog_image = hog(image, orientations, pixels_per_cell, cells_per_block, visualize=True,
    #                     block_norm='L2', feature_vector=True)  # fd= feature descriptor
    
    fd = featuresDescriptor.getFeatureVector(gray)
    fd.reshape(1, -1)
    print(len(fd))

    # fig, (ax1, ax2) = plt.subplots(
    #     1, 2, figsize=(8, 4), sharex=True, sharey=True)

    # ax1.axis('off')
    # ax1.imshow(gray, cmap=plt.cm.gray)
    # ax1.set_title('Input image')

    # # Rescale histogram for better display
    # hog_image_rescaled = exposure.rescale_intensity(
    #     hog_image, in_range=(0, 10))

    # ax2.axis('off')
    # ax2.imshow(hog_image_rescaled, cmap=plt.cm.gray)
    # ax2.set_title('Histogram of Oriented Gradients')
    #plt.show()
    print(fd)
    data.append(fd)
    labels.append(1)

# this loop enables reading the files in the pos_im_listing variable one by one
for file in im_milk_bottle_listing:
    img = Image.open(im_milk_bottle_path + '/' + file)  # open the file
    img = img.resize((150,150))
    cv_image = img_as_ubyte(img)
    cv2.imshow("milk", cv_image)
    cv2.waitKey(0)
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    norm_img = np.zeros((150, 150))
    #final_img = cv2.normalize(gray,  norm_img, 0, 255, cv2.NORM_MINMAX)
    final_img = cv2.equalizeHist(gray)

    image = img_as_float(final_img)

    # calculate HOG for positive features
    fd = featuresDescriptor.getFeatureVector(gray)

    # fd, hog_image = hog(image, orientations, pixels_per_cell, cells_per_block, visualize=True,
    #                     block_norm='L2',  feature_vector=True)  # fd= feature descriptor
    fd.reshape(1, -1)

    # fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 4), sharex=True, sharey=True)

    # ax1.axis('off')
    # ax1.imshow(gray, cmap=plt.cm.gray)
    # ax1.set_title('Input image')

    # # Rescale histogram for better display
    # hog_image_rescaled = exposure.rescale_intensity(hog_image, in_range=(0, 10))

    # ax2.axis('off')
    # ax2.imshow(hog_image_rescaled, cmap=plt.cm.gray)
    # ax2.set_title('Histogram of Oriented Gradients')
    # plt.show()
    print(fd)
    data.append(fd)
    labels.append(2)

# encode the labels, converting them from strings to integers
le = LabelEncoder()
labels = le.fit_transform(labels)

# %%
# Partitioning the data into training and testing splits, using 80%
# of the data for training and the remaining 20% for testing
print(" Constructing training/testing split...")
(trainData, testData, trainLabels, testLabels) = train_test_split(
    np.array(data), labels, test_size=0.2, random_state=42,shuffle=True)

print(testData)
# %% Train the linear SVM
print(" Training Linear SVM classifier...")
scaler = StandardScaler()
trainData = scaler.fit_transform(trainData)

joblib.dump(scaler, 'model_scaler.npy')


model = OneVsRestClassifier(LinearSVC(C=100, loss='squared_hinge', penalty='l2',
        fit_intercept=False, random_state=42,max_iter=2000,dual=True))
model.fit(trainData, trainLabels)

# %% Evaluate the classifier
print(" Evaluating classifier on test data ...")

testData = scaler.transform(testData)
predictions = model.predict(testData)
print(model.decision_function(trainData))
print(predictions[0])
print(testLabels[0])
print(classification_report(testLabels, predictions))


# Save the model:
# %% Save the Model
joblib.dump(model, 'model_hog_svm_rice_milk_bottle.npy')
