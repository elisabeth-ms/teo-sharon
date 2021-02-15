# Importing the necessary modules:
from skimage.feature import hog
from skimage.transform import pyramid_gaussian
from skimage.io import imread
import joblib
from sklearn.preprocessing import LabelEncoder
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
from PIL import Image # This will be used to read/modify images (can be done via OpenCV too)
from numpy import *
from skimage import data, exposure
import matplotlib.pyplot as plt
from skimage import img_as_ubyte

from skimage.util import img_as_float
from sklearn.preprocessing import StandardScaler

# define parameters of HOG feature extraction
orientations = 9
pixels_per_cell = (14, 14)
cells_per_block = (4, 4)
threshold = .4


# define path to images:

im_rice_path = r"/home/elisabeth/data/hog-svm/rice/crop-rgb" # Path rice input dataset
im_milk_bottle_path = r"/home/elisabeth/data/hog-svm/milk-bottle/crop-rgb" # Path milk bottle input dataset

# define the same for negatives
neg_im_path= r"/home/elisabeth/data/hog-svm/background/crop-rgb"

# read the image files:
im_rice_listing = os.listdir(im_rice_path) # it will read all the files in the positive image path (so all the required images)
im_milk_bottle_listing = os.listdir(im_milk_bottle_path)
neg_im_listing = os.listdir(neg_im_path)
num_rice_samples = size(im_rice_listing) # Total no. of rice images
num_milk_bottle_samples = size(im_milk_bottle_listing)
num_neg_samples = size(neg_im_listing)
print(num_rice_samples) # prints the number value of the no.of samples in rice dataset
print(num_milk_bottle_samples) # prints the number value of the no.of samples in milk bottle dataset
print(num_neg_samples)
data= []
labels = []

# compute HOG features and label them:

for file in im_rice_listing: #this loop enables reading the files in the pos_im_listing variable one by one
    img = Image.open(im_rice_path + '/' + file) # open the file
    #img = img.resize((64,128))
    cv_image = img_as_ubyte(img)
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    norm_img = np.zeros((150,150))
    final_img = cv2.normalize(gray,  norm_img, 0, 255, cv2.NORM_MINMAX)
    image = img_as_float(final_img)

    # calculate HOG for positive features
    fd,hog_image = hog(image, orientations, pixels_per_cell, cells_per_block,visualize=True, block_norm='L2', feature_vector=True)# fd= feature descriptor
    fd.reshape(1, -1) 
    print(len(fd))
    
    # fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 4), sharex=True, sharey=True)

    # ax1.axis('off')
    # ax1.imshow(gray, cmap=plt.cm.gray)
    # ax1.set_title('Input image')

    # # Rescale histogram for better display
    # hog_image_rescaled = exposure.rescale_intensity(hog_image, in_range=(0, 10))

    # ax2.axis('off')
    # ax2.imshow(hog_image_rescaled, cmap=plt.cm.gray)
    # ax2.set_title('Histogram of Oriented Gradients')
    #plt.show()
    print(fd)
    data.append(fd)
    labels.append(1)

for file in im_milk_bottle_listing: #this loop enables reading the files in the pos_im_listing variable one by one
    img = Image.open(im_milk_bottle_path + '/' + file) # open the file
    #img = img.resize((64,128))
    cv_image = img_as_ubyte(img)
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

    norm_img = np.zeros((150,150))
    final_img = cv2.normalize(gray,  norm_img, 0, 255, cv2.NORM_MINMAX)
    image = img_as_float(final_img)

    # calculate HOG for positive features
    fd,hog_image = hog(image, orientations, pixels_per_cell, cells_per_block,visualize=True, block_norm='L2', feature_vector=True)# fd= feature descriptor
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
    #plt.show()
    print(fd)
    data.append(fd)
    labels.append(2)
    
# Same for the negative images
for file in neg_im_listing:
    img= Image.open(neg_im_path + '/' + file)
    cv_image = img_as_ubyte(img)
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    norm_img = np.zeros((150,150))
    final_img = cv2.normalize(gray,  norm_img, 0, 255, cv2.NORM_MINMAX)
    image = img_as_float(final_img)
    plt.figure()
    plt.imshow(final_img, cmap=plt.cm.gray)
    #plt.show()

    # Now we calculate the HOG for negative features
    fd,hog_image = hog(image, orientations, pixels_per_cell, cells_per_block,visualize=True, block_norm='L2', feature_vector=True)
    print(fd)
    fd.reshape(1, -1) 
    
    data.append(fd)
    labels.append(0)

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(8, 4), sharex=True, sharey=True)

    ax1.axis('off')
    ax1.imshow(gray, cmap=plt.cm.gray)
    ax1.set_title('Input image')

    # Rescale histogram for better display
    hog_image_rescaled = exposure.rescale_intensity(hog_image, in_range=(0, 10))

    ax2.axis('off')
    ax2.imshow(hog_image_rescaled, cmap=plt.cm.gray)
    ax2.set_title('Histogram of Oriented Gradients')
    #plt.show()


# encode the labels, converting them from strings to integers
le = LabelEncoder()
labels = le.fit_transform(labels)

#%%
# Partitioning the data into training and testing splits, using 80%
# of the data for training and the remaining 20% for testing
print(" Constructing training/testing split...")
(trainData, testData, trainLabels, testLabels) = train_test_split(
	np.array(data), labels, test_size=0.5, random_state=42)
#%% Train the linear SVM
print(" Training Linear SVM classifier...")
model = LinearSVC()
model.fit(trainData, trainLabels)
#%% Evaluate the classifier
print(" Evaluating classifier on test data ...")
predictions = model.predict(testData)

print(predictions)
print(testLabels)
print(classification_report(testLabels, predictions))


# Save the model:
#%% Save the Model
joblib.dump(model, 'model_hog_svm_rice_milk_bottle.npy')