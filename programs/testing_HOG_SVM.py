from skimage.feature import hog
from skimage.transform import pyramid_gaussian
import joblib
from skimage import color
from imutils.object_detection import non_max_suppression
import imutils
import numpy as np
import cv2
import os
import glob
from skimage import img_as_ubyte
from skimage.util import img_as_float
import matplotlib.pyplot as plt

#Define HOG Parameters
# change them if necessary to orientations = 8, pixels per cell = (16,16), cells per block to (1,1) for weaker HOG
orientations = 9
pixels_per_cell = (14, 14)
cells_per_block = (4, 4)
threshold = .4

# define the sliding window:
def sliding_window(image, stepSize, windowSize):# image is the input, step size is the no.of pixels needed to skip and windowSize is the size of the actual window
    # slide a window across the image
    for y in range(0, image.shape[0], stepSize):# this line and the line below actually defines the sliding part and loops over the x and y coordinates
        for x in range(0, image.shape[1], stepSize):
            # yield the current window
            yield (x, y, image[y: y + windowSize[1], x:x + windowSize[0]])

# Upload the saved svm model:

print("Loading model")
model = joblib.load('/home/elisabeth/repos/teo-sharon/programs/model_hog_svm_rice_milk_bottle.npy')

# Test the trained classifier on an image below!
scale = 0
detectionsRice = []
detectionsMilkBottle = []
# read the image you want to detect the object in:
print("Read image where we want to detect the object")
img= cv2.imread("/home/elisabeth/data/01_29_2021/rice/dumper/xtion/rgbImage_o/00000015.ppm")

# Try it with image resized if the image is too big
#img= cv2.resize(img,(300,200)) # can change the size to default by commenting this code out our put in a random number

# defining the size of the sliding window (has to be, same as the size of the image in the training data)
(winW, winH)= (150,150)
windowSize=(winW,winH)
downscale=1.5
# Apply sliding window:
#for resized in pyramid_gaussian(img, downscale=1.5): # loop over each layer of the image that you take!
    # loop over the sliding window for each layer of the pyramid
print("Sliding window to the testing image")

cv_image = img_as_ubyte(img)
gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    # plt.imshow(gray,cmap=plt.cm.gray)
    # plt.show()
norm_img = np.zeros((640,480))
final_img = cv2.normalize(gray,  norm_img, 0, 255, cv2.NORM_MINMAX)
image = img_as_float(final_img)

for (x,y,window) in sliding_window(image, stepSize=15, windowSize=(winW,winH)):
    # if the window does not meet our desired window size, ignore it!
    if window.shape[0] != winH or window.shape[1] !=winW: # ensure the sliding window has met the minimum size requirement
        continue
    fds = hog(window, orientations, pixels_per_cell, cells_per_block, block_norm='L2')  # extract HOG features from the window captured
    fds = fds.reshape(1, -1) # re shape the image to make a silouhette of hog
    pred = model.predict(fds) # use the SVM model to make a prediction on the HOG features extracted from the window
    if pred == 1:
        if model.decision_function(fds)[0][1] > 0.7:  # set a threshold value for the SVM prediction i.e. only firm the predictions above probability of 0.6
            print("Detection rice Location -> ({}, {})".format(x, y))
            print("Scale ->  {} | Confidence Score {} \n".format(scale,model.decision_function(fds)))
            detectionsRice.append((int(x * (downscale**scale)), int(y * (downscale**scale)), model.decision_function(fds)[0][1],
                               int(windowSize[0]*(downscale**scale)), # create a list of all the predictions found
                               int(windowSize[1]*(downscale**scale))))
    if pred == 2:
        if model.decision_function(fds)[0][2] > 0.7:  # set a threshold value for the SVM prediction i.e. only firm the predictions above probability of 0.6
            print("Detection milk bottle Location -> ({}, {})".format(x, y))
            print("Scale ->  {} | Confidence Score {} \n".format(scale,model.decision_function(fds)))
            detectionsMilkBottle.append((int(x * (downscale**scale)), int(y * (downscale**scale)), model.decision_function(fds)[0][2],
                               int(windowSize[0]*(downscale**scale)), # create a list of all the predictions found
                               int(windowSize[1]*(downscale**scale))))
#scale+=1
    
print("Lets check the predictions")
#clone = resized.copy()
if detectionsRice:
    print("Rice found in the image")
    for (x_tl, y_tl, _, w, h) in detectionsRice:
        cv2.rectangle(img, (x_tl, y_tl), (x_tl + w, y_tl + h), (0, 0, 255), thickness = 2)
    rects = np.array([[x, y, x + w, y + h] for (x, y, _, w, h) in detectionsRice]) # do nms on the detected bounding boxes
    sc = [score for (x, y, score, w, h) in detectionsRice]
    print("detection confidence score: ", sc)
    sc = np.array(sc)
    pick = non_max_suppression(rects, probs = sc, overlapThresh = 0.3)

# the peice of code above creates a raw bounding box prior to using NMS
# the code below creates a bounding box after using nms on the detections
# you can choose which one you want to visualise, as you deem fit... simply use the following function:
# cv2.imshow in this right place (since python is procedural it will go through the code line by line).
        
    for (xA, yA, xB, yB) in pick:
        cv2.rectangle(img, (xA, yA), (xB, yB), (0,255,0), 2)
        cv2.putText(img, "rice", (xA, yA),
	    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

if detectionsMilkBottle:
    print("Milk bottle found in the image")
    for (x_tl, y_tl, _, w, h) in detectionsMilkBottle:
        cv2.rectangle(img, (x_tl, y_tl), (x_tl + w, y_tl + h), (0, 0, 255), thickness = 2)
    rects = np.array([[x, y, x + w, y + h] for (x, y, _, w, h) in detectionsMilkBottle]) # do nms on the detected bounding boxes
    sc = [score for (x, y, score, w, h) in detectionsMilkBottle]
    print("detection confidence score: ", sc)
    sc = np.array(sc)
    pick = non_max_suppression(rects, probs = sc, overlapThresh = 0.3)

# the peice of code above creates a raw bounding box prior to using NMS
# the code below creates a bounding box after using nms on the detections
# you can choose which one you want to visualise, as you deem fit... simply use the following function:
# cv2.imshow in this right place (since python is procedural it will go through the code line by line).
        
    for (xA, yA, xB, yB) in pick:
        cv2.rectangle(img, (xA, yA), (xB, yB), (255,0,0), 2)
        cv2.putText(img, "milk bottle", (xA, yA),
	    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 2)
cv2.imshow("Detections", img)

cv2.waitKey(0)
cv2.destroyAllWindows()
