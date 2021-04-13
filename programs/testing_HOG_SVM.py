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
from sklearn.preprocessing import StandardScaler
from FeaturesDescriptor import FeaturesDescriptor

# Define HOG Parameters
# change them if necessary to orientations = 8, pixels per cell = (16,16), cells per block to (1,1) for weaker HOG
orientations = 10
pixels_per_cell = (15, 15)
cells_per_block = (10, 10)
detectionsRice = []
detectionsMilkBottle = []
# define the sliding window:




featuresDescriptor = FeaturesDescriptor(hogFeatures = True, hueFeatures = False, hogWinSize = (150,150), hogBlockSize =(30,30),
                 hogBlockStride = (15,15),hogCellSize = (15,15), hogNbins = 10, hogDerivAperture = 1, 
                 hogWinSigma = -1, hogHistogramNormType = cv2.HOGDescriptor_L2Hys, hogL2HysThreshold = 0.2,
                 hogGammaCorrection = True, hogNlevels = 64, hogSignedGradient = False, hueRange = (0,180),
                 hueHistize = 16)

# image is the input, step size is the no.of pixels needed to skip and windowSize is the size of the actual window
def sliding_window(image, stepSize, windowSize):
    # slide a window across the image
    # this line and the line below actually defines the sliding part and loops over the x and y coordinates
    for y in range(0, image.shape[0], stepSize):
        for x in range(0, image.shape[1], stepSize):
            # yield the current window
            yield (x, y, image[y: y + windowSize[1], x:x + windowSize[0]])

# Upload the saved svm model:


print("Loading model")
model = joblib.load(
    '/home/elisabeth/repos/teo-sharon/programs/model_hog_svm_rice_milk_bottle.npy')
scaler = joblib.load(
    '/home/elisabeth/repos/teo-sharon/programs/model_scaler.npy')
# Test the trained classifier on an image below!
scale = 0

# read the image you want to detect the object in:
print("Read image where we want to detect the object")

# Path validation input dataset
im_validation_path = r"/home/elisabeth/data/hog-svm/rice2/validation/rgb"
# it will read all the files in the positive image path (so all the required images)
im_validation_listing = os.listdir(im_validation_path)

for file in im_validation_listing:  # this loop enables reading the files in the pos_im_listing variable one by one
    img = cv2.imread(im_validation_path + '/' + file)
    print(file)
    cv2.imshow("Image", img)
    cv2.waitKey(0)
    # Try it with image resized if the image is too big
    # img= cv2.resize(img,(300,200)) # can change the size to default by commenting this code out our put in a random number

    # defining the size of the sliding window (has to be, same as the size of the image in the training data)
    (winW, winH) = (150, 150)
    windowSize = (winW, winH)
    downscale = 1.5
    # Apply sliding window:
    # for resized in pyramid_gaussian(img, downscale=1.5): # loop over each layer of the image that you take!
    # loop over the sliding window for each layer of the pyramid
    print("Sliding window to the testing image")
    cv_image = img_as_ubyte(img)
    gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
    norm_img = np.zeros((150, 150))
    final_img = cv2.equalizeHist(gray)


    image = img_as_float(gray)


    detectionsRice = []
    detectionsMilkBottle = []
    # for resized in pyramid_gaussian(image,max_layer=2, downscale=1.5):

    for (x, y, window) in sliding_window(gray, stepSize=10, windowSize=(winW, winH)):

        # if the window does not meet our desired window size, ignore it!
       # ensure the sliding window has met the minimum size requirement
        if window.shape[0] != winH or window.shape[1] != winW:
            continue
        # if y < 200:
        #     continue
        
        fd = featuresDescriptor.getFeatureVector(window)

        # fds = hog(window, orientations, pixels_per_cell, cells_per_block,
        #           block_norm='L2')  # extract HOG features from the window captured
        # re shape the image to make a silouhette of hog
        fd = fd.reshape(1, -1)
        # use the SVM model to make a prediction on the HOG features extracted from the window
        fd = scaler.transform(fd)
        pred = model.predict(fd)

        print(pred[0])
        if pred[0] == 1:
            print("Pred: ", pred[0], "Decision: ",
                  model.decision_function(fd))
            # print("rice")
            # print(model.decision_function(fds))
            # set a threshold value for the SVM prediction i.e. only firm the predictions above probability of 0.6
            if model.decision_function(fd)[0][1] > 0.8:
                print("Detection rice Location -> ({}, {})".format(x, y))
                print("Scale ->  {} | Confidence Score {} \n".format(scale,
                                                                     model.decision_function(fd)))
                detectionsRice.append((int(x * (downscale**scale)), int(y * (downscale**scale)), model.decision_function(fd)[0][1],
                                       # create a list of all the predictions found
                                       int(windowSize[0] * \
                                           (downscale**scale)),
                                       int(windowSize[1]*(downscale**scale))))
        if pred[0] == 2:
            # set a threshold value for the SVM prediction i.e. only firm the predictions above probability of 0.6
            print('milkbottle')
            print (model.decision_function(fd))
            if model.decision_function(fd)[0][2] > 1.5:
                print("Detection milk bottle Location -> ({}, {})".format(x, y))
                print("Scale ->  {} | Confidence Score {} \n".format(scale,
                                                                         model.decision_function(fd)))
                detectionsMilkBottle.append((int(x * (downscale**scale)), int(y * (downscale**scale)), model.decision_function(fd)[0][2],
                                                 # create a list of all the predictions found
                                                 int(windowSize[0] * \
                                                     (downscale**scale)),
                                                 int(windowSize[1]*(downscale**scale))))
        # scale+=1

    print("Lets check the predictions")
    # clone = resized.copy()
    if detectionsRice:
        print("Rice found in the image")
        print(len(detectionsRice))
        for (x_tl, y_tl, _, w, h) in detectionsRice:
            cv2.rectangle(img, (x_tl, y_tl), (x_tl + w, y_tl + h),
                          (0, 0, 255), thickness=2)
        # do nms on the detected bounding boxes
        rects = np.array([[x, y, x + w, y + h]
                          for (x, y, _, w, h) in detectionsRice])
        sc = [score for (x, y, score, w, h) in detectionsRice]
        sc = np.array(sc)
        print("rects: ", rects)
        print("detection confidence score: ", sc)

        for (xA, yA, xB, yB) in rects:
            cv2.rectangle(img, (xA, yA), (xB, yB), (0, 255, 0), 2)
        pick = non_max_suppression(rects, probs=sc, overlapThresh=0.2)

    # the peice of code above creates a raw bounding box prior to using NMS
    # the code below creates a bounding box after using nms on the detections
    # you can choose which one you want to visualise, as you deem fit... simply use the following function:
    # cv2.imshow in this right place (since python is procedural it will go through the code line by line).

        for (xA, yA, xB, yB) in pick:
            cv2.rectangle(img, (xA, yA), (xB, yB), (0, 255, 0), 2)
            cv2.putText(img, "rice", (xA, yA),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)

        detectionsRice = []

    if detectionsMilkBottle:
        print("Milk bottle found in the image")
        for (x_tl, y_tl, _, w, h) in detectionsMilkBottle:
            cv2.rectangle(img, (x_tl, y_tl), (x_tl + w, y_tl + h),
                          (0, 0, 255), thickness=2)
        # do nms on the detected bounding boxes
        rects = np.array([[x, y, x + w, y + h]
                          for (x, y, _, w, h) in detectionsMilkBottle])
        sc = [abs(score) for (x, y, score, w, h) in detectionsMilkBottle]
        print("detection confidence score: ", sc)
        sc = np.array(sc)
        pick = non_max_suppression(rects, probs=sc, overlapThresh=0.5)

    # the peice of code above creates a raw bounding box prior to using NMS
    # the code below creates a bounding box after using nms on the detections
    # you can choose which one you want to visualise, as you deem fit... simply use the following function:
    # cv2.imshow in this right place (since python is procedural it will go through the code line by line).

        for (xA, yA, xB, yB) in pick:
            cv2.rectangle(img, (xA, yA), (xB, yB), (255, 0, 0), 2)
            cv2.putText(img, "milk bottle", (xA, yA),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 0, 0), 2)
    cv2.imshow("Detections", img)

    im_detections_path = r"/home/elisabeth/data/hog-svm/rice2/validation/detections"

    cv2.imwrite(im_detections_path+'/'+file, img)
    cv2.waitKey(0)

    cv2.destroyAllWindows()
