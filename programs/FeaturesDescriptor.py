import cv2
import numpy as np

class FeaturesDescriptor:
    """ Class to create a combined feature vector from Hog features and hue histogram features """
    def __init__(self, hogFeatures = False, hueFeatures = False, hogWinSize = (150,150), hogBlockSize =(20,20),
                 hogBlockStride = (10,10),hogCellSize = (10,10), hogNbins = 9, hogDerivAperture = 1, 
                 hogWinSigma = -1, hogHistogramNormType = cv2.HOGDescriptor_L2Hys, hogL2HysThreshold = 0.2,
                 hogGammaCorrection = False, hogNlevels = 32, hogSignedGradient = False, hueRange = (0,180),
                 hueHistize = 16):
        
        
        self.hogFeatures = hogFeatures
        self.hueFeatures = hueFeatures
        self.hogWinSize = hogWinSize
        self.hogBlockSize = hogBlockSize
        self.hogBlockStride = hogBlockStride
        self.hogCellSize = hogCellSize
        self.hogNbins = hogNbins
        self.hogDerivAperture = hogDerivAperture
        self.hogWinSigma = hogWinSigma
        self.hogHistogramNormType = hogHistogramNormType
        self.hogL2HysThreshold = hogL2HysThreshold
        self.hogGammaCorrection = hogGammaCorrection
        self.hogNlevels = hogNlevels
        self.hogSignedGradient = hogSignedGradient
        self.hueRange = hueRange
        self.hueHistize = hueHistize
        
        if self.hogFeatures:
            self.hogDescriptor = cv2.HOGDescriptor(hogWinSize, hogBlockSize, hogBlockStride,
                                                    hogCellSize, hogNbins, hogDerivAperture,
                                                    hogWinSigma, hogHistogramNormType,hogL2HysThreshold,
                                                    hogGammaCorrection, hogNlevels, hogSignedGradient)      
        
    
    def getFeatureVector(self, image):
        """ Return the combined feature vector from an image"""
        
        featureVector = np.array([])
        
        if self.hogFeatures:
            featureVector = self.hogDescriptor.compute(image)[:,0]

        
        return featureVector        
