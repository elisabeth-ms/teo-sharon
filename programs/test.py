from FeaturesDescriptor import FeaturesDescriptor
import cv2
import matplotlib.pyplot as plt
import numpy as np
image = cv2.imread("/home/elisabeth/data/hog-svm/rice2/crop-rgb/00000010.ppm")

def visualizeHog(im, featuresDescriptor, featureVector):
    
    numCellsX = int(featuresDescriptor.hogWinSize[0]/featuresDescriptor.hogCellSize[0])
    numCellsY = int(featuresDescriptor.hogWinSize[1]/featuresDescriptor.hogCellSize[1])
    
    numBlocksX = int((featuresDescriptor.hogWinSize[0] - featuresDescriptor.hogBlockSize[0] + 
                      featuresDescriptor.hogBlockStride[0])/ featuresDescriptor.hogBlockStride[0])


    
    numBlocksY = int((featuresDescriptor.hogWinSize[1] - featuresDescriptor.hogBlockSize[1] + 
                      featuresDescriptor.hogBlockStride[1])/ featuresDescriptor.hogBlockStride[1])
    print("Num Cells: ", numCellsX, numCellsY)
    print("Num Blocks: ", numBlocksX, numBlocksY)
    
    numCellsInBlockX = int(featuresDescriptor.hogBlockSize[0]/ featuresDescriptor.hogCellSize[0])
    numCellsInBlockY = int(featuresDescriptor.hogBlockSize[1]/ featuresDescriptor.hogCellSize[1])


    num_bins = 9
    max_len = 7  # control sum of segment lengths for visualized histogram bin of each block
    im_h, im_w = im.shape
    num_cell_h, num_cell_w = int(im_h / cell_size), int(im_w / cell_size)
    num_blocks_h, num_blocks_w = num_cell_h - block_size + 1, num_cell_w - block_size + 1
    histo_normalized = hog.reshape((num_blocks_h, num_blocks_w, block_size**2, num_bins))
    histo_normalized_vis = np.sum(histo_normalized**2, axis=2) * max_len  # num_blocks_h x num_blocks_w x num_bins
    angles = np.arange(0, np.pi, np.pi/num_bins)
    mesh_x, mesh_y = np.meshgrid(np.r_[cell_size: cell_size*num_cell_w: cell_size], np.r_[cell_size: cell_size*num_cell_h: cell_size])
    mesh_u = histo_normalized_vis * np.sin(angles).reshape((1, 1, num_bins))  # expand to same dims as histo_normalized
    mesh_v = histo_normalized_vis * -np.cos(angles).reshape((1, 1, num_bins))  # expand to same dims as histo_normalized
    plt.imshow(im, cmap='gray', vmin=0, vmax=1)
    for i in range(num_bins):
        plt.quiver(mesh_x - 0.5 * mesh_u[:, :, i], mesh_y - 0.5 * mesh_v[:, :, i], mesh_u[:, :, i], mesh_v[:, :, i],
                   color='white', headaxislength=0, headlength=0, scale_units='xy', scale=1, width=0.002, angles='xy')
    plt.show()
    
featuresDescriptor = FeaturesDescriptor(hogFeatures=True,hueFeatures=False,hogWinSize=(150,150),
                                        hogBlockSize=(20,20),hogBlockStride=(10,10),hogCellSize=(10,10),
                                        hogNbins=9, hogDerivAperture=1,hogWinSigma=-1,hogHistogramNormType=cv2.HOGDescriptor_L2Hys,
                                        hogL2HysThreshold=0.2, hogGammaCorrection=False,hogNlevels=64,hogSignedGradient=False) 

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
featureVector = featuresDescriptor.getFeatureVector(gray)
print(len(featureVector))

visualizeHog(gray,featuresDescriptor, featureVector)
# for x in featureVector:
#     print(x)