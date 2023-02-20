import numpy as np
import matplotlib.pyplot as plt
import sys
import pickle

def create3DBoundingPointsMilk():
    boundingPoints3D = []
    milkBoxShape = [0.065, 0.075, 0.225]
    milkTriangleSide = 0.014
    milkTriangleHeight = 0.03
    
    p1 = [0, -(milkBoxShape[1]/2.0-milkTriangleSide), milkBoxShape[2]/2.0 + milkTriangleHeight]
    boundingPoints3D.append(p1)
    p2 = [0, +(milkBoxShape[1]/2.0-milkTriangleSide), milkBoxShape[2]/2.0 + milkTriangleHeight]
    boundingPoints3D.append(p2)
    
    p3 = [-milkBoxShape[0]/2.0, -(milkBoxShape[1]/2.0), milkBoxShape[2]/2.0]
    boundingPoints3D.append(p3)
    
    p4 = [-milkBoxShape[0]/2.0, (milkBoxShape[1]/2.0), milkBoxShape[2]/2.0]
    boundingPoints3D.append(p4)
    
    p5 = [milkBoxShape[0]/2.0, (milkBoxShape[1]/2.0), milkBoxShape[2]/2.0]
    boundingPoints3D.append(p5)
    
    p6 = [milkBoxShape[0]/2.0, -(milkBoxShape[1]/2.0), milkBoxShape[2]/2.0]
    boundingPoints3D.append(p6)
    
    p7 = [-milkBoxShape[0]/2.0, -(milkBoxShape[1]/2.0), -milkBoxShape[2]/2.0]
    boundingPoints3D.append(p7)
    
    p8 = [-milkBoxShape[0]/2.0, (milkBoxShape[1]/2.0), -milkBoxShape[2]/2.0]
    boundingPoints3D.append(p8)
    
    p9 = [milkBoxShape[0]/2.0, (milkBoxShape[1]/2.0), -milkBoxShape[2]/2.0]
    boundingPoints3D.append(p9)
    
    p10 = [milkBoxShape[0]/2.0, -(milkBoxShape[1]/2.0), -milkBoxShape[2]/2.0]
    boundingPoints3D.append(p10)
    
    return boundingPoints3D

def create3DBoundingPointsNesquik():
    boundingPoints3D = []
    radius = 0.066
    length = 0.145
    
    thetas = np.linspace(0, 2*np.pi, 50)

    for theta in thetas:
        x = radius*np.cos(theta)
        y = radius*np.sin(theta)
        p=[0,0,0]
        p[0] = x
        p[1] = y
        p[2] = length/2.0
        boundingPoints3D.append(p)
        p=[0,0,0]

        p[0] = x
        p[1] = y
        p[2] = -length/2.0
        boundingPoints3D.append(p)
    
    print(boundingPoints3D)

    return boundingPoints3D   

def create3DBoundingPointsWater():
    boundingPoints3D = []
    radius = 0.04
    length = 0.27
    radiusCap = 0.02
    lengthCap = 0.06
    thetas = np.linspace(0, 2*np.pi, 50)

    for theta in thetas:
        x = radius*np.cos(theta)
        y = radius*np.sin(theta)
        p=[0,0,0]
        p[0] = x
        p[1] = y
        p[2] = (length/2.0)-lengthCap
        boundingPoints3D.append(p)
        p=[0,0,0]

        p[0] = x
        p[1] = y
        p[2] = -length/2.0
        boundingPoints3D.append(p)
    
    for theta in thetas:
        x = radiusCap*np.cos(theta)
        y = radiusCap*np.sin(theta)
        p=[0,0,0]
        p[0] = x
        p[1] = y
        p[2] = (length/2.0)
        boundingPoints3D.append(p)
    
    
    print(boundingPoints3D)
    return boundingPoints3D   

def create3DBoundingPointsCereal():
    boundingPoints3D = []
    cerealBoxShape = [0.08, 0.20, 0.33]

    
    p3 = [-cerealBoxShape[0]/2.0, -cerealBoxShape[1]/2.0, cerealBoxShape[2]/2.0]
    boundingPoints3D.append(p3)
    
    p4 = [-cerealBoxShape[0]/2.0, (cerealBoxShape[1]/2.0), cerealBoxShape[2]/2.0]
    boundingPoints3D.append(p4)
    
    p5 = [cerealBoxShape[0]/2.0, (cerealBoxShape[1]/2.0), cerealBoxShape[2]/2.0]
    boundingPoints3D.append(p5)
    
    p6 = [cerealBoxShape[0]/2.0, -(cerealBoxShape[1]/2.0), cerealBoxShape[2]/2.0]
    boundingPoints3D.append(p6)
    
    p7 = [-cerealBoxShape[0]/2.0, -(cerealBoxShape[1]/2.0), -cerealBoxShape[2]/2.0]
    boundingPoints3D.append(p7)
    
    p8 = [-cerealBoxShape[0]/2.0, (cerealBoxShape[1]/2.0), -cerealBoxShape[2]/2.0]
    boundingPoints3D.append(p8)
    
    p9 = [cerealBoxShape[0]/2.0, (cerealBoxShape[1]/2.0), -cerealBoxShape[2]/2.0]
    boundingPoints3D.append(p9)
    
    p10 = [cerealBoxShape[0]/2.0, -(cerealBoxShape[1]/2.0), -cerealBoxShape[2]/2.0]
    boundingPoints3D.append(p10)
    
    return boundingPoints3D

    

def plot3DPoints(boundingPoints3D):
    fig = plt.figure()
    xs = []
    ys = []
    zs = []
    for i in range(len(boundingPoints3D)):
        xs.append(boundingPoints3D[i][0])
        ys.append(boundingPoints3D[i][1])
        zs.append(boundingPoints3D[i][2])

    ax = fig.add_subplot(projection='3d')
    ax.scatter(xs, ys, zs)
    #ax.set_box_aspect((np.ptp(xs), np.ptp(ys), np.ptp(zs)))  # aspect ratio is 1:1:1 in data space

    plt.show()
    
def saveInPklFile(name, boundingPoints3D):
    with open(name,'wb') as f:
        pickle.dump(boundingPoints3D, f)

if __name__ == "__main__":
    try:
        objectCategory = str(sys.argv[1])
    except:
        print('create3DBoundingPoints.py category')
    
    if objectCategory == 'milk':
        boundingPoints3D = create3DBoundingPointsMilk()
    elif objectCategory == 'nesquik':
        boundingPoints3D = create3DBoundingPointsNesquik()
    elif objectCategory == 'water':
        boundingPoints3D = create3DBoundingPointsWater()
    elif objectCategory == 'cereal':
        boundingPoints3D = create3DBoundingPointsCereal()
        
    plot3DPoints(boundingPoints3D)
    saveInPklFile(objectCategory+'.pkl', boundingPoints3D)
        
    with open(objectCategory+'.pkl','rb') as f:
        X = pickle.load(f)
        print(X)