import yarp
import sys
import random

if __name__ == "__main__":
    print("main")
    yarp.Network.init()  # connect to YARP network

    if not yarp.Network.checkNetwork():  # let's see if there was actually a reachable YARP network
        print('yarp network is not found!!')   
        sys.exit(1)
    
    fakeObjectDataPort = yarp.Port()
    fakeObjectDataPort.open('/matching/object:o')
    
    
    try:
        while True:
            objectData = yarp.Bottle()
            dictObject = objectData.addDict()
            category = 'milk'
            dictObject.put('category',category)
            
            location = [random.uniform(0.2, 0.5), random.uniform(-0.4, 0), random.uniform(0.09,0.15)]
            dictObject.put("x", location[0])
            dictObject.put("y", location[1])
            dictObject.put("z", location[2])
            # bLocation = yarp.Bottle()
            # bLocation.addDouble(location[0])
            # bLocation.addDouble(location[1])
            # bLocation.addDouble(location[2])
            # objectData.addList(bLocation)
            print("Sending object category: "+ category + ", location: ",location[0],",",location[1],",", location[2])
    
            fakeObjectDataPort.write(objectData)
    except KeyboardInterrupt:
        sys.exit(1)
    