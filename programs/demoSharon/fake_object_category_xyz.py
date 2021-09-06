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
    
    input_port = yarp.Port()
    input_port.open("/fake/rgbdDetection/teoSim/camera/state:o")
    yarp.Network.connect("/rgbdDetection/teoSim/camera/state:o", "/fake/rgbdDetection/teoSim/camera/state:o")
    try:
        while True:
            objectData = yarp.Bottle()
            rgbdDetectionData = yarp.Bottle()
            input_port.read(rgbdDetectionData)
            
            dictObject = objectData.addDict()
            category = 'water'
            dictObject.put('category',category)
            dictObject.put("x", rgbdDetectionData.get(0).find("mmZ").asDouble())
            dictObject.put("y", rgbdDetectionData.get(0).find("mmX").asDouble())
            dictObject.put("z", -rgbdDetectionData.get(0).find("mmY").asDouble())
            
            print(rgbdDetectionData.get(0).find("mmZ").asDouble(),rgbdDetectionData.get(0).find("mmX").asDouble(), -rgbdDetectionData.get(0).find("mmY").asDouble())
            # #0.40316834171675936 -0.20038056908926438 -0.3591530511977783
            #location = [0.608877778053283691406,-0.139089086726210647837, -0.101298844294938308885]
            #
            # dictObject.put("y", location[1])
            # dictObject.put("z", location[2])
            # # bLocation = yarp.Bottle()
            # # bLocation.addDouble(location[0])
            # # bLocation.addDouble(location[1])
            # # bLocation.addDouble(location[2])
            # # objectData.addList(bLocation)
            # print("Sending object category: "+ category + ", location: ",location[0],",",location[1],",", location[2])
    
            fakeObjectDataPort.write(objectData)
    except KeyboardInterrupt:
        sys.exit(1)
    
