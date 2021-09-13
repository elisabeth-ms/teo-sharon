import yarp
import sys
import random

robot = "/teo"

if __name__ == "__main__":
    print("main")
    yp = yarp.Network()  # connect to YARP network
    yp.init()
    if not yarp.Network.checkNetwork():  # let's see if there was actually a reachable YARP network
        print('yarp network is not found!!')   
        sys.exit(1)
    
    detection_port_name = None
    input_port_name = None
    
    if robot == "/teoSim":
        detection_port_name = "/rgbdDetection/teoSim/camera/state:o"
        input_port_name = "/fake/rgbdDetection/teoSim/camera/state:o"
    elif robot == "/teo":
        input_port_name = "/fake/rgbdDetection/xtion/state:o"
        detection_port_name = "/rgbdDetection/xtion/state:o"
    else:
        print("robot should be: /teo or /teoSim")
        sys.exit(1)
    
    fakeObjectDataPort = yarp.Port()
    fakeObjectDataPort.open('/matching/object:o')
    
    input_port = yarp.BufferedPortBottle()
   
    input_port.open(input_port_name)

    while True:
        try:
            if yp.isConnected(detection_port_name, input_port_name):
                objectData = yarp.Bottle()
                rgbdDetectionData = yarp.Bottle()
                
                rgbdDetectionData=input_port.read(False)
                if rgbdDetectionData is not None:
                    dictObject = objectData.addDict()
                    category = 'water'
                    dictObject.put('category',category)
                    dictObject.put("x", rgbdDetectionData.get(0).find("mmX").asDouble())
                    dictObject.put("y", rgbdDetectionData.get(0).find("mmY").asDouble())
                    dictObject.put("z", rgbdDetectionData.get(0).find("mmZ").asDouble())


                    # dictObject.put("x", rgbdDetectionData.get(0).find("mmZ").asDouble())
                    # dictObject.put("y", rgbdDetectionData.get(0).find("mmX").asDouble())
                    # dictObject.put("z", -rgbdDetectionData.get(0).find("mmY").asDouble())

                    print(rgbdDetectionData.get(0).find("mmZ").asDouble(),rgbdDetectionData.get(0).find("mmX").asDouble(), -rgbdDetectionData.get(0).find("mmY").asDouble())

                    fakeObjectDataPort.write(objectData)
            else:
               
                yp.connect(detection_port_name, input_port_name)
                
        except KeyboardInterrupt:
            yp.disconnect(detection_port_name, input_port_name)
            input_port.close()
            sys.exit(1)
    
