import yarp
import sys
import random

if __name__ == "__main__":
    print("main")
    yarp.Network.init()  # connect to YARP network

    if not yarp.Network.checkNetwork():  # let's see if there was actually a reachable YARP network
        print('yarp network is not found!!')   
        sys.exit(1)
    
    matching_object_port = yarp.Port()
    matching_object_port.open('/spawn/matching/object:i')
    yarp.Network.connect('/matching/object:o', '/spawn/matching/object:i')
    
    world_interface_client = yarp.RpcClient()
    world_interface_client.open("/test/world_interface")
    world_interface_client.addOutput("/world_input_port")
    

    object_data = yarp.Bottle()
    matching_object_port.read(object_data)
    category = object_data.get(0).find('category').asString()
    locationTCP = [object_data.get(0).find("x").asDouble(), object_data.get(0).find("y").asDouble(), object_data.get(0).find("z").asDouble()]
    print(category, locationTCP)
    
    # Check if we already have a model of the object in Gazebo
    model_present = False
    
    cmd = yarp.Bottle()
    rep = yarp.Bottle()
    cmd.addString("getPose")
    cmd.addString(category)
    world_interface_client.write(cmd, rep)
    model_present = not all(rep.get(i).asDouble() == 0 for i in range(6))
    print(model_present)
    
    if not model_present:
        if category == 'water':
            cmd = yarp.Bottle()
            rep = yarp.Bottle()
            cmd.addString("makeCylinder")
            cmd.addDouble(0.04) # radius [mt]
            cmd.addDouble(0.320)  # lenght x  [mt]
            cmd.addDouble(locationTCP[0])  # pos x  [mt]
            cmd.addDouble(locationTCP[1])  # pos y	    
            
            cmd.addDouble(locationTCP[2]+0.845)  # pos z
            cmd.addDouble(0.0)  # pose.roll  [rad] 
            cmd.addDouble(0.0)  # pose.pitch [rad]	    
            cmd.addDouble(0.0)  # pose.yaw   [rad]
            cmd.addInt(0)       # color.r 
            cmd.addInt(0)     # color.g 
            cmd.addInt(255)       # color.b
            cmd.addString('')
            cmd.addString('water')
        elif category == 'milk':
            cmd = yarp.Bottle()
            rep = yarp.Bottle()
            cmd.addString("makeBox")
            cmd.addDouble(0.195) # radius [mt]
            cmd.addDouble(0.08)  # lenght x  [mt]
            cmd.addDouble(0.340)
            cmd.addDouble(locationTCP[0])  # pos x  [mt]
            cmd.addDouble(locationTCP[1])  # pos y	    
            cmd.addDouble(locationTCP[2]+0.845)  # pos z
            cmd.addDouble(0.0)  # pose.roll  [rad] 
            cmd.addDouble(0.0)  # pose.pitch [rad]	    
            cmd.addDouble(0.0)  # pose.yaw   [rad]
            cmd.addInt(0)       # color.r 
            cmd.addInt(0)     # color.g 
            cmd.addInt(255)       # color.b 
        world_interface_client.write(cmd, rep)
    else:
        print("Lets modify the position of the object ", category)
        cmd = yarp.Bottle()
        rep = yarp.Bottle()
        cmd.addString('setPose')
        cmd.addString(category)
        cmd.addDouble(locationTCP[0])
        cmd.addDouble(locationTCP[1])
        cmd.addDouble(locationTCP[2]+0.845)
        cmd.addDouble(0)
        cmd.addDouble(0)
        cmd.addDouble(0)
        world_interface_client.write(cmd, rep)