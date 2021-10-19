// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "GetGraspingPoses.hpp"

#include <cstdio>

#include <ColorDebug.h>

using namespace sharon;

/************************************************************************/

bool GetGraspingPoses::configure(yarp::os::ResourceFinder & rf)
{
    std::string strRGBDDevice = DEFAULT_RGBD_DEVICE;
    std::string strRGBDLocal = DEFAULT_RGBD_LOCAL;
    std::string strRGBDRemote = DEFAULT_RGBD_REMOTE;
    watchdog = DEFAULT_WATCHDOG;  // double

    std::string robot = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"name of /robot to be used").asString();

    printf("GetGraspingPoses options:\n");
    printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
    printf("\t--RGBDDevice (device we create, default: \"%s\")\n",strRGBDDevice.c_str());
    printf("\t--RGBDLocal (if accesing remote, local port name, default: \"%s\")\n",strRGBDLocal.c_str());
    printf("\t--RGBDRemote (if accesing remote, remote port name, default: \"%s\")\n",strRGBDRemote.c_str());
    printf("\t--watchdog ([s] default: \"%f\")\n",watchdog);


    if(rf.check("RGBDDevice")) strRGBDDevice = rf.find("RGBDDevice").asString();
    if(rf.check("RGBDLocal")) strRGBDLocal = rf.find("RGBDLocal").asString();
    if(rf.check("RGBDRemote")) strRGBDRemote = rf.find("RGBDRemote").asString();
    if(rf.check("watchdog")) watchdog = rf.find("watchdog").asFloat64();

    printf("RgbdDetection using RGBDDevice: %s, RGBDLocal: %s, RGBDRemote: %s.\n",
           strRGBDDevice.c_str(), strRGBDLocal.c_str(), strRGBDRemote.c_str());
    printf("RgbdDetection using watchdog: %f.\n",watchdog);

    yarp::os::Property options;
    options.fromString( rf.toString() );  //-- Should get noMirror, noRGBMirror, noDepthMirror, video modes...
    options.put("device",strRGBDDevice);  //-- Important to override in case there is a "device" in the future
    options.put("localImagePort",strRGBDLocal+"/rgbImage:i");
    options.put("localDepthPort",strRGBDLocal+"/depthImage:i");
    options.put("localRpcPort",strRGBDLocal+"/rpc:o");
    options.put("remoteImagePort",strRGBDRemote+"/rgbImage:o");
    options.put("remoteDepthPort",strRGBDRemote+"/depthImage:o");
    options.put("remoteRpcPort",strRGBDRemote+"/rpc:i");
    //if(rf.check("noMirror")) options.put("noMirror",1);  //-- Replaced by options.fromString( rf.toString() );

    if(!dd.open(options))
    {
        yError("Bad RGBDDevice \"%s\"...\n",strRGBDDevice.c_str());
        return false;
    }
    yInfo("RGBDDevice available.\n");

    if (! dd.view(iRGBDSensor) )
    {
        yError("RGBDDevice bad view.\n");
        return false;
    }
    yInfo("RGBDDevice ok view.\n");

    //-----------------OPEN LOCAL PORTS------------//
    std::string portPrefix("/getGraspingPoses");
    portPrefix += strRGBDRemote;
    if(!outRgbImgPort.open(portPrefix + "/croppedImg:o"))
    {
        yError("Bad outRgbImgPort.open\n");
        return false;
    }
    if(!outDepthImgPort.open(portPrefix + "/croppedDepthImg:o"))
    {
        yError("Bad outDepthImgPort.open\n");
        return false;
    }

    if(!outPointCloudPort.open(portPrefix + "/pointCloud:o"))
    {
        yError("Bad outPointCloudPort.open\n");
        return false;
    }

    // if(!inMarchingObjectDataPort.open(portPrefix + "/matching/object:i"))
    // {
    //     yError("Bad inMarchingObjectDataPort.open\n");
    //     return false;
    // }

    // if(!outGraspingPosesPort.open(portPrefix + "/graspingPoses:o")){
    //     yError("Bad outGraspingPosesPort.open\n");
    //     return false;
    // }

    yarp::os::Property headOptions;
    headOptions.put("device","remote_controlboard");
    headOptions.put("remote",robot+"/head");
    headOptions.put("local","/getGraspingPoses"+robot+"/head");
    headDevice.open(headOptions);

    if( ! headDevice.isValid() )
    {
        printf("head remote_controlboard instantiation not worked.\n");
        return false;
    }


    if( ! headDevice.view(iHeadEncoders) )
    {
        printf("view(iEncoders) not worked.\n");
        return false;
    }


    if (!headDevice.view(headIControlLimits))
    {
        printf("Could not view iControlLimits.\n");
        return false;
    }


    yarp::os::Property trunkOptions;
    trunkOptions.put("device","remote_controlboard");
    trunkOptions.put("remote",robot+"/trunk");
    trunkOptions.put("local","/getGraspingPoses"+robot+"/trunk");
    trunkDevice.open(trunkOptions);

    if( ! trunkDevice.isValid() )
    {
        printf("trunk remote_controlboard instantiation not worked.\n");
        return false;
    }


    if( ! trunkDevice.view(iTrunkEncoders) )
    {
        printf("view(iEncoders) not worked.\n");
        return false;
    }

    if (!trunkDevice.view(trunkIControlLimits))
    {
        printf("Could not view iControlLimits.\n");
        return false;
    }

    yarp::os::Bottle qrMin;
    yarp::os::Bottle qrMax;


    for (unsigned int joint = 0; joint < 2; joint++)
    {
        double min, max;
        trunkIControlLimits->getLimits(joint, &min, &max);
        qrMin.addDouble(min);
        qrMax.addDouble(max);
        yInfo("Joint %d limits: [%f,%f]", joint, min, max);
    }

    for (unsigned int joint = 0; joint < 2; joint++)
    {
        double min, max;
        headIControlLimits->getLimits(joint, &min, &max);
        qrMin.addDouble(min);
        qrMax.addDouble(max);
        yInfo("Joint %d limits: [%f,%f]", joint, min, max);
    }

    yarp::os::Property trunkAndHeadSolverOptions;

    std::string trunkHeadKinPath = rf.findFileByName("teo-trunk-head.ini");
    trunkAndHeadSolverOptions.fromConfigFile(trunkHeadKinPath);
    trunkAndHeadSolverOptions.put("device","KdlSolver");
    trunkAndHeadSolverOptions.put("mins", yarp::os::Value::makeList(qrMin.toString().c_str()));
    trunkAndHeadSolverOptions.put("maxs", yarp::os::Value::makeList(qrMax.toString().c_str()));
    trunkAndHeadSolverOptions.put("ik", "st"); // to use screw theory IK
    trunkAndHeadSolverDevice.open(trunkAndHeadSolverOptions);

    if( ! trunkAndHeadSolverDevice.isValid() )
    {
        yError() << "KDLSolver solver device for trunk and head is not valid";
        return false;
    }

    if( ! trunkAndHeadSolverDevice.view(trunkAndHeadSolverDeviceICartesianSolver) )
    {
        yError() << "Could not view iCartesianSolver in KDLSolver";
        return false;
    }


    if (!rpcServer.open("/getGraspingPoses/rpc:s"))
    {
        yError() << "Unable to open RPC server port" << rpcServer.getName();
        return false;
    }

    // yarp::os::Network::connect("/matching/object:o", portPrefix + "/matching/object:i");

    segmentorThread.setIRGBDSensor(iRGBDSensor);
    segmentorThread.setOutRgbImgPort(&outRgbImgPort);
    segmentorThread.setOutDepthImgPort(&outDepthImgPort);
    // segmentorThread.setInMarchingObjectDataPort(&inMarchingObjectDataPort);
    segmentorThread.setOutPointCloudPort(&outPointCloudPort);
    segmentorThread.setRpcServer(&rpcServer);
    // segmentorThread.setOutGraspingPosesPort(&outGraspingPosesPort);
    segmentorThread.setHeadIEncoders(iHeadEncoders);
    segmentorThread.setTrunkIEncoders(iTrunkEncoders);
    segmentorThread.setICartesianSolver(trunkAndHeadSolverDeviceICartesianSolver);
    if(!segmentorThread.init(rf))
    {
        yError("Bad segmentorThread.init\n");
        return false;
    }
    yInfo("--- end: configure\n");
    return true;
}


/************************************************************************/

double GetGraspingPoses::getPeriod()
{
    return watchdog;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/

bool GetGraspingPoses::updateModule()
{
    printf("GetGraspingPoses alive...\n");
    return true;
}

/************************************************************************/

bool GetGraspingPoses::interruptModule()
{
    printf("GetGraspingPoses closing...\n");
    segmentorThread.stop();
    outRgbImgPort.interrupt();
    outDepthImgPort.interrupt();
    inMarchingObjectDataPort.interrupt();


    dd.close();
    outRgbImgPort.close();
    outDepthImgPort.close();
    inMarchingObjectDataPort.close();

    return true;
}

/************************************************************************/
