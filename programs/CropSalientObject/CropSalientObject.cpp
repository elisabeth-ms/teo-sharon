// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CropSalientObject.hpp"

#include <cstdio>

#include <ColorDebug.h>

using namespace sharon;

/************************************************************************/

bool CropSalientObject::configure(yarp::os::ResourceFinder & rf)
{
    cropSelector = DEFAULT_CROP_SELECTOR;
    std::string strRGBDDevice = DEFAULT_RGBD_DEVICE;
    std::string strRGBDLocal = DEFAULT_RGBD_LOCAL;
    std::string strRGBDRemote = DEFAULT_RGBD_REMOTE;
    watchdog = DEFAULT_WATCHDOG;  // double

    printf("RgbdDetection options:\n");
    printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
    printf("\t--cropSelector (default: \"%d\")\n",cropSelector);
    printf("\t--RGBDDevice (device we create, default: \"%s\")\n",strRGBDDevice.c_str());
    printf("\t--RGBDLocal (if accesing remote, local port name, default: \"%s\")\n",strRGBDLocal.c_str());
    printf("\t--RGBDRemote (if accesing remote, remote port name, default: \"%s\")\n",strRGBDRemote.c_str());
    printf("\t--watchdog ([s] default: \"%f\")\n",watchdog);

    if(rf.check("cropSelector")) cropSelector = rf.find("cropSelector").asInt32();
    printf("RgbdDetection using cropSelector: %d.\n",cropSelector);
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
    std::string portPrefix("/rgbdDetection");
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


    if(!outPort.open(portPrefix + "/state:o"))
    {
        yError("Bad outPort.open\n");
        return false;
    }

    if(!inFileNamePort.open(portPrefix + "/imageName:i"))
    {
        yError("Bad fileNamePort.open\n");
        return false;
    }

    yarp::os::Network::connect("/server/imageName", portPrefix + "/imageName:i");

    if(cropSelector != 0)
    {
        outCropSelectorImg.open(strRGBDLocal + "/cropSelector/img:o");
        inCropSelectorPort.open(strRGBDLocal + "/cropSelector/state:i");
    }

    segmentorThread.setIRGBDSensor(iRGBDSensor);
    segmentorThread.setOutRgbImgPort(&outRgbImgPort);
    segmentorThread.setOutDepthImgPort(&outDepthImgPort);
    segmentorThread.setOutPort(&outPort);
    segmentorThread.setFileNamePort(&inFileNamePort);


    segmentorThread.setCropSelector(cropSelector);
    if(cropSelector != 0) {
        segmentorThread.setOutCropSelectorImg(&outCropSelectorImg);
        segmentorThread.setInCropSelectorPort(&inCropSelectorPort);
    }


    if(!segmentorThread.init(rf))
    {
        yError("Bad segmentorThread.init\n");
        return false;
    }
    yInfo("--- end: configure\n");
    return true;
}


/************************************************************************/

double CropSalientObject::getPeriod()
{
    return watchdog;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/

bool CropSalientObject::updateModule()
{
    printf("ColorsDetection alive...\n");
    return true;
}

/************************************************************************/

bool CropSalientObject::interruptModule()
{
    printf("RgbdDetection closing...\n");
    segmentorThread.stop();
    outRgbImgPort.interrupt();
    outDepthImgPort.interrupt();
    outPort.interrupt();
    inFileNamePort.interrupt();
    if(cropSelector != 0) {
        outCropSelectorImg.interrupt();
        inCropSelectorPort.interrupt();
    }
    dd.close();
    outRgbImgPort.close();
    outDepthImgPort.close();
    inFileNamePort.close();
    outPort.close();
    if(cropSelector != 0) {
        outCropSelectorImg.close();
        inCropSelectorPort.close();
    }
    return true;
}

/************************************************************************/
