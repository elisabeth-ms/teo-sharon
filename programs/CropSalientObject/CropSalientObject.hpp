// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __CROP_SALIENT_OBJECT_HPP__
#define __CROP_SALIENT_OBJECT_HPP__

#include <yarp/os/RFModule.h>
#include "SegmentorThread.hpp"

#define DEFAULT_CROP_SELECTOR 0  // 1=true
#define DEFAULT_RGBD_DEVICE "RGBDSensorClient"
//#define DEFAULT_RGBD_DEVICE "fakeDepthCameraFromRGBDDataDriver"
#define DEFAULT_RGBD_LOCAL "/rgbdDetection"
#define DEFAULT_RGBD_REMOTE "/xtion"
#define DEFAULT_WATCHDOG    10       // [s]

namespace sharon
{

/**
 * @ingroup cropSalientObject
 *
 * @brief cropSalientObject
 */
class CropSalientObject : public yarp::os::RFModule
{
public:
    virtual bool configure(yarp::os::ResourceFinder & rf) override;

protected:
    virtual bool interruptModule() override;
    virtual double getPeriod() override;
    virtual bool updateModule() override;

private:
    SegmentorThread segmentorThread;
    yarp::dev::PolyDriver dd;
    yarp::dev::IRGBDSensor *iRGBDSensor;

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outRgbImgPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> > outDepthImgPort;
    yarp::os::BufferedPort<yarp::os::Bottle> inFileNamePort;
    yarp::os::Port outPort;

    int cropSelector;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb>> outCropSelectorImg;
    yarp::os::Port inCropSelectorPort;



    double watchdog;
};

} // namespace sharon

#endif // __CROP_SALIENT_OBJECT_HPP__
