// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __COLORS_DETECTION_HPP__
#define __COLORS_DETECTION_HPP__

#include <yarp/os/RFModule.h>
#include "SegmentorThread.hpp"

#define DEFAULT_CROP_SELECTOR 0  // 1=true
//#define DEFAULT_RGBD_DEVICE "RGBDSensorClient"
#define DEFAULT_RGBD_DEVICE "fakeDepthCameraFromRGBDDataDriver"
#define DEFAULT_RGBD_LOCAL "/rgbdDetection"
#define DEFAULT_RGBD_REMOTE "/rgbd"
#define DEFAULT_WATCHDOG    2       // [s]

namespace sharon
{

/**
 * @ingroup colorsDetection
 *
 * @brief colorsDetection
 */
class ColorsDetection : public yarp::os::RFModule
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

    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outImg;
    yarp::os::Port outPort;

    int cropSelector;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outCropSelectorImg;
    yarp::os::Port inCropSelectorPort;

    double watchdog;
};

} // namespace sharon

#endif // __COLORS_DETECTION_HPP__
