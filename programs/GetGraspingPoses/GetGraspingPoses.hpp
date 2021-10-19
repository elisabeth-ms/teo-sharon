// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __GET_GRASPING_POSES_HPP__
#define __GET_GRASPING_POSES_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IAnalogSensor.h>
#include "SegmentorThread.hpp"
#include "ICartesianSolver.h"

#define DEFAULT_CROP_SELECTOR 0  // 1=true
#define DEFAULT_RGBD_DEVICE "RGBDSensorClient"
#define DEFAULT_RGBD_LOCAL "/getGraspingPoses"
#define DEFAULT_RGBD_REMOTE "/xtion"
#define DEFAULT_WATCHDOG    10       // [s]


namespace sharon
{

/**
 * @ingroup getGraspingPoses
 *
 * @brief getGraspingPoses
 */
class GetGraspingPoses : public yarp::os::RFModule
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

    yarp::dev::PolyDriver headDevice;
    yarp::dev::IEncoders *iHeadEncoders;

    yarp::dev::PolyDriver trunkDevice;
    yarp::dev::IEncoders *iTrunkEncoders;

    yarp::dev::PolyDriver trunkAndHeadSolverDevice;
    ICartesianSolver * trunkAndHeadSolverDeviceICartesianSolver;
    yarp::dev::IControlLimits *headIControlLimits;
    yarp::dev::IControlLimits *trunkIControlLimits;


    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > outRgbImgPort;
    yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelFloat> > outDepthImgPort;
    yarp::os::BufferedPort<yarp::os::Bottle> inMarchingObjectDataPort;
    // yarp::os::BufferedPort<yarp::os::Bottle> outGraspingPosesPort;   
    yarp::os::BufferedPort<yarp::sig::PointCloud<yarp::sig::DataXYZ>> outPointCloudPort;
    yarp::os::RpcServer rpcServer;
    

    double watchdog;
};

} // namespace sharon

#endif // __GET_GRASPING_POSES_HPP__
