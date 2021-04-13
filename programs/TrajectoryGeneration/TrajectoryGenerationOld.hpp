// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRAJECTORY_GENERATION_HPP__
#define __TRAJECTORY_GENERATION_HPP__

#include <yarp/os/RFModule.h>
#include <yarp/os/LogStream.h>
#include "TrajectoryThread.hpp"

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/ControlBoardInterfaces.h>

#include <yarp/os/RpcServer.h>

#include <kdl/frames.hpp>

#include "ICartesianSolver.h"
#include <KinematicRepresentation.hpp>
#include <KdlVectorConverter.hpp>
#include <ConfigurationSelector.hpp>
#include <yarp/os/Semaphore.h>

// #include <ompl/base/spaces/RealVectorStateSpace.h>
//  #include <ompl/geometric/planners/rrt/RRTstar.h>
#define DEFAULT_WATCHDOG    10       // [s]
#define DEFAULT_POSESIZE    7 // position + quaternion


using namespace roboticslab::KdlVectorConverter;
using namespace roboticslab::KinRepresentation;
using namespace yarp::os;
using namespace roboticslab;
// namespace ob = ompl::base;
// namespace og = ompl::geometric;

namespace sharon
{

/**
 * @ingroup TrajectoryGeneration
 *
 * @brief TrajectoryGeneration
 */
class TrajectoryGeneration : public yarp::os::RFModule,  private yarp::os::PortReader
{
public:
    virtual bool configure(yarp::os::ResourceFinder & rf) override;

protected:
    virtual bool interruptModule() override;
    virtual double getPeriod() override;
    virtual bool updateModule() override;
    bool read(yarp::os::ConnectionReader & connection) override;
    //bool isValid(const ob::State *state);
private:
    // TrajectoryThread TrajectoryThread;
    yarp::dev::PolyDriver clientRightArm;
    yarp::dev::IControlLimits *lim;
    yarp::dev::IControlMode * mod;
    yarp::dev::IEncoders * iEncoders;
    yarp::dev::IPositionControl *pos;
    yarp::dev::PolyDriver rightArmSolverDevice;
    roboticslab::ICartesianSolver * iCartesianSolver;
    int axes;
    yarp::os::RpcServer inTrajectoryGenerationPort;
    //ob::SpaceInformationPtr si;
    yarp::dev::IControlLimits *limTrunk;


    double watchdog;
};

} // namespace sharon

#endif // __TRAJECTORY_GENERATION_HPP__
