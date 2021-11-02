// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IAnalogSensor.h>
#include "ICartesianSolver.h"
#include "KinematicRepresentation.hpp"
#include <yarp/dev/DeviceDriver.h>
// #include <kdl/chain.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/frames.hpp>
// #include <kdl/jntarray.hpp>
// #include <kdl/joint.hpp>
// #include <kdl/utilities/utility.h>

#include <yarp/os/Semaphore.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/Planner.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/geometric/planners/rrt/BiTRRT.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rlrt/BiRLRT.h>

#include "../../libraries/CheckSelfCollisionLibrary/CheckSelfCollisionLibrary.hpp"

// fcl
// #include "fcl/octree.h"
// #include "fcl/config.h"
// #include <fcl/collision.h>
// #include <fcl/collision_object.h>
// #include <fcl/shape/geometric_shapes.h>
// #include <fcl/shape/geometric_shapes_utility.h>


#define DEFAULT_ROBOT "teo" // teo or teoSim (default teo)
#define DEFAULT_PLANNING_SPACE "joint" // joint or cartesian
#define DEFAULT_MODE "keyboard"
#define PT_MODE_MS 50
#define INPUT_READING_MS 10
#define SEND_TRAJECTORY_DATA true


#define AXIAL_SHOULDER_LINK_LENGTH 0.32901
#define AXIAL_SHOULDER_LINK_RADIUS 0.075
#define FRONTAL_ELBOW_LINK_LENGTH 0.215
#define FRONTAL_ELBOW_LINK_RADIUS 0.07

#define FRONTAL_WRIST_LINK_LENGTH 0.45
#define FRONTAL_WRIST_LINK_RADIUS 0.2



using namespace yarp::os;
using namespace roboticslab;

namespace ob = ompl::base;
namespace og = ompl::geometric;


/**
 * @ingroup teo-sharon_programs
 *
 * @brief Trajectory Generation Core.
 *
 */
   class TrajectoryGeneration : public yarp::dev::DeviceDriver, public yarp::os::PortReader
    {        
        public:
        //TrajectoryGeneration();// constructor
        bool open(yarp::os::Searchable &config) override;
        bool read(yarp::os::ConnectionReader & reader) override;

        protected:
          typedef std::shared_ptr<fcl::CollisionGeometryf> CollisionGeometryPtr_t;
          sharon::CheckSelfCollision *checkSelfCollision;

        private:
            bool openDevices();
            /** robot used (teo/teoSim) **/
            std::string robot;
            std::string prefix;
            yarp::os::ResourceFinder resourceFinder;
            /** (joint/cartesian) space to plan a trajectory **/
            std::string planningSpace;  

            /** device name to plan a trajectory **/
            std::string deviceName;
            /** kinematics config file **/
            std::string kinematicsConfig;

            KDL::Chain chain;
            KDL::JntArray qmin;
            KDL::JntArray qmax;
            std::vector<fcl::CollisionObjectf> collisionObjects;
            std::vector<std::array<float, 3>> offsetCollisionObjects;
            std::vector<fcl::CollisionObjectf> tableCollision;
            
            std::vector<KDL::Frame> centerLinkWrtJoint;

            /*-- Arm Device --*/
            /** Axes number **/
            int numArmJoints;
            int numTrunkJoints;
            /** Device **/
            yarp::dev::PolyDriver armDevice;
            yarp::dev::PolyDriver trunkDevice;

            std::string rightArmDeviceName;
            std::string trunkDeviceName;
            /** Encoders **/
            yarp::dev::IEncoders *armIEncoders;
            yarp::dev::IEncoders *trunkIEncoders;
           /** ControlMode2 Interface */

            yarp::dev::IControlMode *armIControlMode;
            yarp::dev::IControlMode *trunkIControlMode;
            /** PositionControl2 Interface */
            yarp::dev::IPositionControl *armIPositionControl;
            yarp::dev::IPositionControl *trunkIPositionControl;

            /** Right Arm ControlLimits2 Interface */
            yarp::dev::IControlLimits *armIControlLimits;
            yarp::dev::IControlLimits *trunkIControlLimits;


            /** Solver device **/
            yarp::dev::PolyDriver armSolverDevice;
            ICartesianSolver *armICartesianSolver;
            yarp::os::Property armSolverOptions;

            /** Joints limits **/
            yarp::os::Bottle qrMin;
            yarp::os::Bottle qrMax;

            /****** FUNCTIONS ******/            


            /** movement finished */
            bool done;

            /** Current time **/
            double initTime;


            ob::SpaceInformationPtr si;
            ob::ProblemDefinitionPtr pdef;
            og::PathGeometric * pth;

            yarp::os::RpcServer rpcServer;
            
            bool isValid(const ob::State *state);
            bool getCurrentQ(std::vector<double> & currentQ);
            bool computeDiscretePath(ob::ScopedState<ob::SE3StateSpace> start, ob::ScopedState<ob::SE3StateSpace> goal, std::vector<std::vector<double>> &jointsTrajectory, bool &validStartState, bool &validGoalState);
            bool computeDiscretePath(ob::ScopedState<ob::RealVectorStateSpace> start, ob::ScopedState<ob::RealVectorStateSpace> goal, std::vector<std::vector<double>> &jointsTrajectory, bool &validStartState, bool &validGoalState);
            std::vector<double> goalQ;
            // bool followDiscretePath();

            ob::StateSpacePtr space;

            #ifdef SEND_TRAJECTORY_DATA
            // Publish trajectory just for visualization
            yarp::os::BufferedPort<yarp::os::Bottle> outCartesianTrajectoryPort;
            #endif



     }; // class TrajectoryGeneration

