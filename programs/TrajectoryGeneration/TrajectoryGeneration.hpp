// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IAnalogSensor.h>
#include "ICartesianSolver.h"
#include "KinematicRepresentation.hpp"
#include <yarp/dev/DeviceDriver.h>

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
#include <yarp/os/Vocab.h>
#include <yarp/dev/GenericVocabs.h>

#include <string>
#include <ostream>

#define DEFAULT_ROBOT "teo" // teo or teoSim (default teo)
#define DEFAULT_PLANNING_SPACE "joint" // joint or cartesian
#define DEFAULT_PLANNER "RRTConnect" //RRTCONNECT, RRTstar

#define AXIAL_SHOULDER_LINK_LENGTH 0.305
#define AXIAL_SHOULDER_LINK_RADIUS 0.075
#define FRONTAL_ELBOW_LINK_LENGTH 0.215
#define FRONTAL_ELBOW_LINK_RADIUS 0.07

#define FRONTAL_WRIST_LINK_LENGTH 0.45
#define FRONTAL_WRIST_LINK_RADIUS 0.2



using namespace yarp::os;
using namespace roboticslab;

namespace ob = ompl::base;
namespace og = ompl::geometric;


constexpr auto VOCAB_CHECK_GOAL_POSE = yarp::os::createVocab32('c','h','g','p');
constexpr auto VOCAB_CHECK_GOAL_JOINTS = yarp::os::createVocab32('c','h','g','j');
constexpr auto VOCAB_COMPUTE_JOINTS_PATH_GOAL_POSE = yarp::os::createVocab32('c','p','g','p');
constexpr auto VOCAB_COMPUTE_JOINTS_PATH_GOAL_JOINTS = yarp::os::createVocab32('c','p','g','j');



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

            /** planner type**/
            std::string plannerType;
            ob::PlannerPtr planner;
            double plannerRange;
            double pruneThreshold;
            double maxPlannerTime;

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

            /** movement finished */
            bool done;

            /** Current time **/
            double initTime;

            bool jointsInsideBounds;

            ob::SpaceInformationPtr si;
            ob::ProblemDefinitionPtr pdef;
            og::PathGeometric * pth;

            yarp::os::RpcServer rpcServer;
            
            void changeJointsLimitsFromConfigFile(KDL::JntArray & qlim, const yarp::os::Searchable& config, const std::string& mode);
            bool checkGoalPose(yarp::os::Bottle *, std::vector<double> & desireQb, std::string & errorMessage);
            bool checkGoalJoints(yarp::os::Bottle * bGoal, std::string & errorMessage);


            bool isValid(const ob::State *state);
            bool getCurrentQ(std::vector<double> & currentQ);
            bool computeDiscretePath(ob::ScopedState<ob::SE3StateSpace> start, ob::ScopedState<ob::SE3StateSpace> goal, std::vector<std::vector<double>> &jointsTrajectory, bool &validStartState, bool &validGoalState);
            bool computeDiscretePath(ob::ScopedState<ob::RealVectorStateSpace> start, ob::ScopedState<ob::RealVectorStateSpace> goal, std::vector<std::vector<double>> &jointsTrajectory, std::string & errorMessage);
            std::vector<double> goalQ;
            ob::StateSpacePtr space;


          yarp::os::Bottle makeUsage()
          {
            return {
            yarp::os::Value(VOCAB_HELP, true),
            yarp::os::Value("\tlist commands"),
            yarp::os::Value(VOCAB_CHECK_GOAL_POSE, true),
            yarp::os::Value("\tcheck if the robot collides in the goal pose (x, y, z, rotx, roty, rotz). If it does not collide it retrives the joints position."),
            yarp::os::Value(VOCAB_CHECK_GOAL_JOINTS, true),
            yarp::os::Value("\tcheck if the robot collides with the goal joints (j0, j1, ..., jn)"),
            yarp::os::Value(VOCAB_COMPUTE_JOINTS_PATH_GOAL_JOINTS, true),
            yarp::os::Value("\tcompute the joints path to the goal joints."),
            yarp::os::Value(VOCAB_COMPUTE_JOINTS_PATH_GOAL_POSE, true),
            yarp::os::Value("\tcompute the joints path to the goal pose."),
        };
}




     }; // class TrajectoryGeneration

