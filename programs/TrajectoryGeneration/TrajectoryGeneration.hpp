// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IAnalogSensor.h>
#include "ICartesianSolver.h"
#include "KinematicRepresentation.hpp"

#include <yarp/os/Semaphore.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/base/Planner.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/goals/GoalState.h>

#define DEFAULT_ROBOT "teoSim" // teo or teoSim (default teo)
#define DEFAULT_MODE "keyboard"
#define PT_MODE_MS 50
#define INPUT_READING_MS 10

using namespace yarp::os;
using namespace roboticslab;

namespace ob = ompl::base;
namespace og = ompl::geometric;
namespace teo
{

/**
 * @ingroup teo-sharon_programs
 *
 * @brief Trajectory Generation Core.
 *
 */
   class TrajectoryGeneration : public yarp::os::RFModule, private yarp::os::PortReader
    {        
        public:
        //TrajectoryGeneration();// constructor
        virtual bool configure(yarp::os::ResourceFinder &rf);
        bool read(yarp::os::ConnectionReader & reader) override;

        private:

            /** robot used (teo/teoSim) **/
            std::string robot;

            /** RFModule interruptModule. */
            virtual bool interruptModule();
            /** RFModule getPeriod. */
            virtual double getPeriod();
            /** RFModule updateModule. */
            virtual bool updateModule();

            /*-- Right Arm Device --*/
            /** Axes number **/
            int numRightArmJoints;
            /** Device **/
            yarp::dev::PolyDriver rightArmDevice;
            /** Encoders **/
            yarp::dev::IEncoders *rightArmIEncoders;
            /** Right Arm ControlMode2 Interface */
            yarp::dev::IControlMode *rightArmIControlMode;
            /** Right Arm PositionControl2 Interface */
            yarp::dev::IPositionControl *rightArmIPositionControl;
            /** Right Arm PositionDirect Interface */
            yarp::dev::IPositionDirect *rightArmIPositionDirect;
            /** Right Arm ControlLimits2 Interface */
            yarp::dev::IControlLimits *rightArmIControlLimits;
            /** Right Arm RemoteVariables **/
            yarp::dev::IRemoteVariables *rightArmIRemoteVariables;

            /** Solver device **/
            yarp::dev::PolyDriver rightArmSolverDevice;
            ICartesianSolver *rightArmICartesianSolver;
         
          

            /****** FUNCTIONS ******/            

            /** Execute trajectory using a thread and KdlTrajectory**/
            bool executeTrajectory(std::vector<double> rx, std::vector<double> lx, std::vector<double> rxd, std::vector<double> lxd, double duration, double maxvel);

            /** movement finished */
            bool done;

            /** Current time **/
            double initTime;


            /** Thread run */
            virtual bool threadInit();
            virtual void run();

            ob::SpaceInformationPtr si;
            ob::ProblemDefinitionPtr pdef;
            ob::PlannerPtr planner;
            og::PathGeometric * pth;

            yarp::os::RpcServer rpcServer;
            
            bool isValid(const ob::State *state);
            bool computeDiscretePath( ob::ScopedState<ob::SE3StateSpace>start,  ob::ScopedState<ob::SE3StateSpace>goal);
            bool followDiscretePath(og::PathGeometric * discretePath);



            ob::StateSpacePtr space;



     }; // class TrajectoryGeneration
}
