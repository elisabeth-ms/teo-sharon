#ifndef __HUMAN_HAND_TO_ROBOT_JOINTS_IK_HPP__
#define __HUMAN_HAND_TO_ROBOT_JOINTS_IK_HPP__

#include <string>
#include <vector>
#include <array>


#include "ICartesianSolver.h"
#include "KinematicRepresentation.hpp"

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>
#include <eigen3/unsupported/Eigen/Splines>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
// #include <kdl/chainiksolvervel_pinv.hpp>
// #include <kdl/chainiksolverpos_nr_jl.hpp>
#include <nlopt.hpp>
#include "CheckSelfCollisionLibrary.hpp"

// #define TrunkHeight  0.894

// typedef Eigen::Spline<double,1, 2> Spline1D;
// typedef Eigen::SplineFitting<Spline1D> SplineFitting1D;


namespace HumanHandToRobotJointsIK{
class HumanHandToRobotJointsIK{
    public: 
        HumanHandToRobotJointsIK(int numberJoints, KDL::JntArray qmin, KDL::JntArray qmax, int passFramesUsed, KDL::Chain chain){
            _numberJoints = numberJoints;
            _qmin = qmin;
            _qmax = qmax;
            _passFramesUsed = passFramesUsed;
            _chain = chain;        
            _fksolver = new KDL::ChainFkSolverPos_recursive(_chain);
            _opt = nlopt_create(NLOPT_LN_BOBYQA, _numberJoints);
            _jntToJacSolver = new KDL::ChainJntToJacSolver(_chain);

        }

        HumanHandToRobotJointsIK(){

        }


        ~HumanHandToRobotJointsIK(){

        }

        void setWeights(float wPosition, float wOrientation, float wShoulderElbow, float wElbowWrist){
            _wPosition = wPosition;
            _wOrientation = wOrientation;
            _wShoulderElbow = wShoulderElbow;
            _wElbowWrist = wElbowWrist;
        }
        KDL::ChainFkSolverPos_recursive getFKSolver(){
            return *(_fksolver);
        }

        int getNumberJoints(){
            return _numberJoints;
        }

        KDL::Frame getGoalPose(){
            return _goalPose;
        }

        float getWPosition(){
            return _wPosition;
        }

        KDL::JntArray getQMin()
        {
            return _qmin;
        }

        KDL::JntArray getQMax()
        {
            return _qmax;
        }
        nlopt_opt getOpt(){
            return _opt;
        }
        void setGoalPose(KDL::Frame goalPose){
            _goalPose = goalPose;
        }

        void setGoalShoulderPosition( Eigen::Vector3d goalShoulder){
            _goalShoulderPosition = goalShoulder;
        }

        void setGoalElbowPosition( Eigen::Vector3d goalElbow){
            _goalElbowPosition = goalElbow;
        }

        void setGoalWristPosition( Eigen::Vector3d goalWrist){
            _goalWristPosition = goalWrist;
        }

        Eigen::Vector3d getGoalShoulderPosition(){
            return _goalShoulderPosition;
        }

        Eigen::Vector3d getGoalElbowPosition(){
            return _goalElbowPosition;
        }

        Eigen::Vector3d getGoalWristPosition(){
            return _goalWristPosition;
        }

        float getWSwivelAngle(){
            return _wSwivelAngle;
        }

        KDL::ChainJntToJacSolver getJntToJacSolver(){
            return (*_jntToJacSolver);
        }

    private:
        int _numberJoints;
        KDL::JntArray _qmin;
        KDL::JntArray _qmax;
        int _passFramesUsed;
        KDL::Chain _chain;
        KDL::ChainFkSolverPos_recursive *_fksolver;
        nlopt_opt _opt;
        float _wPosition = 5.0;
        float _wOrientation = 0.0;
        float _wShoulderElbow = 0.0;
        float _wElbowWrist = 0.0;
        float _wSwivelAngle = 5.0;
        double _tol = 1e-4;
        double _vectorShoulderElbow[3];
        double _vectorElbowWrist[3];
        KDL::Frame _goalPose;
        Eigen::Vector3d _goalShoulderPosition;
        Eigen::Vector3d _goalElbowPosition;
        Eigen::Vector3d _goalWristPosition;
        KDL::ChainJntToJacSolver * _jntToJacSolver;
    };
    HumanHandToRobotJointsIK * humanToRobotJointsIK;
    bool computeOPT(const double * qInit, const double * qBounds, std::vector<double> & q, Eigen::Vector3d goalShoulder,
                Eigen::Vector3d goalElbow, Eigen::Vector3d goalWrist, const KDL::Frame & goalPose);
    double optimizationFunctionJoints(unsigned n, const double *x, double *grad, void *data);
    double positionGoalMatchingObjectiveTerm(const KDL::Frame &currentPose, const double (&positionGoal)[3]);
    double computeSwivelAngle(Eigen::Vector3d & shoulderPosition, Eigen::Vector3d & elbowPosition, Eigen::Vector3d & wristPosition);
    double angleBetweenVectors(Eigen::Vector3d a, Eigen::Vector3d b);
    double computeConditionNumber(const KDL::JntArray &q_in, KDL::Jacobian &jac);
    double kinematicSingularityConstraint(unsigned n, const double *x, double *grad, void *data);
}


// namespace sharon{
//     /**
//      * @brief Get the Trajectory From Csv File object.
//      * 
//      * @param filename Csv file to get the data from.
//      * @return Trajectory data as a vector constructed with arrays of NJoints elements [frame, x, y, z, qx, qy, qz, qw]. Each array represents one frame and pose.
//      */
//     std::vector<std::array<double,NJoints>> getTrajectoryFromCsvFile(const std::string& filename);
 
//     void getHumanData(const std::string &filename, std::vector<std::array<double, 8>> & handTrajectory, std::vector<std::array<double, 7>> & wristTrajectory, std::vector<std::array<double, 7>> &elbowTrajectory,std::vector<std::array<double, 7>> &shoulderTrajectory);
    
//     void printHumanData(const std::vector<std::array<double, 8>> &HandData, const std::vector<std::array<double, 7>> &wristData, 
//                         const std::vector<std::array<double, 7>> &elbowData, const std::vector<std::array<double, 7>> &shoulderData);
//     /**
//      * @brief Print in the console the trajectory stored in data.
//      * 
//      * @param data Trajectory data stored as blocks of NJoints elements [frame, x, y, z, qx, qy, qz, qw]
//      */
//     void printTrajectoryData(const std::vector<std::array<double,NJoints>> &data);

//     /**
//      * @brief Get the Trajectory Poses object
//      * 
//      * @param trajectoryData  Trajectory data stored as blocks of NJoints elements [frame, x, y, z, qx, qy, qz, qw]
//      * @param numPoses Number of poses of the initial trajectory to generate the discrete trajectory.
//      * @param discreteTrajectory Discrete trajectory constructed as a array with numPoses*7 elements. Each group of 7 elements in the vector represent one discrete pose. 
//      * @param size size of the discrete trajectory arrray
//      */

//     void getTrajectoryPoses(const std::vector<std::array<double, NJoints>> &trajectoryData, const unsigned int &numPoses, std::vector<double>& discreteTrajectory);
    

//     void getTrajectoryPoses(std::vector<std::array<double, NJoints>> &trajectoryData, const float &distBtwPoses, std::vector<double> &discreteTrajectory, unsigned int & sizeDiscreteTrajectory);

//     void getHumanTrajectoryPoses(std::vector<std::array<double, 8>> &trajectoryHandData, std::vector<std::array<double, 7>> &trajectoryWristData,
//                               std::vector<std::array<double, 7>> &trajectoryElbowData, std::vector<std::array<double, 7>> &trajectoryShoulderData,
//                               const float &distBtwPoses, std::vector<std::array<double, 7>> &discreteHandTrajectory, std::vector<std::array<double, 7>> &discreteWristTrajectory, 
//                               std::vector<std::array<double, 7>> &discreteElbowTrajectory, std::vector<std::array<double, 7>> &discreteShoulderTrajectory,
//                               unsigned int &sizeDiscreteTrajectory, std::vector<int>&usedFrames);


//     /**
//      * @brief Optimization function to minimize the distance between the pose of the desired trajectory and the final trajectory that satisfies the constraints.
//      * This function is defined to be used with nlopt library. 
//      * 
//      * @param x Optimization parameters.
//      * @param grad Gradient of the optimization function with respect to the optimization pararameters.
//      * @param data Additional data to be used in the optimization function.
//      * @return Value of the optimization function at x.
//      */

//     /**
//      * @brief Optimization function to minimize the distance between the pose of the desired trajectory and the final trajectory that satisfies the constraints.
//      * This function is defined to be used with nlopt library. 
//      * 
//      * @param n number of parameters
//      * @param x Optimization parameters.
//      * @param grad Gradient of the optimization function with respect to the optimization pararameters.
//      * @param data Additional data to be used in the optimization function.
//      * @return Value of the optimization function at x. 
//      */

//     double positionGoalMatchingObjectiveTerm(const KDL::Frame & currentPose, const double (& positionGoal) [3]);

//     double jointVelocityObjectiveTerm(const double *x, std::vector<std::array<double, NJoints>> &velocity, double dt, int iPose);

//     double jointAccelerationObjectiveTerm(const std::vector<std::array<double, NJoints>> & velocity, std::vector<std::array<double, NJoints>> & acceleration, double dt, int iPose);

//     double jointJerkObjectiveTerm(const std::vector<std::array<double, NJoints>> & acceleration, std::vector<std::array<double, NJoints>>  & jerk, double dt, int iPose);

//     double endEffectorVelocityObjectiveTerm(const KDL::Frame &currentPose, const KDL::Frame & prevPose,  double dt, int iPose);
    
//     void jointVelocityLimit (unsigned n, const double *x, double *grad, void *data);

//     double selfCollisionObjectiveTerm(const KDL::JntArray &q);

//     double shoulderElbowGoalObjectiveTerm(const KDL::Frame & currentElbowPose, const KDL::Frame & currentShoulderPose, const Eigen::Vector3d & goalShoulderElbow);

//     double elbowWristGoalObjectiveTerm(const KDL::Frame & currentElbowPose, const KDL::Frame & currentWristPose, const Eigen::Vector3d & goalElbowWrist);


//     double normalArmPlaneGoalObjectiveTerm(const KDL::Frame &currentWristPose, const KDL::Frame & currentElbowPose, const KDL::Frame & currentShoulderPose,
//                                      const Eigen::Vector3d & normalArmPlaneGoal);

//     double c1(unsigned n, const double *x, double *grad, void *data);
    
//     double optimizationFunctionJoints(unsigned n, const double *x, double *grad, void *data);
//     /**
//      * @brief 
//      * 
//      * @param m 
//      * @param result 
//      * @param n 
//      * @param x 
//      * @param grad 
//      * @param data 
//      */
//     double orientationGoalMatchingObjectiveTerm(const Eigen::Quaternionf &q, const Eigen::Quaternionf &qg);

//     void writeCsv(std::string filename, const double * x, const unsigned int & numPoses);
    
//     void writeQCsv(std::string filename, const std::vector<std::array<double,NJoints>>& qTraj);

//     void writePoseCsv(std::string filename, const std::vector<std::array<double, 7>> &poses);

//     void  writeDataCsv(std::string filename, const std::vector<double> &orientatationdist,  const std::vector<double> &angleShoudlerElbow,  const std::vector<double> &angleElbowWrist);

//     KDL::Frame getFrame(std::vector<std::array<double, 7>> discreteHandPoses, int iPose);
//         double jointVelocityObjectiveTerm(std::vector<std::array<double, NJoints>> qx, int iPose, KDL::JntArray q);
// }// namespace sharon


#endif //__HUMAN_HAND_TO_ROBOT_JOINTS_IK_HPP__
