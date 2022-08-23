#include <iostream>
#include <vector>

#include "ICartesianSolver.h"
#include "KinematicRepresentation.hpp"
#include "HumanMotionData.hpp"

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include "trac-ik/trac_ik/trac_ik.hpp"
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

#include <numeric>


#define MAX_JVEL 100 // DEG/S
#define TIME_STEP 0.2 // IN SECONDS
#define NODESINLAYER 240

using namespace std;
using namespace KDL;
using namespace TRAC_IK;


// struct PoseData{
//     double q[NJoints];
//     double currentPosition[3];
//     double currentOrientation[4];
//     double swivelAngle;
//     double goalSwivelAngle;
//     // double angleShoulderElbow;
//     // double angleElbowWrist;
// };

class MapHumanToRobotMotionPerFrameIK{
private:
    vector<array<double, 7>> _discreteWristPoses;
    vector<array<double, 7>> _discreteElbowPoses;
    vector<array<double, 7>> _discreteShoulderPoses;
    vector<array<double, 7>> _discreteHandPoses;
    KDL::Chain _chain;
    KDL::JntArray _qmin;
    KDL::JntArray _qmax;
    KDL::ChainFkSolverPos_recursive *_fksolver;
    KDL::ChainIkSolverVel_pinv *_iksolverv;

    TRAC_IK::TRAC_IK * _iksolver;
    int _solverMaxIter;
    double _eps;
    
    float _wpos = 100.0; // weight for end-effector position distance

    float _wangle = 100.0;
    float _wv = 100.0;
    float _wa = 0.0;
    float _wj = 0.0;
    float _wse = 100.0;
    float _wew = 100.0;

    float _wjv = 100.0;
    float _wja = 100.0;    
    float _wjj = 100.0;

    double _timeout_in_secs =0.1; // TODO PASS TO THE CONSTRUCTOR
    unsigned int _indexGraph;
    KDL::Twist _bounds;

public:
    MapHumanToRobotMotionPerFrameIK(KDL::Chain chain, KDL::JntArray qmin, KDL::JntArray qmax, int solverMaxIter, double eps){
        
        _chain = chain;
        _qmin = qmin;
        _qmax = qmax;
        
        _fksolver = new KDL::ChainFkSolverPos_recursive(_chain);
        _iksolverv = new KDL::ChainIkSolverVel_pinv(_chain);
        _solverMaxIter = solverMaxIter;
        _eps = eps;

        nlopt::opt opt; // If I don't include this before create new Trac_ik, it gives undefined refence to opt references 
        _iksolver = new  TRAC_IK::TRAC_IK(_chain, _qmin, _qmax, _timeout_in_secs, _eps, TRAC_IK::Distance);  
        _bounds = KDL::Twist::Zero();
        _bounds.vel.x(0.0005);
        _bounds.vel.y(0.0005);
        _bounds.vel.z(0.0005);
        _bounds.rot.x(0.015);
        _bounds.rot.y(0.015);
        _bounds.rot.z(0.015);
        // _bounds.rot.x(std::numeric_limits<float>::max());
        // _bounds.rot.y(std::numeric_limits<float>::max());
        // _bounds.rot.z(std::numeric_limits<float>::max());
        // humanToRobotJointsIK = new  HumanHandToRobotJointsIK::HumanHandToRobotJointsIK(NJoints, _qmin, _qmax, 4, _chain);

    };
    ~MapHumanToRobotMotionPerFrameIK(){};
    void getHumanData(const string &filename, vector<array<double, 8>> &  handTrajectory, vector<array<double, 7>> &wristTrajectory, vector<array<double, 7>> & elbowTrajectory,vector<array<double, 7>> & shoulderTrajectory);
    void printHumanData(const vector<array<double, 8>> &HandData, const vector<array<double, 7>> &wristData, const vector<array<double, 7>> &elbowData, const vector<array<double, 7>> &shoulderData);
    void getHumanTrajectoryPoses(vector<array<double, 8>> &trajectoryHandData, vector<array<double, 7>> &trajectoryWristData, vector<array<double, 7>> &trajectoryElbowData, vector<array<double, 7>> &trajectoryShoulderData, const float &distBtwPoses);

    vector<array<double, 7>> getDiscreteHandPoses(){
        return _discreteHandPoses;
    }

    vector<array<double, 7>> getDiscreteElbowPoses(){
        return _discreteElbowPoses;
    }

    vector<array<double, 7>> getDiscreteShoulderPoses(){
        return _discreteShoulderPoses;
    }

    vector<array<double, 7>> getDiscreteWristPoses(){
        return _discreteWristPoses;
    }

    KDL::ChainFkSolverPos_recursive getFKSolver(){
        return (*_fksolver);
    }

    int ik(const KDL::JntArray &qInit, const KDL::Frame &fpose, KDL::JntArray &q);
    int fk(const KDL::JntArray &q, KDL::Frame &fpose);
    void getCurrentPoses(const KDL::JntArray &q, KDL::Frame &currentShoulderPose, KDL::Frame &currentElbowPose, KDL::Frame &currentWristPose, KDL::Frame &currentPose);

    void handPosesToFPose(int index, KDL::Frame &fGoal);

    double angleBetweenVectors(Eigen::Vector3d a, Eigen::Vector3d b);

    // double computeSwivelAngle(Eigen::Vector3d & shoulderPosition, Eigen::Vector3d & elbowPosition, Eigen::Vector3d & wristPosition);

    // double shoulderElbowAngleDistance(const KDL::Frame & currentElbowPose, const KDL::Frame & currentShoulderPose, const Eigen::Vector3d & goalShoulderElbow);

    
};
