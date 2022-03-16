#include <iostream>
#include <vector>

#include "ICartesianSolver.h"
#include "KinematicRepresentation.hpp"

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>


#include "graph.h"



#define TrunkHeight  0.894
#define MAX_JVEL 30 // DEG/S
#define TIME_STEP 0.2 // IN SECONDS
using namespace std;
class MapHumanToRobotMotion{
private:
    vector<array<double, 7>> _discreteWristPoses;
    vector<array<double, 7>> _discreteElbowPoses;
    vector<array<double, 7>> _discreteShoulderPoses;
    vector<array<double, 7>> _discreteHandPoses;
    KDL::Chain _chain;
    KDL::JntArray _qmin;
    KDL::JntArray _qmax;
    KDL::ChainFkSolverPos_recursive *_fksolver;
    KDL::ChainIkSolverPos_NR_JL *_iksolver;
    KDL::ChainIkSolverVel_pinv *_iksolverv;
    int _solverMaxIter;
    double _eps;
    Graph _graph;
    float _wpos = 50.0; // weight for end-effector position distance

public:
    MapHumanToRobotMotion(KDL::Chain chain, KDL::JntArray qmin, KDL::JntArray qmax, int solverMaxIter, double eps){
        _chain = chain;
        _qmin = qmin;
        _qmax = qmax;
        _fksolver = new KDL::ChainFkSolverPos_recursive(_chain);
        _iksolverv = new KDL::ChainIkSolverVel_pinv(_chain);
        _solverMaxIter = solverMaxIter;
        _eps = eps;
        _iksolver = new KDL::ChainIkSolverPos_NR_JL(_chain, _qmin, _qmax, *_fksolver, *_iksolverv, _solverMaxIter, _eps);

    };
    ~MapHumanToRobotMotion(){};
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

    void ik(const KDL::JntArray &qInit, const KDL::Frame &fpose, KDL::JntArray &q);
    
    void addFirstLayerToGraph(int numberOfNodes,  const KDL::Frame &fpose);

    void addNextLayersToGraph(int numberOfNodes,  const KDL::Frame &fpose, const int timeStep);
    
    void getCurrentPoses(const KDL::JntArray &q, KDL::Frame &currentShoulderPose, KDL::Frame &currentElbowPose, KDL::Frame &currentWristPose, KDL::Frame &currentPose);
 
    bool feasibleJointsVel(const VertexProperty&  vp1, const VertexProperty&  vp2);

    void findShortestPathInGraph();

};
