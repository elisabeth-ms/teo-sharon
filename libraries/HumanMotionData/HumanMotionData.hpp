
#ifndef __HUMAN_MOTION_DATA_HPP__
#define __HUMAN_MOTION_DATA_HPP__

#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <sstream>
#include <limits>
#include <math.h>
#include "kdl/frames.hpp"
#include <iomanip>
using namespace std;
namespace HumanMotionData{
        #define HipToTrunkHeight 0.19320
        #define TrunkHeight  0.894
        #define TrunkToShoulderHeight 0.305
        #define TrunkToShoulderWidth 0.34692
        #define ShoulderToElbowHeight 0.32901
        #define ElbowToWristHeight 0.215
        #define WristToHandHeight 0.17
        #define NJoints 8
        void writePosesCsv(std::string filename, const vector<array<double, 7>> &handTrajectory, 
                          vector<array<double, 7>> &wristTrajectory, vector<array<double, 7>> &elbowTrajectory,
                          vector<array<double, 7>> &shoulderTrajectory, vector<array<double, 7>> &neckTrajectory,
                          vector<array<double, 7>> &hipTrajectory);
        void getHumanData(const string &filename, vector<array<double, 7>> &handTrajectory, 
                          vector<array<double, 7>> &wristTrajectory, vector<array<double, 7>> &elbowTrajectory,
                          vector<array<double, 7>> &shoulderTrajectory,vector<array<double, 7>> &neckTrajectory,
                          vector<array<double, 7>> &hipTrajectory);
        void printHumanData(const vector<array<double, 7>> &HandData, const vector<array<double, 7>> &wristData, 
                            const vector<array<double, 7>> &elbowData, const vector<array<double, 7>> &shoulderData,
                            const vector<array<double, 7>> &neckData,const vector<array<double, 7>> &hipData);

        void getHumanTrajectoryPoses(vector<array<double, 7>> &trajectoryHandData, vector<array<double, 7>> &trajectoryWristData, 
                                     vector<array<double, 7>> &trajectoryElbowData, vector<array<double, 7>> &trajectoryShoulderData,
                                     vector<array<double, 7>> &trajectoryNeckData,vector<array<double, 7>> &trajectoryHipData, const float &distBtwPoses);
        
        KDL::Frame getFrame(std::vector<std::array<double, 7>> discreteHandPoses, int iPose);
        
        void linkLengthAdjustementPoses(const vector<array<double, 7>> &handTrajectory, const vector<array<double, 7>> &wristTrajectory,
                                    const vector<array<double, 7>> &elbowTrajectory, const vector<array<double, 7>> &shoulderTrajectory,
                                    vector<array<double, 7>> &neckTrajectory, vector<array<double, 7>> &hipTrajectory, 
                                    vector<array<double, 7>> &adjustedHipTrajectory, vector<array<double, 7>> &adjustedNeckTrajectory,
                                    vector<array<double, 7>> &adjustedShoulderTrajectory, vector<array<double, 7>> &adjustedElbowTrajectory,
                                    vector<array<double, 7>> &adjustedWristTrajectory, vector<array<double, 7>> &adjustedHandTrajectory);
        void writeQCsv(std::string filename, const std::vector<std::array<double, NJoints>> &qTraj);
}

#endif //__HUMAN_MOTION_DATA_HPP__