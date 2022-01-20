#ifndef __MAP_HUMAN_HAND_TRAJECTORY_TO_ROBOT_JOINT_SPACE_HPP__
#define __MAP_HUMAN_HAND_TRAJECTORY_TO_ROBOT_JOINT_SPACE_HPP__

#include <string>
#include <vector>
#include <array>


#include "ICartesianSolver.h"
#include "KinematicRepresentation.hpp"

#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Geometry>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

#include "CheckSelfCollisionLibrary.hpp"

#define NJoints 8


namespace sharon{
    /**
     * @brief Get the Trajectory From Csv File object.
     * 
     * @param filename Csv file to get the data from.
     * @return Trajectory data as a vector constructed with arrays of NJoints elements [frame, x, y, z, qx, qy, qz, qw]. Each array represents one frame and pose.
     */
    std::vector<std::array<double,NJoints>> getTrajectoryFromCsvFile(const std::string& filename);
 
    /**
     * @brief Print in the console the trajectory stored in data.
     * 
     * @param data Trajectory data stored as blocks of NJoints elements [frame, x, y, z, qx, qy, qz, qw]
     */
    void printTrajectoryData(const std::vector<std::array<double,NJoints>> &data);

    /**
     * @brief Get the Trajectory Poses object
     * 
     * @param trajectoryData  Trajectory data stored as blocks of NJoints elements [frame, x, y, z, qx, qy, qz, qw]
     * @param numPoses Number of poses of the initial trajectory to generate the discrete trajectory.
     * @param discreteTrajectory Discrete trajectory constructed as a array with numPoses*7 elements. Each group of 7 elements in the vector represent one discrete pose. 
     * @param size size of the discrete trajectory arrray
     */

    void getTrajectoryPoses(const std::vector<std::array<double, NJoints>> &trajectoryData, const unsigned int &numPoses, std::vector<double>& discreteTrajectory);
    

    void getTrajectoryPoses(std::vector<std::array<double, NJoints>> &trajectoryData, const float &distBtwPoses, std::vector<double> &discreteTrajectory, unsigned int & sizeDiscreteTrajectory);

    /**
     * @brief Optimization function to minimize the distance between the pose of the desired trajectory and the final trajectory that satisfies the constraints.
     * This function is defined to be used with nlopt library. 
     * 
     * @param x Optimization parameters.
     * @param grad Gradient of the optimization function with respect to the optimization pararameters.
     * @param data Additional data to be used in the optimization function.
     * @return Value of the optimization function at x.
     */

    /**
     * @brief Optimization function to minimize the distance between the pose of the desired trajectory and the final trajectory that satisfies the constraints.
     * This function is defined to be used with nlopt library. 
     * 
     * @param n number of parameters
     * @param x Optimization parameters.
     * @param grad Gradient of the optimization function with respect to the optimization pararameters.
     * @param data Additional data to be used in the optimization function.
     * @return Value of the optimization function at x. 
     */

    double positionGoalMatchingObjectiveTerm(const KDL::Frame & currentPose, const double (& positionGoal) [3]);

    double jointVelocityObjectiveTerm(const double *x, std::vector<std::array<double, NJoints>> &velocity, double dt, int iPose);

    double jointAccelerationObjectiveTerm(const std::vector<std::array<double, NJoints>> & velocity, std::vector<std::array<double, NJoints>> & acceleration, double dt, int iPose);

    double jointJerkObjectiveTerm(const std::vector<std::array<double, NJoints>> & acceleration, std::vector<std::array<double, NJoints>>  & jerk, double dt, int iPose);

    double endEffectorVelocityObjectiveTerm(const KDL::Frame &currentPose, const KDL::Frame & prevPose,  double dt, int iPose);
    
    void jointVelocityLimit (unsigned n, const double *x, double *grad, void *data);

    double selfCollisionObjectiveTerm(const KDL::JntArray &q);

    double c1(unsigned n, const double *x, double *grad, void *data);
    
    double optimizationFunctionJoints(unsigned n, const double *x, double *grad, void *data);
    /**
     * @brief 
     * 
     * @param m 
     * @param result 
     * @param n 
     * @param x 
     * @param grad 
     * @param data 
     */
    double orientationGoalMatchingObjectiveTerm(const Eigen::Quaternionf &q, const Eigen::Quaternionf &qg);

    void writeCsv(std::string filename, const double * x, const unsigned int & numPoses);
    
    void writeQCsv(std::string filename, const std::vector<std::array<double,NJoints>>& qTraj);

    void writePoseCsv(std::string filename, const std::vector<std::array<double, 7>> &poses);

    void writeDataCsv(std::string filename, const std::vector<double> orientatationdist);


}// namespace sharon


#endif //__MAP_HUMAN_HAND_TRAJECTORY_TO_ROBOT_JOINT_SPACE_HPP__
