#ifndef __MAP_HUMAN_HAND_TRAJECTORY_TO_ROBOT_END_EFFECTOR_HPP__
#define __MAP_HUMAN_HAND_TRAJECTORY_TO_ROBOT_END_EFFECTOR_HPP__

#include <string>
#include <vector>
#include <array>

#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <yarp/dev/IAnalogSensor.h>
#include "ICartesianSolver.h"
#include "KinematicRepresentation.hpp"

#include "CheckSelfCollisionLibrary.hpp"

namespace sharon{
    /**
     * @brief Get the Trajectory From Csv File object.
     * 
     * @param filename Csv file to get the data from.
     * @return Trajectory data as a vector constructed with arrays of 8 elements [frame, x, y, z, qx, qy, qz, qw]. Each array represents one frame and pose.
     */
    std::vector<std::array<double,8>> getTrajectoryFromCsvFile(const std::string& filename);
 
    /**
     * @brief Print in the console the trajectory stored in data.
     * 
     * @param data Trajectory data stored as blocks of 8 elements [frame, x, y, z, qx, qy, qz, qw]
     */
    void printTrajectoryData(const std::vector<std::array<double,8>> &data);

    /**
     * @brief Get the Trajectory Poses object
     * 
     * @param trajectoryData  Trajectory data stored as blocks of 8 elements [frame, x, y, z, qx, qy, qz, qw]
     * @param numPoses Number of poses of the initial trajectory to generate the discrete trajectory.
     * @param discreteTrajectory Discrete trajectory constructed as a array with numPoses*7 elements. Each group of 7 elements in the vector represent one discrete pose. 
     * @param size size of the discrete trajectory arrray
     */

    void getTrajectoryPoses(const std::vector<std::array<double, 8>> &trajectoryData, const unsigned int &numPoses, std::vector<double>& discreteTrajectory);
    

    void getTrajectoryPoses(const std::vector<std::array<double, 8>> &trajectoryData, const float &distBtwPoses, std::vector<double> &discreteTrajectory, unsigned int & sizeDiscreteTrajectory);

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

    double optimizationFunction(unsigned n, const double *x, double * grad, void * data);

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
    void quaternionConstraint(unsigned m,double * result, unsigned n, const double *x, double * grad, void *data);


    void noSelfCollisionConstraint(unsigned m, double * result, unsigned n, const double * x, double * grad, void * data);


    void writeCsv(std::string filename, const double * x, const unsigned int & numPoses);
    
    void writeQCsv(std::string filename, const std::vector<std::array<double,8>>& qTraj);

}// namespace sharon


#endif //__MAP_HUMAN_HAND_TRAJECTORY_TO_ROBOT_END_EFFECTOR_HPP__