#ifndef __MAP_HUMAN_HAND_TRAJECTORY_TO_ROBOT_END_EFFECTOR_HPP__
#define __MAP_HUMAN_HAND_TRAJECTORY_TO_ROBOT_END_EFFECTOR_HPP__

#include <string>
#include <vector>
#include <array>

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
     * @return  Discrete trajectory constructed as a vector with numPoses*7 elements. Each group of 7 elements in the vector represent one discrete pose. 
     */
    std::vector<double> getTrajectoryPoses(const std::vector<std::array<double,8>>& trajectoryData, const unsigned int &numPoses);
    
    /**
     * @brief Optimization function to minimize the distance between the pose of the desired trajectory and the final trajectory that satisfies the constraints.
     * This function is defined to be used with nlopt library. 
     * 
     * @param x Optimization parameters.
     * @param grad Gradient of the optimization function with respect to the optimization pararameters.
     * @param data Additional data to be used in the optimization function.
     * @return Value of the optimization function at x.
     */
    double optimizationFunction(const std::vector<double> &x, std::vector<double>&grad, void * data);

    /**
     * @brief Constraint for only allowing normalized quaternions.
     * 
     * @param x Optimation parameters that represent the final trajectory. This trajectory is a row vector, where each group of 7 continuous elements represent one pose.   
     * @param grad Gradient of the optimization function with respect to the optimization parameters.
     * @param data Additional data to be used in the constraint function.
     * @return norm(quaternion)-1
     */
    double quaternionConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data);

}// namespace sharon


#endif //__MAP_HUMAN_HAND_TRAJECTORY_TO_ROBOT_END_EFFECTOR_HPP__