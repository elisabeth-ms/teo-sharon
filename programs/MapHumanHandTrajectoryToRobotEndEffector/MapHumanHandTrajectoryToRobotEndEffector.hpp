#ifndef __MAP_HUMAN_HAND_TRAJECTORY_TO_ROBOT_END_EFFECTOR_HPP__
#define __MAP_HUMAN_HAND_TRAJECTORY_TO_ROBOT_END_EFFECTOR_HPP__

#include <string>
#include <vector>
#include <array>

namespace sharon{
    std::vector<std::array<double,8>> getTrajectoryFromCsvFile(const std::string& filename);
    void printTrajectoryData(const std::vector<std::array<double,8>> &data);
    std::vector<double> getTrajectoryPoses(const std::vector<std::array<double,8>>& trajectoryData, const unsigned int &numPoses);
    double optimizationFunction(const std::vector<double> &x, std::vector<double>&grad, void * data);


}// namespace sharon


#endif //__MAP_HUMAN_HAND_TRAJECTORY_TO_ROBOT_END_EFFECTOR_HPP__