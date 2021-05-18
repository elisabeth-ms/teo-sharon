#ifndef __MAP_HUMAN_HAND_TRAJECTORY_TO_ROBOT_END_EFFECTOR_HPP__
#define __MAP_HUMAN_HAND_TRAJECTORY_TO_ROBOT_END_EFFECTOR_HPP__

#include <string>
#include <vector>
#include <array>

namespace sharon{
    std::vector<std::array<double,8>> getTrajectoryFromCsvFile(const std::string& filename);
    void printTrajectoryData(const std::vector<std::array<double,8>> &data);


}// namespace sharon


#endif //__MAP_HUMAN_HAND_TRAJECTORY_TO_ROBOT_END_EFFECTOR_HPP__