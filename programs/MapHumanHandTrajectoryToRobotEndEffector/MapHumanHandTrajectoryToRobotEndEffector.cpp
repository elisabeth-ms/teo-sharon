#include "MapHumanHandTrajectoryToRobotEndEffector.hpp"

#include <iostream>
#include <fstream>
#include <sstream>

namespace sharon{
    std::vector<std::array<double,8>> getTrajectoryFromCsvFile(const std::string& filename){
        std::vector<std::array<double,8>> result;
        std::cout<<filename<<std::endl;

        std::ifstream csvFile(filename);

        if(!csvFile.is_open())
            throw std::runtime_error("Could not open csv file");

        double val;
        std::string line;
        while(std::getline(csvFile, line)){
            std::stringstream ss(line);
            std::array<double, 8> pose;
            unsigned int colIdx = 0;
            while(ss>>val){
                pose[colIdx] = val;
                if(ss.peek() == ','){
                    ss.ignore();
                }
                
                colIdx++;
                
            }
            result.push_back(pose);

        }

        csvFile.close();
        return result;
    }

    void printTrajectoryData(const std::vector<std::array<double,8>> &data){
        for(unsigned int i= 0; i<data.size(); i++){
            std::cout<<"Frame: "<<data[i][0]<<" x: "<<data[i][1]<<" y: "<<data[i][2]<<" z: "<<data[i][3]<<" qx: "<<data[i][4]<<
                        " qy: "<<data[i][5]<<" qz: "<<data[i][6]<<" qw: "<<data[i][7]<<std::endl;
        }

    }
}


int main(){
    std::string csvFile = "../programs/MapHumanHandTrajectoryToRobotEndEffector/test-right-arm-motion1.csv";
    std::vector<std::array<double, 8>> trajectoryData = sharon::getTrajectoryFromCsvFile(csvFile);
    sharon::printTrajectoryData(trajectoryData);
    return 0;

}