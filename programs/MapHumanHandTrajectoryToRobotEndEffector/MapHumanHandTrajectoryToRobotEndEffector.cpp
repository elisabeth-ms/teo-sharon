#include "MapHumanHandTrajectoryToRobotEndEffector.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <nlopt.hpp>
#include <math.h>
namespace sharon
{
    std::vector<std::array<double, 8>> getTrajectoryFromCsvFile(const std::string &filename)
    {
        std::vector<std::array<double, 8>> result;
        std::cout << filename << std::endl;

        std::ifstream csvFile(filename);

        if (!csvFile.is_open())
            throw std::runtime_error("Could not open csv file");

        double val;
        std::string line;
        while (std::getline(csvFile, line))
        {
            std::stringstream ss(line);
            std::array<double, 8> pose;
            unsigned int colIdx = 0;
            while (ss >> val)
            {
                pose[colIdx] = val;
                if (ss.peek() == ',')
                {
                    ss.ignore();
                }

                colIdx++;
            }
            result.push_back(pose);
        }

        csvFile.close();
        return result;
    }

    void printTrajectoryData(const std::vector<std::array<double, 8>> &data)
    {
        for (unsigned int i = 0; i < data.size(); i++)
        {
            std::cout << "Frame: " << data[i][0] << " x: " << data[i][1] << " y: " << data[i][2] << " z: " << data[i][3] << " qx: " << data[i][4] << " qy: " << data[i][5] << " qz: " << data[i][6] << " qw: " << data[i][7] << std::endl;
        }
    }

    std::vector<double> getTrajectoryPoses(const std::vector<std::array<double, 8>> &trajectoryData, const unsigned int &numPoses)
    {
        std::vector<double> desiredTrajectory;

        unsigned int step = trajectoryData.size() / numPoses;
        std::cout << "size: " << trajectoryData.size() << std::endl;
        std::cout << "step: " << step << std::endl;

        unsigned int count = 0;
        unsigned int iPose = 0;

        while (count < numPoses - 1)
        {
            for (unsigned int i = 1; i < trajectoryData[iPose].size(); i++)
            {
                desiredTrajectory.push_back(trajectoryData[iPose][i]);
            }
            count++;
            iPose += step;
        }

        for (unsigned int i = 1; i < trajectoryData[0].size(); i++)
        {
            desiredTrajectory.push_back(trajectoryData.back()[i]);
        }

        return desiredTrajectory;
    }

    double optimizationFunction(const std::vector<double> &x, std::vector<double> &grad, void *data)
    {
        auto *voidToVector = reinterpret_cast<std::vector<double> *>(data);
        //std::cout << (*voidToVector)[0] << " " << (*voidToVector)[1] << " " << (*voidToVector)[2] << " " << (*voidToVector)[3] << " " << (*voidToVector)[4] << " " << (*voidToVector)[5] << " " << (*voidToVector)[6] << std::endl;
        double sum = 0;
        for (unsigned int i = 0; i < x.size(); i++)
        {
            sum += (x[i] - (*voidToVector)[i]) * (x[i] - (*voidToVector)[i]);
            
        }
        std::cout << sum << std::endl;
        return sum;
    }

    //
    double quaternionConstraint(const std::vector<double> &x, std::vector<double> &grad, void *data)
    {
        return sqrt(x[3] * x[3] + x[4] * x[4] + x[5] * x[5] + x[6] * x[6]) - 1;
    }

    

}

int main()
{
    std::string csvFile = "../programs/MapHumanHandTrajectoryToRobotEndEffector/test-right-arm-motion1.csv";
    std::vector<std::array<double, 8>> desiredTrajectoryData = sharon::getTrajectoryFromCsvFile(csvFile);
    sharon::printTrajectoryData(desiredTrajectoryData);

    std::vector<double> desiredPoses = sharon::getTrajectoryPoses(desiredTrajectoryData, 100);
    void *parameters = &desiredPoses;

    std::vector<double> x(desiredPoses);
    // x[0] +=0.01;
    // x[1] += 0.05;
    std::vector<double> grad(desiredPoses);

    unsigned int nOfVariables = desiredPoses.size();
    nlopt::opt opt(nlopt::LN_COBYLA, nOfVariables);
    opt.set_min_objective(sharon::optimizationFunction, parameters);
    std::vector<double> lb(nOfVariables);
    std::vector<double> ub(nOfVariables);

    for (unsigned int i = 0; i < nOfVariables; i++)
    {
        // Our vector x is constructed like [x0 y0 z0 qx0 qy0 qz0 qw0 ... xi yi zi qxi qyi qzi qwi ... xn yn zn qxn qyn qzn qwn]. Where each group of 7 elements
        // represents a pose
        if(i<(( (unsigned int) (i/7)+1)*7-4)) // Bounds for position elements [x y z]
        {   
            lb[i] = -1.3;
            ub[i] = 1.3;
        }
        else{ // Bounds for quaternion elements [qx qy qz qw]
            lb[i] = -1;
            ub[i] = 1.0;
        }
    }

    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);

    opt.add_equality_constraint(sharon::quaternionConstraint, NULL, 1e-6);

    opt.set_ftol_abs(1e-3);
    opt.set_xtol_rel(1e-3);
    opt.set_stopval(1e-5);

    double minf;
    try
    {
        nlopt::result result = opt.optimize(x, minf);
        std::cout << "found minimum at f(x)=" << minf << std::endl;
    }
    catch (std::exception &e)
    {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }

    // nlopt::opt opt(nlopt::LD_MMA,7); // TODO: Check the optimization method to be used

    // opt.add_equality_constraint(sharon::quaternionConstraint, NULL, 1e-8);

    return 0;
}