#include "MapHumanHandTrajectoryToRobotEndEffector.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <nlopt.hpp>
#include <math.h>

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

/*** TODO: Include this functions in a common library ***/
static KDL::Chain makeTeoTrunkAndRightArmKinematicsFromDH()
{
    const KDL::Joint rotZ(KDL::Joint::RotZ);
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.0, -KDL::PI / 2, 0.1932, 0.0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.305, 0.0, -0.34692, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, -0.32901, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, KDL::PI / 2, 0, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, -0.215, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(-0.09, 0, 0, -KDL::PI / 2)));

    return chain;
}

void makeQLimitsTeoTrunkAndRightArmKinematics(KDL::JntArray & qmin, KDL::JntArray & qmax)
{
    qmin.resize(8);
    qmax.resize(8);
    qmin(0) = -59.3*KDL::deg2rad;
    qmin(1) = -15.4*KDL::deg2rad;
    qmin(2) = -98.1*KDL::deg2rad;
    qmin(3) = -75.5*KDL::deg2rad;
    qmin(4) = -80.1*KDL::deg2rad;
    qmin(5) = -99.6*KDL::deg2rad;
    qmin(6) = -80.4*KDL::deg2rad;
    qmin(7) = -115.1*KDL::deg2rad;
    qmax(0) = 46.3*KDL::deg2rad;
    qmax(1) = 10.1*KDL::deg2rad;
    qmax(2) = 106.0*KDL::deg2rad;
    qmax(3) = 22.4*KDL::deg2rad;
    qmax(4) = 57.0*KDL::deg2rad;
    qmax(5) = 98.4*KDL::deg2rad;
    qmax(6) = 99.6*KDL::deg2rad;
    qmax(7) = 44.7*KDL::deg2rad;

}
/*** ------------------------------------------------------------------------------------------------ ***/

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

    void getTrajectoryPoses(const std::vector<std::array<double, 8>> &trajectoryData, const unsigned int &numPoses, double *discreteTrajectory, unsigned int &size)
    {
        //std::vector<double> desiredTrajectory;

        unsigned int step = trajectoryData.size() / numPoses;
        std::cout << "size: " << trajectoryData.size() << std::endl;
        std::cout << "step: " << step << std::endl;

        unsigned int count = 0;
        unsigned int iPose = 0;

        while (count < numPoses - 1)
        {
            for (unsigned int i = 1; i < trajectoryData[iPose].size(); i++)
            {
                discreteTrajectory[(i - 1) + 7 * count] = trajectoryData[iPose][i];
                //desiredTrajectory.push_back(trajectoryData[iPose][i]);
            }
            count++;
            iPose += step;
        }

        for (unsigned int i = 1; i < trajectoryData[0].size(); i++)
        {
            discreteTrajectory[(i - 1) + 7 * count] = trajectoryData.back()[i];
        }
    }

    double optimizationFunction(unsigned n, const double *x, double *grad, void *data)
    {
        auto *voidToVector = reinterpret_cast<std::vector<double> *>(data);
        std::cout << (*voidToVector)[0] << " " << (*voidToVector)[1] << " " << (*voidToVector)[2] << " " << (*voidToVector)[3] << " " << (*voidToVector)[4] << " " << (*voidToVector)[5] << " " << (*voidToVector)[6] << std::endl;
        std::cout << x[0] << " " << x[1] << " " << x[2] << " " << x[3] << " " << x[4] << " " << x[5] << " " << x[6] << std::endl;
        double sum = 0;
        for (unsigned int i = 0; i < n; i++)
        {
            sum += (x[i] - (*voidToVector)[i]) * (x[i] - (*voidToVector)[i]);
        }
        std::cout << sum << std::endl;
        return sum;
    }

    void quaternionConstraint(unsigned m, double *result, unsigned n, const double *x, double *grad, void *data)
    {

        for (unsigned int iq = 0; iq < (unsigned int)m / 3; iq++)
        {
            result[iq] = sqrt(x[iq * 7 + 3] * x[iq * 7 + 3] + x[iq * 7 + 4] * x[iq * 7 + 4] + x[iq * 7 + 5] * x[iq * 7 + 5] + x[iq * 7 + 6] * x[iq * 7 + 6]) - 1;
            // std::cout<<"constraint: "<<result[iq]<<std::endl;
        }
    }

}

int main()
{
    std::string csvFile = "../programs/MapHumanHandTrajectoryToRobotEndEffector/test-right-arm-motion-smooth1.csv";
    std::vector<std::array<double, 8>> desiredTrajectoryData = sharon::getTrajectoryFromCsvFile(csvFile);
    sharon::printTrajectoryData(desiredTrajectoryData);

    unsigned int numPoses = 40;
    unsigned int sizeDesiredDiscretePoses = numPoses * 7;
    double desiredDiscretePoses[sizeDesiredDiscretePoses];

    sharon::getTrajectoryPoses(desiredTrajectoryData, numPoses, desiredDiscretePoses, sizeDesiredDiscretePoses);




    unsigned int n = sizeDesiredDiscretePoses;
    double x[n];
    std::vector<double> vectorX;
    for (unsigned int i = 0; i < sizeDesiredDiscretePoses; i++)
    {
        x[i] = desiredDiscretePoses[i];
        vectorX.push_back(x[i]);
    }

    KDL::Chain chain = makeTeoTrunkAndRightArmKinematicsFromDH();
    KDL::JntArray qmin, qmax;
    makeQLimitsTeoTrunkAndRightArmKinematics(qmin, qmax);
    KDL::ChainFkSolverPos_recursive fksolver(chain);
    KDL::ChainIkSolverVel_pinv iksolverv(chain);
    KDL::ChainIkSolverPos_NR_JL iksolver(chain, qmin, qmax, fksolver, iksolverv, 1000);

    KDL::JntArray q(chain.getNrOfJoints());
    KDL::JntArray qInit(chain.getNrOfJoints());


    KDL::Frame fGoal;
    double norm = sqrt(x[3]*x[3] + x[4]*x[4] + x[5]*x[5] + x[6]*x[6]);
    KDL::Rotation rotKdl = KDL::Rotation::Quaternion(x[3]/norm, x[4]/norm, x[5]/norm, x[6]/norm);
    KDL::Vector posKdl = KDL::Vector(x[0], x[1], x[2]-0.894);
    rotKdl = rotKdl*KDL::Rotation::RotX(-KDL::PI/2.0);
    rotKdl = rotKdl*KDL::Rotation::RotZ(-KDL::PI/2.0-KDL::PI/4.0);
    fGoal.M = rotKdl;
    fGoal.p = posKdl;

    std::cout<<x[0]<<" "<<x[1]<<" "<<x[2]<<std::endl;
    


    int ret = iksolver.CartToJnt(qInit, fGoal, q);

    std::cout<<ret<<std::endl;

    // void *parameters = &vectorX;

    // nlopt_opt opt;
    // opt = nlopt_create(NLOPT_LN_COBYLA, n);
    // nlopt_set_min_objective(opt, sharon::optimizationFunction, parameters);
    // double lb[n];
    // double ub[n];

    // for (unsigned int i = 0; i < n; i++)
    // {
    //     // Our vector x is constructed like [x0 y0 z0 qx0 qy0 qz0 qw0 ... xi yi zi qxi qyi qzi qwi ... xn yn zn qxn qyn qzn qwn]. Where each group of 7 elements
    //     // represents a pose
    //     if (i < (((unsigned int)(i / 7) + 1) * 7 - 4)) // Bounds for position elements [x y z]
    //     {
    //         lb[i] = -1.3;
    //         ub[i] = 1.3;
    //     }
    //     else
    //     { // Bounds for quaternion elements [qx qy qz qw]
    //         lb[i] = -1;
    //         ub[i] = 1.0;
    //     }
    // }

    // nlopt_set_lower_bounds(opt, lb);
    // nlopt_set_upper_bounds(opt, ub);

    // // //     std::vector<double> tolQuaternionConstraint;
    // // //     tolQuaternionConstraint.push_back(1e-6);

    // unsigned int m = numPoses * 3;
    // double tolQuaternionConstraint[m];
    // for (unsigned int i = 0; i < m; i++)
    //     tolQuaternionConstraint[i] = 1e-4;

    // nlopt_add_equality_mconstraint(opt, m, sharon::quaternionConstraint, NULL, tolQuaternionConstraint);
    // nlopt_set_xtol_rel(opt, 1e-5);
    // nlopt_set_maxtime(opt, 200);
    // //nlopt_set_xtol_abs(opt, tolQuaternionConstraint);

    // double minf;
    // try
    // {
    //     nlopt_result result = nlopt_optimize(opt, x, &minf);
    //     std::cout << "found minimum at f(x)=" << minf << std::endl;
    // }
    // catch (std::exception &e)
    // {
    //     std::cout << "nlopt failed: " << e.what() << std::endl;
    // }

    return 0;
}