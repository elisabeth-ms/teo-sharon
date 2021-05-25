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

#include <iomanip> // std::setprecision

KDL::ChainIkSolverPos_NR_JL *iksolver;
std::vector<double> qTrajectory;
std::vector<fcl::CollisionObject> collisionObjects;
std::vector<std::array<float, 3>> offsetCollisionObjects;
typedef std::shared_ptr<fcl::CollisionGeometry> CollisionGeometryPtr_t;
sharon::CheckSelfCollision *checkSelfCollision;

/*** TODO: Include this functions in a common library ***/
static KDL::Chain makeTeoTrunkAndRightArmKinematicsFromDH()
{
    const KDL::Joint rotZ(KDL::Joint::RotZ);
    const KDL::Joint fixed(KDL::Joint::None); //Rigid Connection

    KDL::Chain chain;
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.0, -KDL::PI / 2, 0.1932, 0.0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.305, 0.0, -0.34692, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, 0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, -0.32901, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, KDL::PI / 2, 0, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0, -KDL::PI / 2, -0.215, 0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(-0.09, 0, 0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0, KDL::PI / 2, 0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0, 0, 0.0975, 0)));

    return chain;
}

void makeQLimitsTeoTrunkAndRightArmKinematics(KDL::JntArray &qmin, KDL::JntArray &qmax)
{
    qmin.resize(8);
    qmax.resize(8);
    qmin(0) = -59.3 * KDL::deg2rad;
    qmin(1) = -15.4 * KDL::deg2rad;
    qmin(2) = -98.1 * KDL::deg2rad;
    qmin(3) = -75.5 * KDL::deg2rad;
    qmin(4) = -80.1 * KDL::deg2rad;
    qmin(5) = -99.6 * KDL::deg2rad;
    qmin(6) = -80.4 * KDL::deg2rad;
    qmin(7) = -115.1 * KDL::deg2rad;
    qmax(0) = 46.3 * KDL::deg2rad;
    qmax(1) = 10.1 * KDL::deg2rad;
    qmax(2) = 106.0 * KDL::deg2rad;
    qmax(3) = 22.4 * KDL::deg2rad;
    qmax(4) = 57.0 * KDL::deg2rad;
    qmax(5) = 98.4 * KDL::deg2rad;
    qmax(6) = 99.6 * KDL::deg2rad;
    qmax(7) = 44.7 * KDL::deg2rad;
}
void createSelfCollisionObjects()
{

    CollisionGeometryPtr_t teoRootTrunk{new fcl::Box{0.25, 0.25, 0.6}};
    fcl::Transform3f tfTest{fcl::Vec3f{0.0, 0.0, 0.0}};
    fcl::CollisionObject collisionObject1{teoRootTrunk, tfTest};

    CollisionGeometryPtr_t teoTrunk{new fcl::Box{0.3, 0.3, 0.46}};
    fcl::CollisionObject collisionObject2{teoTrunk, tfTest};

    CollisionGeometryPtr_t teoAxialShoulder{new fcl::Box{0.10, 0.10, 0.32901}}; //{new fcl::Box{0.15, 0.15, 0.32901}};
    fcl::CollisionObject collisionObject3{teoAxialShoulder, tfTest};

    CollisionGeometryPtr_t teoElbow{new fcl::Box{0.10, 0.10, 0.22}};
    fcl::CollisionObject collisionObject4{teoElbow, tfTest};

    CollisionGeometryPtr_t teoWrist{new fcl::Box{0.2, 0.20, 0.2}};
    fcl::CollisionObject collisionObject5{teoWrist, tfTest};

    CollisionGeometryPtr_t teoHand{new fcl::Box{0.01, 0.01, 0.01}};
    fcl::CollisionObject collisionObject6{teoHand, tfTest};

    int nOfCollisionObjects = 6;
    collisionObjects.clear();
    collisionObjects.reserve(nOfCollisionObjects);
    collisionObjects.emplace_back(collisionObject1);
    collisionObjects.emplace_back(collisionObject2);
    collisionObjects.emplace_back(collisionObject3);
    collisionObjects.emplace_back(collisionObject4);
    collisionObjects.emplace_back(collisionObject5);
    collisionObjects.emplace_back(collisionObject6);

    offsetCollisionObjects.resize(nOfCollisionObjects);
    std::array<float, 3> offsetObject = {0, 0, 0};
    offsetCollisionObjects[0][2] = -0.2;
    offsetCollisionObjects[1][1] = 0.0;
    offsetCollisionObjects[1][2] = +0.1734;
    offsetCollisionObjects[4][1] = 0.055;
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

    void getTrajectoryPoses(const std::vector<std::array<double, 8>> &trajectoryData, const unsigned int &numPoses, std::vector<double> &discreteTrajectory)
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
                //discreteTrajectory[(i - 1) + 7 * count] = trajectoryData[iPose][i];
                discreteTrajectory.push_back(trajectoryData[iPose][i]);
                //desiredTrajectory.push_back(trajectoryData[iPose][i]);
            }
            count++;
            iPose += step;
        }

        for (unsigned int i = 1; i < trajectoryData[0].size(); i++)
        {
            //discreteTrajectory[(i - 1) + 7 * count] = trajectoryData.back()[i];
            discreteTrajectory.push_back(trajectoryData.back()[i]);
        }
    }

    void getTrajectoryPoses(const std::vector<std::array<double, 8>> &trajectoryData, const float &distBtwPoses, std::vector<double> &discreteTrajectory, unsigned int & sizeDiscreteTrajectory)
    {
        float dist = std::numeric_limits<float>::max();
        sizeDiscreteTrajectory = 0;
        for (unsigned int i = 0; i < trajectoryData.size(); i++)
        {
            if (i == 0 || i == (trajectoryData.size() - 1))
            {
                for (unsigned int j = 1; j < trajectoryData[i].size(); j++)
                {
                    discreteTrajectory.push_back(trajectoryData[i][j]);
                }
                sizeDiscreteTrajectory++;
            }
            else
            {
                dist = sqrt(pow((trajectoryData[i][1] - discreteTrajectory[discreteTrajectory.size() - 7]), 2) + pow((trajectoryData[i][2] - discreteTrajectory[discreteTrajectory.size() - 6]), 2) + pow((trajectoryData[i][3] - discreteTrajectory[discreteTrajectory.size() - 5]), 2));
                std::cout << dist << std::endl;
                if (dist >= distBtwPoses)
                {
                    for (unsigned int j = 1; j < trajectoryData[i].size(); j++)
                    {
                        discreteTrajectory.push_back(trajectoryData[i][j]);
                    }
                    sizeDiscreteTrajectory++;
                }
            }

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
        auto *voidToVector = reinterpret_cast<std::vector<double> *>(data);
        (*voidToVector)[0] = 1.0;
    }

    void noSelfCollisionConstraint(unsigned m, double *result, unsigned n, const double *x, double *grad, void *data)
    {
        //auto *voidToVector = reinterpret_cast<std::vector<double> *>(data);
        KDL::JntArray qInit(8); // TODO correct in order to gather the q init from outside and also the size of the joints array
        KDL::JntArray q(8);

        for (unsigned int i = 0; i < n; i++)
        {
            KDL::Frame fGoal;
            double norm = sqrt(x[(i * 7) + 3] * x[i * 7 + 3] + x[i * 7 + 4] * x[i * 7 + 4] + x[i * 7 + 5] * x[i * 7 + 5] + x[i * 7 + 6] * x[i * 7 + 6]);
            KDL::Rotation rotKdl = KDL::Rotation::Quaternion(x[i * 7 + 3] / norm, x[i * 7 + 4] / norm, x[i * 7 + 5] / norm, x[i * 7 + 6] / norm);
            KDL::Vector posKdl = KDL::Vector(x[i * 7 + 0], x[i * 7 + 1], x[i * 7 + 2] - 0.894);
            rotKdl = rotKdl * KDL::Rotation::RotX(-KDL::PI / 2.0);
            rotKdl = rotKdl * KDL::Rotation::RotZ(-KDL::PI / 2.0 - KDL::PI / 4.0);
            fGoal.M = rotKdl;
            fGoal.p = posKdl;

            int foundIk = (*iksolver).CartToJnt(qInit, fGoal, q);
            if (foundIk == 0) //Ik found
            {
                for (unsigned int i = 0; i < q.rows(); i++)
                    q(i) = q(i) * KDL::rad2deg;
                checkSelfCollision->updateCollisionObjectsTransform(q);
                if (checkSelfCollision->selfCollision() == false)
                {
                    for (unsigned int joint = 0; joint < qInit.rows(); joint++)
                    {
                        qInit(joint) = q(joint) * KDL::deg2rad;
                    }
                    result[i] = 0;
                    //std::cout << "q: " << q(0) << " " << q(1) << " " << q(2) << " " << q(3) << " " << q(4) << " " << q(5) << " " << q(6) << " " << q(7) << std::endl;
                }
                else
                {
                    result[i] = 1;
                    //std::cout << "self collision" << std::endl;
                }
            }
            else //Ik not found
            {
                result[i] = foundIk;
            }
        }
    }
    void writeCsv(std::string filename, const double *x, const unsigned int &numPoses)
    {

        std::ofstream myFile(filename);
        for (unsigned int i = 0; i < numPoses; i++)
        {
            double norm = sqrt(x[(i * 7) + 3] * x[i * 7 + 3] + x[i * 7 + 4] * x[i * 7 + 4] + x[i * 7 + 5] * x[i * 7 + 5] + x[i * 7 + 6] * x[i * 7 + 6]);
            myFile << i << "," << std::setprecision(14) << x[i * 7 + 0] << "," << x[i * 7 + 1] << "," << x[i * 7 + 2] << "," << x[i * 7 + 3] / norm << "," << x[i * 7 + 4] / norm << "," << x[i * 7 + 5] / norm << "," << x[i * 7 + 6] / norm << "\n";
        }
        myFile.close();
    }

    void writeQCsv(std::string filename, const std::vector<std::array<double, 8>> &qTraj)
    {
        std::ofstream myFile(filename);
        for (unsigned int i = 0; i < qTraj.size(); i++)
        {
            myFile << i << "," << std::setprecision(10) << qTraj[i][0] << "," << qTraj[i][1] << "," << qTraj[i][2] << "," << qTraj[i][3] << "," << qTraj[i][4]
                   << "," << qTraj[i][5] << "," << qTraj[i][6] << "," << qTraj[i][7] << "\n";
        }
        myFile.close();
    }

}

int main()
{
    std::string csvFile = "/home/elisabeth/repos/teo-sharon/programs/GenerateManipulationTrajectories/trajectories/graspcup1/test-right-arm-motion-smooth1-aproach.csv";
    std::vector<std::array<double, 8>> desiredTrajectoryData = sharon::getTrajectoryFromCsvFile(csvFile);
    sharon::printTrajectoryData(desiredTrajectoryData);

    unsigned int numPoses = 0;
    std::vector<double> desiredDiscretePoses;

    //sharon::getTrajectoryPoses(desiredTrajectoryData, numPoses, desiredDiscretePoses);

    sharon::getTrajectoryPoses(desiredTrajectoryData, (float)0.005, desiredDiscretePoses, numPoses);

    unsigned int n = desiredDiscretePoses.size();
    double x[n];
    for (unsigned int i = 0; i < n; i++)
    {
        x[i] = desiredDiscretePoses[i];

    }
    std::cout << "Num poses: " << numPoses << std::endl;

    KDL::Chain chain = makeTeoTrunkAndRightArmKinematicsFromDH();
    KDL::JntArray qmin(8), qmax(8);
    makeQLimitsTeoTrunkAndRightArmKinematics(qmin, qmax);

    KDL::ChainFkSolverPos_recursive fksolver(chain);
    KDL::ChainIkSolverVel_pinv iksolverv(chain);
    iksolver = new KDL::ChainIkSolverPos_NR_JL(chain, qmin, qmax, fksolver, iksolverv, 200, 1e-6);

    KDL::JntArray q(chain.getNrOfJoints());
    KDL::JntArray qInit(chain.getNrOfJoints());

    qTrajectory.resize((int)(desiredDiscretePoses.size() / 7) * 8);

    void *parameters = &qTrajectory;

    createSelfCollisionObjects();

    checkSelfCollision = new sharon::CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects);

    nlopt_opt opt;
    opt = nlopt_create(NLOPT_LN_COBYLA, n);
    nlopt_set_min_objective(opt, sharon::optimizationFunction, parameters);
    double lb[n];
    double ub[n];

    for (unsigned int i = 0; i < n; i++)
    {
        // Our vector x is constructed like [x0 y0 z0 qx0 qy0 qz0 qw0 ... xi yi zi qxi qyi qzi qwi ... xn yn zn qxn qyn qzn qwn]. Where each group of 7 elements
        // represents a pose
        if (i < (((unsigned int)(i / 7) + 1) * 7 - 4)) // Bounds for position elements [x y z]
        {
            lb[i] = -1.3;
            ub[i] = 1.3;
        }
        else
        { // Bounds for quaternion elements [qx qy qz qw]
            lb[i] = -1;
            ub[i] = 1.0;
        }
    }

    nlopt_set_lower_bounds(opt, lb);
    nlopt_set_upper_bounds(opt, ub);

    // //     std::vector<double> tolQuaternionConstraint;
    // //     tolQuaternionConstraint.push_back(1e-6);

    unsigned int m = numPoses * 3;
    double tolQuaternionConstraint[m];
    for (unsigned int i = 0; i < m; i++)
        tolQuaternionConstraint[i] = 1e-4;

    double tol[numPoses];
    for (unsigned int i = 0; i < numPoses; i++)
        tol[i] = 1e-6;

    nlopt_add_equality_mconstraint(opt, m, sharon::quaternionConstraint, parameters, tolQuaternionConstraint);
    nlopt_add_equality_mconstraint(opt, numPoses, sharon::noSelfCollisionConstraint, NULL, tol);
    nlopt_set_xtol_rel(opt, 1e-6);
    nlopt_set_maxtime(opt, 500);

    double minf;
    try
    {
        nlopt_result result = nlopt_optimize(opt, x, &minf);
        std::cout << "found minimum at f(x)=" << minf << std::endl;
        KDL::JntArray qInit(8); // TODO correct in order to gather the q init from outside and also the size of the joints array
        KDL::JntArray q(8);
        std::vector<std::array<double, 8>> qTraj;
        std::array<double, 8> qArray;
        for (unsigned int i = 0; i < numPoses; i++)
        {
            KDL::Frame fGoal;
            double norm = sqrt(x[(i * 7) + 3] * x[i * 7 + 3] + x[i * 7 + 4] * x[i * 7 + 4] + x[i * 7 + 5] * x[i * 7 + 5] + x[i * 7 + 6] * x[i * 7 + 6]);
            KDL::Rotation rotKdl = KDL::Rotation::Quaternion(x[i * 7 + 3] / norm, x[i * 7 + 4] / norm, x[i * 7 + 5] / norm, x[i * 7 + 6] / norm);
            KDL::Vector posKdl = KDL::Vector(x[i * 7 + 0], x[i * 7 + 1], x[i * 7 + 2] - 0.894);
            rotKdl = rotKdl * KDL::Rotation::RotX(-KDL::PI / 2.0);
            rotKdl = rotKdl * KDL::Rotation::RotZ(-KDL::PI / 2.0 - KDL::PI / 4.0);
            fGoal.M = rotKdl;
            fGoal.p = posKdl;
            bool result = (*iksolver).CartToJnt(qInit, fGoal, q);
            if (result >= 0)
            {
                for (unsigned int joint = 0; joint < qInit.rows(); joint++)
                {
                    qInit(joint) = q(joint);
                    qArray[joint] = q(joint) * KDL::rad2deg;
                }
                qTraj.push_back(qArray);
                // std::cout << "qTraj: " << qTrajectory[i * 8 + 0] << " " << qTrajectory[i * 8 + 1] << " " << qTrajectory[i * 8 + 2] << " " << qTrajectory[i * 8 + 3] << " " << qTrajectory[i * 8 + 4] << " " << qTrajectory[i * 8 + 5] << " " << qTrajectory[i * 8 + 6] << " " << qTrajectory[i * 8 + 7] << std::endl;
                // std::cout << "q: " << q(0) * KDL::rad2deg << " " << q(1) * KDL::rad2deg << " " << q(2) * KDL::rad2deg << " " << q(3) * KDL::rad2deg << " " << q(4) * KDL::rad2deg << " " << q(5) * KDL::rad2deg << " " << q(6) * KDL::rad2deg << " " << q(7) * KDL::rad2deg << std::endl;
                // std::cout << x[i * 7 + 0] << " " << x[i * 7 + 1] << " " << x[i * 7 + 2] << x[i * 7 + 3] << " " << x[i * 7 + 4] << " " << x[i * 7 + 5] << " " << x[i * 7 + 6] << std::endl;
            }
            std::cout << result << std::endl;
        }
        std::string csvFileWrite = "/home/elisabeth/repos/teo-sharon/programs/GenerateManipulationTrajectories/trajectories/graspcup1/test-right-arm-motion-smooth1-optimized.csv";
        std::string csvQFileWrite = "/home/elisabeth/repos/teo-sharon/programs/GenerateManipulationTrajectories/trajectories/graspcup1/test-right-arm-motion-smooth1-joint.csv";
        sharon::writeCsv(csvFileWrite, x, numPoses);
        sharon::writeQCsv(csvQFileWrite, qTraj);
    }
    catch (std::exception &e)
    {
        std::cout << "nlopt failed: " << e.what() << std::endl;
    }

    return 0;
}