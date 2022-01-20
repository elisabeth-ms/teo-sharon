#include "MapHumanHandTrajectoryToRobotJointSpace.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <nlopt.hpp>
#include <math.h>
#include <iomanip> // std::setprecision

KDL::ChainIkSolverPos_NR_JL *iksolver;
KDL::ChainFkSolverPos_recursive *fksolver;
std::vector<double> qTrajectory;
std::vector<fcl::CollisionObjectf> collisionObjects;
std::vector<std::array<float, 3>> offsetCollisionObjects;
typedef std::shared_ptr<fcl::CollisionGeometryf> CollisionGeometryPtr_t;
sharon::CheckSelfCollision *checkSelfCollision;
std::vector<fcl::CollisionObjectf> tableCollision;

unsigned int numPoses = 0;
unsigned int indexDiscrete = 0;

unsigned int iteration = 0;

float wpos = 250.0; // weight for end-effector position matching objective term
float wori = 1.0; // weight for end-effector orientation matching objective term
float wjv = 25.0;   // weight for joint velocity objective term
float wja = 10.0;   // weight for joint acceleration objective term
float wjj = 8.0;   // weight for joint jerk objective term
float wv = 5.0;    // weight for end-effector position velocity objective term
float wsc = 1.0;   // weight for self-collision avoidance objective term

std::vector<std::array<double, NJoints>> xprev;
std::vector<std::array<double, NJoints>> velocity;
std::vector<std::array<double, NJoints>> acceleration;
std::vector<std::array<double, NJoints>> jerk;
std::vector<std::array<double, 7>> poses;
std::vector<double>orientatationdist;

float initialDistances[15] = {0.118561 * 1, 0.0804199 * 1, 0.0740597 * 1, 0.140743 * 1, 0.0568082 * 1, 0.0635505 * 1, 0.146518 * 1, 0.151592 * 1, 0.271188 * 1, 0.106259 * 1};
float q_endEffector_HumanHand[4] = {-0.142387024727039, 0.778327151043027, 0.486859494151674, 0.370000829854479};

/*** TODO: Include this functions in a common library ***/
static KDL::Chain makeTeoTrunkAndRightArmKinematicsFromDH()
{
    const KDL::Joint rotZ(KDL::Joint::RotZ);
    const KDL::Joint fixed(KDL::Joint::None); // Rigid Connection
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
    qmin.resize(NJoints);
    qmax.resize(NJoints);
    qmin(0) = -31.0;
    qmin(1) = -10.1;
    qmin(2) = -98.1;
    qmin(3) = -75.5;
    qmin(4) = -80.1;
    qmin(5) = -99.6;
    qmin(6) = -80.4;
    qmin(7) = -115.1;
    qmax(0) = 31.0;
    qmax(1) = 20.5;
    qmax(2) = 106.0;
    qmax(3) = 22.4;
    qmax(4) = 57.0;
    qmax(5) = 98.4;
    qmax(6) = 99.6;
    qmax(7) = 44.7;
}

void createSelfCollisionObjects()
{

    CollisionGeometryPtr_t teoRootTrunk{new fcl::Boxf{0.35, 0.4, 0.75}};
    fcl::Transform3f tfTest;
    fcl::CollisionObjectf collisionObject1{teoRootTrunk, tfTest};

    CollisionGeometryPtr_t teoTrunk{new fcl::Boxf{0.3, 0.3, 0.46}};
    fcl::CollisionObjectf collisionObject2{teoTrunk, tfTest};

    CollisionGeometryPtr_t teoAxialShoulder{new fcl::Boxf{0.10, 0.10, 0.32901}}; //{new fcl::Box{0.15, 0.15, 0.32901}};
    fcl::CollisionObjectf collisionObject3{teoAxialShoulder, tfTest};

    CollisionGeometryPtr_t teoElbow{new fcl::Boxf{0.10, 0.10, 0.22}};
    fcl::CollisionObjectf collisionObject4{teoElbow, tfTest};

    CollisionGeometryPtr_t teoWrist{new fcl::Boxf{0.2, 0.2, 0.2}};
    fcl::CollisionObjectf collisionObject5{teoWrist, tfTest};

    CollisionGeometryPtr_t teoHand{new fcl::Boxf{0.05, 0.05, 0.05}};
    fcl::CollisionObjectf collisionObject6{teoHand, tfTest};

    CollisionGeometryPtr_t table{new fcl::Boxf{0.0, 0.0, 0.0}};
    fcl::CollisionObjectf collisionObjectTable{table, tfTest};
    fcl::Quaternionf rotation(1.0, 0.0, 0.0, 0.0);
    // fcl::Vector3f translation(1.65, 0.0, -0.43);
    fcl::Vector3f translation(2.0, 2.0, 0.0);
    collisionObjectTable.setTransform(rotation, translation);
    tableCollision.clear();
    tableCollision.push_back(collisionObjectTable);

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

namespace sharon
{
    std::vector<std::array<double, NJoints>> getTrajectoryFromCsvFile(const std::string &filename)
    {
        std::vector<std::array<double, NJoints>> result;
        std::cout << filename << std::endl;

        std::ifstream csvFile(filename);

        if (!csvFile.is_open())
            throw std::runtime_error("Could not open csv file");

        double val;
        std::string line;
        while (std::getline(csvFile, line))
        {
            std::stringstream ss(line);
            std::array<double, NJoints> pose;
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

    void printTrajectoryData(const std::vector<std::array<double, NJoints>> &data)
    {
        for (unsigned int i = 0; i < data.size(); i++)
        {
            std::cout << "Frame: " << data[i][0] << " x: " << data[i][1] << " y: " << data[i][2] << " z: " << data[i][3] << " qx: " << data[i][4] << " qy: " << data[i][5] << " qz: " << data[i][6] << " qw: " << data[i][7] << std::endl;
        }
    }

    void getTrajectoryPoses(const std::vector<std::array<double, NJoints>> &trajectoryData, const unsigned int &numPoses, std::vector<double> &discreteTrajectory)
    {
        // std::vector<double> desiredTrajectory;

        unsigned int step = trajectoryData.size() / numPoses;
        std::cout << "size: " << trajectoryData.size() << std::endl;
        std::cout << "step: " << step << std::endl;

        unsigned int count = 0;
        unsigned int iPose = 0;

        while (count < numPoses - 1)
        {
            for (unsigned int i = 1; i < trajectoryData[iPose].size(); i++)
            {
                // discreteTrajectory[(i - 1) + 7 * count] = trajectoryData[iPose][i];
                discreteTrajectory.push_back(trajectoryData[iPose][i]);
                // desiredTrajectory.push_back(trajectoryData[iPose][i]);
            }
            count++;
            iPose += step;
        }

        for (unsigned int i = 1; i < trajectoryData[0].size(); i++)
        {
            // discreteTrajectory[(i - 1) + 7 * count] = trajectoryData.back()[i];
            discreteTrajectory.push_back(trajectoryData.back()[i]);
        }
    }

    void getTrajectoryPoses(std::vector<std::array<double, NJoints>> &trajectoryData, const float &distBtwPoses, std::vector<double> &discreteTrajectory, unsigned int &sizeDiscreteTrajectory)
    {
        std::reverse(trajectoryData.begin(), trajectoryData.end());
        float dist = std::numeric_limits<float>::max();
        sizeDiscreteTrajectory = 0;
        for (unsigned int i = 0; i < trajectoryData.size(); i++)
        {
            if (i == 0 || i == (trajectoryData.size() - 1))
            {
                for (unsigned int j = 1; j < trajectoryData[i].size(); j++)
                {
                    if (j == 3)
                    {
                        discreteTrajectory.push_back(trajectoryData[i][j] - 0.894);
                        printf("%f ", trajectoryData[i][j] - 0.894);
                    }
                    else
                    {
                        discreteTrajectory.push_back(trajectoryData[i][j]);
                        printf("%f ", trajectoryData[i][j]);
                    }
                }
                printf("\n");
                sizeDiscreteTrajectory++;
            }
            else
            {
                dist = sqrt(pow((trajectoryData[i][1] - discreteTrajectory[discreteTrajectory.size() - 7]), 2) + pow((trajectoryData[i][2] - discreteTrajectory[discreteTrajectory.size() - 6]), 2) + pow(((trajectoryData[i][3] - 0.894) - discreteTrajectory[discreteTrajectory.size() - 5]), 2));
                if (dist >= distBtwPoses)
                {
                    for (unsigned int j = 1; j < trajectoryData[i].size(); j++)
                    {
                        if (j == 3)
                        {
                            discreteTrajectory.push_back(trajectoryData[i][j] - 0.894);
                            printf("%f ", trajectoryData[i][j] - 0.894);
                        }
                        else
                        {
                            discreteTrajectory.push_back(trajectoryData[i][j]);
                            printf("%f ", trajectoryData[i][j]);
                        }
                    }
                    printf("\n");
                    sizeDiscreteTrajectory++;
                }
            }
        }

    }

    double grooveLoss(double fValue, int n, double s, double c, double r)
    {
        return pow(-1, n) * exp(-pow((fValue - s), 2) / (2 * c * c)) + r * pow((fValue - s), 4);
    }
    double positionGoalMatchingObjectiveTerm(const KDL::Frame &currentPose, const double (&positionGoal)[3])
    {

        double sum = sqrt((currentPose.p.x() - positionGoal[0]) * (currentPose.p.x() - positionGoal[0]) +
                          (currentPose.p.y() - positionGoal[1]) * (currentPose.p.y() - positionGoal[1]) +
                          (currentPose.p.z() - positionGoal[2]) * (currentPose.p.z() - positionGoal[2]));
        std::cout<<"x: "<<positionGoal[0]<<" y: "<<positionGoal[1]<<" z: "<<positionGoal[2]<<" dist: "<<sum<<std::endl;

        return sum;
    }

    double orientationGoalMatchingObjectiveTerm(const Eigen::Quaternionf &q, const Eigen::Quaternionf &qg)
    {
        auto disp = q.inverse().angularDistance(qg);
        Eigen::Quaternionf antipodalq(-q.w(), -q.x(), -q.y(), -q.z());

        auto dispAnti = antipodalq.inverse().angularDistance(qg);

        std::cout<<"disp: "<<disp<<" antidisp: "<<dispAnti<<std::endl;
        return disp;
    }

    double jointVelocityObjectiveTerm(const double *x, std::vector<std::array<double, NJoints>> &velocity, double dt, int iPose)
    {
        double sum = 0;
        // std::cout<<"iPose= "<<iPose<<std::endl;

        if (iPose > 0)
        {
            for (unsigned int j = 0; j < NJoints; j++)
            {
                velocity[iPose][j] = (xprev[iPose][j] - xprev[(iPose - 1)][j]);
                sum += velocity[iPose][j] * velocity[iPose][j];
            }
        }

        return sqrt(sum);
    }

    double jointAccelerationObjectiveTerm(const std::vector<std::array<double, NJoints>> &velocity, std::vector<std::array<double, NJoints>> &acceleration, double dt, int iPose)
    {
        double sum = 0;
        if (iPose > 1)
        {
            for (unsigned int j = 0; j < NJoints; j++)
            {
                acceleration[iPose][j] = (velocity[iPose][j] - velocity[iPose - 1][j]);
                sum += acceleration[iPose][j] * acceleration[iPose][j];
            }
        }
        else if (iPose == 1)
        {
            for (unsigned int j = 0; j < NJoints; j++)
            {
                acceleration[iPose][j] = (velocity[iPose][j] - velocity[iPose - 1][j]);
                sum += acceleration[iPose][j] * acceleration[iPose][j];
            }
        }
        return sqrt(sum);
    }

    double jointJerkObjectiveTerm(const std::vector<std::array<double, NJoints>> &acceleration, std::vector<std::array<double, NJoints>> &jerk, double dt, int iPose)
    {
        double sum = 0;
        if (iPose > 0)
        {
            for (unsigned int j = 0; j < NJoints; j++)
            {
                // double v3 = xprev[iPose][j] - xprev[iPose-1][j];
                // double v2 = xprev[iPose-1][j] - xprev[iPose-2][j];
                // double v1 = xprev[iPose-2][j] - xprev[iPose-3][j];

                // double a2 = v3 -v2;
                // double a1 = v2 - v1;

                // std::cout<<"a: "<<acceleration[iPose*NJoints+j]<<" aprev: "<<acceleration[(iPose-1)*NJoints+j]<<" jerk: "<<jerk[iPose*NJoints+j]<<std::endl;
                jerk[iPose][j] = acceleration[iPose][j] - acceleration[iPose - 1][j];
                sum += jerk[iPose][j] * jerk[iPose][j];
            }
        }
        return sqrt(sum);
    }

    double selfCollisionObjectiveTerm(KDL::JntArray &q)
    {
        double sum = 0.0;

        KDL::JntArray qcurrent(NJoints);
        for (int i = 0; i < NJoints; i++)
        {
            qcurrent(i) = q(i) * KDL::rad2deg;
        }

        checkSelfCollision->updateCollisionObjectsTransform(q);

        int k = 0;
        float b = 50.0;
        for (int i = 0; i < collisionObjects.size(); i++)
        {
            for (int j = i + 2; j < collisionObjects.size(); j++)
            {
                float c = sqrt(-pow(initialDistances[k], 4) / (2 * (log((1e-15) / b))));
                float d = checkSelfCollision->twoLinksDistance(qcurrent, i, j) * 1000;
                if (d < 0)
                {
                    d = 0;
                }
                // std::cout<<"i: "<<i<<"j: "<<j<<" d: "<<d<<" collide: "<<checkSelfCollision->twoLinksCollide(qcurrent, i, j)<<std::endl;
                // std::cout<<-(d*d*d*d)/(2*c*c)<<" exp: "<<std::exp(-(d*d*d*d)/(2*c*c))<<std::endl;

                sum += b * std::exp((-d * d * d * d) / (2 * c * c));
                k++;
            }
        }
        // std::cout<<"self-collision term: "<<sum<<std::endl;
        return sum;
    }

    double endEffectorVelocityObjectiveTerm(const KDL::Frame &currentPose, const KDL::Frame &prevPose, double dt, int iPose)
    {
        if (iPose > 0)
        {
            return sqrt(pow((currentPose.p.x() - prevPose.p.x()), 2) + pow((currentPose.p.y() - prevPose.p.y()), 2) + pow((currentPose.p.z() - prevPose.p.z()), 2));
        }
        return 0.0;
    }

    double optimizationFunctionJoints(unsigned n, const double *x, double *grad, void *data)
    {
        iteration += 1;
        std::cout << "optimization iter: " << iteration << std::endl;
        auto *voidToVector = reinterpret_cast<std::vector<double> *>(data);
        double sum = 0;
        double dt = 0.1;
        KDL::JntArray q(NJoints); // TODO correct in order to gather the q init from outside and also the size of the joints array

        KDL::Frame prevPose;

        if ((int)(n / NJoints) > 1)
        {
            for (unsigned int i = 0; i < (int)(n / NJoints); i++)
            {
                for (unsigned int j = 0; j < NJoints; j++)
                {
                    q(j) = x[i * NJoints + j];
                    xprev[i][j] = x[i * NJoints + j];
                }
                selfCollisionObjectiveTerm(q);

                KDL::Frame currentPose;

                int foundfk = (*fksolver).JntToCart(q, currentPose);
                // Here we will need to change the index from when we gathered the desired positions
                double auxq[4];
                currentPose.M.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

                // std::cout<<"End effector quat: "<<auxq[0]<<" "<<auxq[1]<<" "<<auxq[2]<<" "<<auxq[3]<<std::endl;
                KDL::Rotation rotKdl1 = KDL::Rotation::Quaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

                Eigen::Quaternionf qc(auxq[3], auxq[0], auxq[1], auxq[2]);

                double norm = sqrt((*voidToVector)[(indexDiscrete + i) * 7 + 3] * (*voidToVector)[(indexDiscrete + i) * 7 + 3] + (*voidToVector)[(indexDiscrete + i) * 7 + 4] * (*voidToVector)[(indexDiscrete + i) * 7 + 4] + (*voidToVector)[(indexDiscrete + i) * 7 + 5] * (*voidToVector)[(indexDiscrete + i) * 7 + 5] + (*voidToVector)[(indexDiscrete + i) * 7 + 6] * (*voidToVector)[(indexDiscrete + i) * 7 + 6]);
                KDL::Rotation rotKdl = KDL::Rotation::Quaternion((*voidToVector)[(indexDiscrete + i) * 7 + 3] / norm, (*voidToVector)[(indexDiscrete + i) * 7 + 4] / norm, (*voidToVector)[(indexDiscrete + i) * 7 + 5] / norm, (*voidToVector)[(indexDiscrete + i) * 7 + 6] / norm);
                rotKdl.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

                std::cout<<"qgoal: "<<auxq[0]<<" "<<auxq[1]<<" "<<auxq[2]<<" "<<auxq[3]<<std::endl;
                rotKdl = rotKdl * KDL::Rotation::RotX(-KDL::PI/2.0);
                rotKdl = rotKdl * KDL::Rotation::RotZ(-KDL::PI / 2.0 - KDL::PI / 4.0);
                rotKdl = rotKdl*KDL::Rotation::RotY(KDL::PI / 5);

                rotKdl.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

                std::cout<<"qgoal: "<<auxq[0]<<" "<<auxq[1]<<" "<<auxq[2]<<" "<<auxq[3]<<std::endl;

                Eigen::Quaternionf qg(auxq[3], auxq[0], auxq[1], auxq[2]);

                double positionGoal[3] = {(*voidToVector)[(indexDiscrete + i) * 7], (*voidToVector)[(indexDiscrete + i) * 7 + 1], (*voidToVector)[(indexDiscrete + i) * 7 + 2]};
                        std::cout<<"x: "<<positionGoal[0]<<" y: "<<positionGoal[1]<<" z: "<<positionGoal[2]<<" dist: "<<sum<<std::endl;

                sum += wpos * grooveLoss(positionGoalMatchingObjectiveTerm(currentPose, positionGoal), 1, 0, 0.1, 10.0)+\
                       wjv * grooveLoss(jointVelocityObjectiveTerm(x, velocity, dt, i), 1, 0, 0.1,10.0)+\
                       wori * grooveLoss(orientationGoalMatchingObjectiveTerm(qc, qg), 1, 0, 0.1, 10.0)+\
                       wv * grooveLoss(endEffectorVelocityObjectiveTerm(currentPose, prevPose, dt, i), 1, 0, 0.1, 10.0)+\
                       wja * grooveLoss(jointAccelerationObjectiveTerm(velocity, acceleration, dt, i), 1, 0, 0.1, 10.0)+\
                       wjj * grooveLoss(jointJerkObjectiveTerm(acceleration, jerk, dt, i), 1, 0, 0.1, 10.0)+\
                       wsc * grooveLoss(selfCollisionObjectiveTerm(q), 1, 0.0, 0.1, 10.0);

                prevPose = currentPose;
            }
        }
        else
        {   std::cout<<"Se supone que estamos aqui"<<std::endl;
            unsigned int i = 3;
            for (unsigned int j = 0; j < NJoints; j++)
            {
                q(j) = x[j];
                xprev[i][j] = x[j];
            }
            selfCollisionObjectiveTerm(q);

            KDL::Frame currentPose;

            int foundfk = (*fksolver).JntToCart(q, currentPose);
            // Here we will need to change the index from when we gathered the desired positions
            double auxq[4];
            currentPose.M.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

            // std::cout<<"End effector quat: "<<auxq[0]<<" "<<auxq[1]<<" "<<auxq[2]<<" "<<auxq[3]<<std::endl;
            KDL::Rotation rotKdl1 = KDL::Rotation::Quaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

            Eigen::Quaternionf qc(auxq[3], auxq[0], auxq[1], auxq[2]);

            double norm = sqrt((*voidToVector)[(indexDiscrete + i) * 7 + 3] * (*voidToVector)[(indexDiscrete + i) * 7 + 3] + (*voidToVector)[(indexDiscrete + i) * 7 + 4] * (*voidToVector)[(indexDiscrete + i) * 7 + 4] + (*voidToVector)[(indexDiscrete + i) * 7 + 5] * (*voidToVector)[(indexDiscrete + i) * 7 + 5] + (*voidToVector)[(indexDiscrete + i) * 7 + 6] * (*voidToVector)[(indexDiscrete + i) * 7 + 6]);
            KDL::Rotation rotKdl = KDL::Rotation::Quaternion((*voidToVector)[(indexDiscrete + i) * 7 + 3] / norm, (*voidToVector)[(indexDiscrete + i) * 7 + 4] / norm, (*voidToVector)[(indexDiscrete + i) * 7 + 5] / norm, (*voidToVector)[(indexDiscrete + i) * 7 + 6] / norm);
            rotKdl = rotKdl * KDL::Rotation::RotX(-KDL::PI/2.0);
            rotKdl = rotKdl * KDL::Rotation::RotZ(-KDL::PI / 2.0 - KDL::PI / 4.0);
            rotKdl = rotKdl*KDL::Rotation::RotY(KDL::PI / 5);

            rotKdl.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

            Eigen::Quaternionf qg(auxq[3], auxq[0], auxq[1], auxq[2]);

            double positionGoal[3] = {(*voidToVector)[(indexDiscrete + i) * 7], (*voidToVector)[(indexDiscrete + i) * 7 + 1], (*voidToVector)[(indexDiscrete + i) * 7 + 2]};
            
            sum += wpos * grooveLoss(positionGoalMatchingObjectiveTerm(currentPose, positionGoal), 1, 0, 0.1, 10.0)+\
                   wjv * grooveLoss(jointVelocityObjectiveTerm(x, velocity, dt, i), 1, 0, 0.1, 10.0)+\
                   wori * grooveLoss(orientationGoalMatchingObjectiveTerm(qc, qg), 1, 0, 0.1, 10.0)+\
                   wv * grooveLoss(endEffectorVelocityObjectiveTerm(currentPose, prevPose, dt, i), 1, 0, 0.1, 10.0)+\
                   wja * grooveLoss(jointAccelerationObjectiveTerm(velocity, acceleration, dt, i), 1, 0, 0.1, 10.0)+\
                   wjj * grooveLoss(jointJerkObjectiveTerm(acceleration, jerk, dt, i), 1, 0, 0.1, 10.0)+\
                   wsc * grooveLoss(selfCollisionObjectiveTerm(q), 1, 0.0, 0.1, 10.0);

            prevPose = currentPose;
        }
        std::cout << "opt: "<<sum << std::endl;
        return sum;
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

    void writeQCsv(std::string filename, const std::vector<std::array<double, NJoints>> &qTraj)
    {
        std::ofstream myFile(filename);
        for (unsigned int i = 0; i < qTraj.size(); i++)
        {
            myFile << i << " " << std::setprecision(10) << qTraj[i][0] << " " << qTraj[i][1] << " " << qTraj[i][2] << " " << qTraj[i][3] << " " << qTraj[i][4]
                   << " " << qTraj[i][5] << " " << qTraj[i][6] << " " << qTraj[i][7] << "\n";
        }
        myFile.close();
    }

    void  writeDataCsv(std::string filename, const std::vector<double> orientatationdist){
        std::ofstream myFile(filename);
        for (unsigned int i = 0; i < orientatationdist.size(); i++)
        {
            myFile << i <<" " << std::setprecision(10) <<orientatationdist[i] <<"\n";
        }
        myFile.close();
    }


    void writePoseCsv(std::string filename, const std::vector<std::array<double, 7>> &poses){
        std::ofstream myFile(filename);
        for (unsigned int i = 0; i < poses.size(); i++)
        {
            myFile << i << " " << std::setprecision(10) << poses[i][0] << " " << poses[i][1] << " " << poses[i][2] << " " << poses[i][3] << " " << poses[i][4]
                   << " " << poses[i][5] << " " << poses[i][6] << "\n";
        }
        myFile.close();
    }

    void writeResults(std::string filename, const std::vector<int> results, int solverMaxIter, double boundsDiscretePoses, double maxTime)
    {
        std::ofstream myFile(filename);
        myFile << "Number of poses: " << results.size() << "\n";
        myFile << "Ik solver max iter: " << solverMaxIter << "\n";
        myFile << "Bounds range: " << boundsDiscretePoses << "\n";
        myFile << "Max time: " << maxTime << "\n";
        myFile << "Results ik: "
               << "\n";
        for (unsigned int i = 0; i < results.size(); i++)
        {
            myFile << i << "," << results[i] << "\n";
        }
        myFile.close();
    }

    void jointVelocityLimit(unsigned m, double *result, unsigned n, const double *x, double *grad, void *data)
    {
        std::cout << "constraint" << std::endl;
        for (unsigned int i = 0; i < n; i++)
        {
            if (i < NJoints)
                result[i] = -5 * KDL::deg2rad;
            else
            {
                result[i] = fabs(x[i - NJoints] - x[i]) / 0.5 - 5 * KDL::deg2rad;
                std::cout << "limit: " << fabs(x[i - NJoints] - x[i]) / 0.5 * KDL::rad2deg;
            }
        }
    }

}

int main()
{
    KDL::JntArray qmin(NJoints), qmax(NJoints);
    makeQLimitsTeoTrunkAndRightArmKinematics(qmin, qmax);
    createSelfCollisionObjects();
    KDL::Chain chain = makeTeoTrunkAndRightArmKinematicsFromDH();

    checkSelfCollision = new sharon::CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects, tableCollision);

    std::string csvFile = "/home/elisabeth/repos/teo-sharon/programs/GenerateManipulationTrajectories/trajectories/graspcup1/test-right-arm-motion-smooth2-reaching.csv";
    std::vector<std::array<double, NJoints>> desiredTrajectoryData = sharon::getTrajectoryFromCsvFile(csvFile);
    sharon::printTrajectoryData(desiredTrajectoryData);

    std::vector<double> desiredDiscretePoses;

    sharon::getTrajectoryPoses(desiredTrajectoryData, (float)0.0005, desiredDiscretePoses, numPoses);

    std::cout << "numPoses: " << numPoses << std::endl;
    // Start optimizing first 4 poses
    int nFirstOptimization = 4;

    xprev.reserve(nFirstOptimization);
    velocity.reserve(nFirstOptimization);
    acceleration.reserve(nFirstOptimization);
    jerk.reserve(nFirstOptimization);

    // joint velocity, acceleration, jerk are set to zero at t=0 for all the joints
    for (unsigned int j = 0; j < NJoints; j++)
    {
        velocity[0][j] = 0.0;
        acceleration[0][j] = 0.0;
        acceleration[1][j] = 0.0;
        jerk[0][j] = 0.0;
        jerk[1][j] = 0.0;
        jerk[2][j] = 0.0;
    }

    double x[nFirstOptimization * NJoints];
    std::fill_n(x, nFirstOptimization * NJoints, 0.05);

    void *parameters = &desiredDiscretePoses;

    fksolver = new KDL::ChainFkSolverPos_recursive(chain);

    nlopt_opt opt;
    opt = nlopt_create(NLOPT_LN_BOBYQA, nFirstOptimization * NJoints);
    indexDiscrete = 0;
    nlopt_set_min_objective(opt, sharon::optimizationFunctionJoints, parameters);
    double lb[nFirstOptimization * NJoints];
    double ub[nFirstOptimization * NJoints];

    for (unsigned int i = 0; i < nFirstOptimization; i++)
    {
        for (unsigned int j = 0; j < NJoints; j++)
        {
            lb[i * NJoints + j] = qmin(j) * KDL::deg2rad;
            ub[i * NJoints + j] = qmax(j) * KDL::deg2rad;
        }
    }

    std::cout << x[0];
    nlopt_set_lower_bounds(opt, lb);
    nlopt_set_upper_bounds(opt, ub);
    double tol[nFirstOptimization * NJoints];
    for (unsigned int i = 0; i < nFirstOptimization * NJoints; i++)
        tol[i] = 1e-6;

    nlopt_add_inequality_mconstraint(opt, nFirstOptimization * NJoints, sharon::jointVelocityLimit, NULL, tol);

    double minf=MAXFLOAT;
    std::vector<std::array<double, NJoints>> qTraj;
    std::array<double, NJoints> qArray;
    std::vector<int> results;

    double maxTime = 100000;
    // nlopt_set_maxtime(opt, maxTime);
    nlopt_set_ftol_abs(opt, 1e-10);
    // nlopt_set_ftol_rel(opt, 0.000001);
    nlopt_result result = nlopt_optimize(opt, x, &minf);
    std::cout << "found minimum at f(x)=" << minf << std::endl;

    for (unsigned int i = 0; i < nFirstOptimization; i++)
    {
        for (unsigned int joint = 0; joint < NJoints; joint++)
            qArray[joint] = x[i * NJoints + joint] * KDL::rad2deg;
        qTraj.push_back(qArray);
    }

    // This is just for having also the path in cartersian space
    for (unsigned int i = 0; i < nFirstOptimization; i++){

        KDL::JntArray q(NJoints); // TODO correct in order to gather the q init from outside and also the size of the joints array
        for (unsigned int j = 0; j < NJoints; j++)
        {
            q(j) = x[i * NJoints + j];
            // q(j) = 0.0;
        }

        KDL::Frame currentPose;
        int foundfk = (*fksolver).JntToCart(q, currentPose);
        double quat[4];
        currentPose.M.GetQuaternion(quat[0], quat[1], quat[2], quat[3]);
        std::array<double, 7> pose;
        pose[0] = currentPose.p.x();
        pose[1] = currentPose.p.y();
        pose[2] = currentPose.p.z()+0.894;
        pose[3] = quat[0];
        pose[4] = quat[1];
        pose[5] = quat[2];
        pose[6] = quat[3];
        std::cout<<"quat: "<<quat[0]<<" "<<quat[1]<<" "<<quat[2]<<" "<<quat[3]<<std::endl;

        poses.push_back(pose);

        double norm = sqrt((desiredDiscretePoses)[(indexDiscrete + i) * 7 + 3] * (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 3] + (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 4] * (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 4] + (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 5] * (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 5] + (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 6] * (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 6]);
        KDL::Rotation rotKdl = KDL::Rotation::Quaternion((desiredDiscretePoses)[(indexDiscrete + i) * 7 + 3] / norm, (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 4] / norm, (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 5] / norm, (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 6] / norm);
        rotKdl = rotKdl * KDL::Rotation::RotX(-KDL::PI/2.0);
        rotKdl = rotKdl * KDL::Rotation::RotZ(-KDL::PI / 2.0 - KDL::PI / 4.0);
        rotKdl = rotKdl*KDL::Rotation::RotY(KDL::PI / 5.0);
        double auxq[4];
        rotKdl.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

        Eigen::Quaternionf qg(auxq[3], auxq[0], auxq[1], auxq[2]);
        Eigen::Quaternionf qcurrent(quat[3], quat[0], quat[1], quat[2]);

        orientatationdist.push_back(sharon::orientationGoalMatchingObjectiveTerm(qcurrent, qg));

    }



    for (unsigned int i = 0; i < nFirstOptimization; i++)
    {
        std::cout << "xprev: ";
        for (unsigned int j = 0; j < NJoints; j++)
        {
            std::cout << xprev[i][j] << " ";
        }
        std::cout << std::endl;
        std::cout << "velocity: ";
        for (unsigned int j = 0; j < NJoints; j++)
        {
            std::cout << velocity[i][j] << " ";
        }
        std::cout << std::endl;
        std::cout << "acceleration: ";
        for (unsigned int j = 0; j < NJoints; j++)
        {
            std::cout << acceleration[i][j] << " ";
        }
        std::cout << std::endl;
        std::cout << "jerk: ";
        for (unsigned int j = 0; j < NJoints; j++)
        {
            std::cout << jerk[i][j] << " ";
        }
        std::cout << std::endl;
    }



    // Now we will call the optimizer one pose at a time, taking into account the previous optimizations
    double x_one_by_one[NJoints];

    double lb_obo[NJoints];
    double ub_obo[NJoints];

    for (unsigned int j = 0; j < NJoints; j++)
    {
        lb_obo[j] = qmin(j) * KDL::deg2rad;
        ub_obo[j] = qmax(j) * KDL::deg2rad;
    }

    for (indexDiscrete = 1; indexDiscrete < numPoses-3; indexDiscrete++)
    {
        double minf_obo=MAXFLOAT;
        std::cout<<"indexDiscrete: "<< indexDiscrete<<std::endl;
        std::cout << "numPoses: " << numPoses << std::endl;
    

        // Move to the left one position the elements of the xprev, velocity, acceleration and jerk
        for (unsigned int i = 0; i < 3; i++)
        {
            for (unsigned int j = 0; j < NJoints; j++)
            {
                xprev[i][j] = xprev[i + 1][j];
                velocity[i][j] = velocity[i + 1][j];
                // std::cout<<"vel: "<<velocity[i][j]<<" ";
                acceleration[i][j] = acceleration[i + 1][j];
                jerk[i][j] = jerk[i + 1][j];
            }
            // std::cout<<std::endl;

        }

        for (unsigned int j = 0; j < NJoints; j++)
        {
            x_one_by_one[j] = xprev[3][j];
            std::cout<<x_one_by_one[j]<<" lb: "<<lb_obo[j]<<" ub: "<<ub_obo[j]<<std::endl;
            if(x_one_by_one[j] <= lb_obo[j])
                x_one_by_one[j] = lb_obo[j]+0.0001;
            else if(x_one_by_one[j] >= ub_obo[j])
                x_one_by_one[j] = ub_obo[j]-0.0001;
        }
        std::cout<<std::endl;

        nlopt_opt opt_obo;
        opt_obo = nlopt_create(NLOPT_LN_BOBYQA, NJoints);
        nlopt_set_min_objective(opt_obo, sharon::optimizationFunctionJoints, parameters);
        nlopt_set_lower_bounds(opt_obo, lb_obo);
        nlopt_set_upper_bounds(opt_obo, ub_obo);
        double tol_obo[NJoints];
        for (unsigned int i = 0; i < NJoints; i++)
            tol_obo[i] = 1e-4;

        nlopt_add_inequality_mconstraint(opt_obo, NJoints, sharon::jointVelocityLimit, NULL, tol_obo);

        try
        {
            double maxTime = 100000;
            // nlopt_set_maxtime(opt, maxTime);
            nlopt_set_ftol_abs(opt_obo, 1e-10);
            // nlopt_set_ftol_rel(opt, 0.000001);
            nlopt_result result = nlopt_optimize(opt_obo, x_one_by_one, &minf_obo);
            std::cout << "found minimum at f(x)=" << minf_obo << std::endl;

            for (unsigned int joint = 0; joint < NJoints; joint++)
                qArray[joint] = x_one_by_one[joint] * KDL::rad2deg;
            qTraj.push_back(qArray);

            // ------ Poses ---------------- //
            KDL::JntArray q(NJoints); // TODO correct in order to gather the q init from outside and also the size of the joints array
            for (unsigned int j = 0; j < NJoints; j++)
            {
                q(j) = x_one_by_one[j];
            }

            KDL::Frame currentPose;
            int foundfk = (*fksolver).JntToCart(q, currentPose);
            double quat[4];
            currentPose.M.GetQuaternion(quat[0], quat[1], quat[2], quat[3]);
            std::array<double, 7> pose;
            pose[0] = currentPose.p.x();
            pose[1] = currentPose.p.y();
            pose[2] = currentPose.p.z()+0.894;
            pose[3] = quat[0];
            pose[4] = quat[1];
            pose[5] = quat[2];
            pose[6] = quat[3];

            double norm = sqrt((desiredDiscretePoses)[(indexDiscrete+3) * 7 + 3] * (desiredDiscretePoses)[(indexDiscrete+3) * 7 + 3] + (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 4] * (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 4] + (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 5] * (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 5] + (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 6] * (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 6]);
            KDL::Rotation rotKdl = KDL::Rotation::Quaternion((desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 3] / norm, (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 4] / norm, (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 5] / norm, (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 6] / norm);
            rotKdl = rotKdl * KDL::Rotation::RotX(-KDL::PI / 2.0);
            rotKdl = rotKdl * KDL::Rotation::RotZ(-KDL::PI / 2.0 - KDL::PI / 4.0);
            rotKdl = rotKdl*KDL::Rotation::RotY(KDL::PI / 5);
            double auxq[4];
            rotKdl.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

            Eigen::Quaternionf qg(auxq[3], auxq[0], auxq[1], auxq[2]);
            Eigen::Quaternionf qcurrent(quat[3], quat[0], quat[1], quat[2]);

            orientatationdist.push_back(sharon::orientationGoalMatchingObjectiveTerm(qcurrent, qg));
            poses.push_back(pose);

        }
        catch (std::exception &e)
        {
            std::cout << "nlopt failed: " << e.what() << std::endl;
            break;
        }
    }
    std::reverse(qTraj.begin(), qTraj.end());
    std::string csvQFileWrite = "/home/elisabeth/repos/teo-sharon/programs/GenerateManipulationTrajectories/trajectories/test/test-right-arm-motion-smooth" + std::to_string(2) + "1-joint.csv";
    sharon::writeQCsv(csvQFileWrite, qTraj);
    std::reverse(poses.begin(), poses.end());
    std::string csvPoseFileWrite = "/home/elisabeth/repos/teo-sharon/programs/GenerateManipulationTrajectories/trajectories/test/test-right-arm-motion-smooth" + std::to_string(2) + "1-poses.csv";
    sharon::writePoseCsv(csvPoseFileWrite, poses);
    std::reverse(orientatationdist.begin(), orientatationdist.end());
    std::string csvDataFileWrite = "/home/elisabeth/repos/teo-sharon/programs/GenerateManipulationTrajectories/trajectories/test/test-right-arm-motion-smooth" + std::to_string(2) + "1-data.csv";
    sharon::writeDataCsv(csvDataFileWrite, orientatationdist);
    return 0;
}