#include "MapHumanHandTrajectoryToRobotJointSpace.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <nlopt.hpp>
#include <math.h>
#include <iomanip> // std::setprecision
#include <limits>  // std::numeric_limits
#define DEFAULT_WORI 2.0
#define MAX_POS_DISTANCE 1.5
#define MIN_POS_DISTANCE 0.0
#define MAX_ORI_DISTANCE M_PI
#define MIN_ORI_DISTANCE 0.0
#define MAX_JVEL 40.0 * KDL::deg2rad
#define MIN_JVEL 0.0
#define MAX_JACC 10.0 * KDL::deg2rad
#define MAX_JERK 50.0 * KDL::deg2rad

#define TIME_STEP 1.0
float aux = 0.001;

KDL::ChainIkSolverPos_NR_JL *iksolver;
KDL::ChainFkSolverPos_recursive *fksolver;
KDL::ChainIkSolverVel_pinv *iksolverv;

std::vector<double> qTrajectory;
// std::vector<fcl::CollisionObjectf> collisionObjects;
// std::vector<std::array<float, 3>> offsetCollisionObjects;
// typedef std::shared_ptr<fcl::CollisionGeometryf> CollisionGeometryPtr_t;
// sharon::CheckSelfCollision *checkSelfCollision;
// std::vector<fcl::CollisionObjectf> tableCollision;

unsigned int numPoses = 0;
unsigned int indexDiscrete = 0;
int nDemo = 2;
unsigned int iteration = 0;
int nColsData = 29;

// std::vector<std::array<double, 7>> desiredDiscreteWristPoses;
// std::vector<std::array<double, 7>> desiredDiscreteElbowPoses;
// std::vector<std::array<double, 7>> desiredDiscreteShoulderPoses;

vector<array<double, 7>> adjHandTrajectoryData;
vector<array<double, 7>> adjWristTrajectoryData;
vector<array<double, 7>> adjElbowTrajectoryData;
vector<array<double, 7>> adjShoulderTrajectoryData;
vector<array<double, 7>> adjNeckTrajectoryData;
vector<array<double, 7>> adjHipTrajectoryData;

float wpos = 20.0; // weight for end-effector position matching objective term
float wori = 10.0; // weight for end-effector orientation matching objective term
float wjv = 5.0;   // weight for joint velocity objective term
float wja = 5.0;   // weight for joint acceleration objective term
float wjj = 5.0;   // weight for joint jerk objective term
float wv = 0.0;    // weight for end-effector position velocity objective term
float wsc = 0.0;   // weight for self-collision avoidance objective term
float wes = 10.0;
float wew = 10.0;
KDL::Frame prevPose;

std::vector<std::array<double, NJoints>> xprev;
std::vector<std::array<double, NJoints>> velocity;
std::vector<std::array<double, NJoints>> acceleration;
std::vector<std::array<double, NJoints>> jerk;
std::vector<std::array<double, 7>> poses;
std::vector<double> orientatationdist;
std::vector<double> positiondist;
std::vector<double> angleShoudlerElbow;
std::vector<double> angleElbowWrist;
std::vector<std::array<double, NJoints>> qx;

float initialDistances[15] = {0.118561 * 1, 0.0804199 * 1, 0.0740597 * 1, 0.140743 * 1, 0.0568082 * 1, 0.0635505 * 1, 0.146518 * 1, 0.151592 * 1, 0.271188 * 1, 0.106259 * 1};
float q_endEffector_HumanHand[4] = {-0.142387024727039, 0.778327151043027, 0.486859494151674, 0.370000829854479};

double fRand(double fMin, double fMax)
{
    double f = (double)rand() / RAND_MAX;
    return fMin + f * (fMax - fMin);
}

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
    qmin.resize(8);
    qmax.resize(8);
    qmin(0) = -31.0 * KDL::deg2rad;
    qmin(1) = -10.1 * KDL::deg2rad;
    qmin(2) = -98.1 * KDL::deg2rad;
    qmin(3) = -75.5 * KDL::deg2rad;
    qmin(4) = -80.1 * KDL::deg2rad;
    qmin(5) = -99.6 * KDL::deg2rad;
    qmin(6) = -80.4 * KDL::deg2rad;
    qmin(7) = -115.1 * KDL::deg2rad;
    qmax(0) = 31.0 * KDL::deg2rad;
    qmax(1) = 25.5 * KDL::deg2rad;
    qmax(2) = 106.0 * KDL::deg2rad;
    qmax(3) = 22.4 * KDL::deg2rad;
    qmax(4) = 57.0 * KDL::deg2rad;
    qmax(5) = 98.4 * KDL::deg2rad;
    qmax(6) = 99.6 * KDL::deg2rad;
    qmax(7) = 44.7 * KDL::deg2rad;
}

// void createSelfCollisionObjects()
// {

//     CollisionGeometryPtr_t teoRootTrunk{new fcl::Boxf{0.35, 0.4, 0.75}};
//     fcl::Transform3f tfTest;
//     fcl::CollisionObjectf collisionObject1{teoRootTrunk, tfTest};

//     CollisionGeometryPtr_t teoTrunk{new fcl::Boxf{0.3, 0.3, 0.46}};
//     fcl::CollisionObjectf collisionObject2{teoTrunk, tfTest};

//     CollisionGeometryPtr_t teoAxialShoulder{new fcl::Boxf{0.10, 0.10, 0.32901}}; //{new fcl::Box{0.15, 0.15, 0.32901}};
//     fcl::CollisionObjectf collisionObject3{teoAxialShoulder, tfTest};

//     CollisionGeometryPtr_t teoElbow{new fcl::Boxf{0.10, 0.10, 0.22}};
//     fcl::CollisionObjectf collisionObject4{teoElbow, tfTest};

//     CollisionGeometryPtr_t teoWrist{new fcl::Boxf{0.2, 0.2, 0.2}};
//     fcl::CollisionObjectf collisionObject5{teoWrist, tfTest};

//     CollisionGeometryPtr_t teoHand{new fcl::Boxf{0.05, 0.05, 0.05}};
//     fcl::CollisionObjectf collisionObject6{teoHand, tfTest};

//     CollisionGeometryPtr_t table{new fcl::Boxf{0.0, 0.0, 0.0}};
//     fcl::CollisionObjectf collisionObjectTable{table, tfTest};
//     fcl::Quaternionf rotation(1.0, 0.0, 0.0, 0.0);
//     // fcl::Vector3f translation(1.65, 0.0, -0.43);
//     fcl::Vector3f translation(2.0, 2.0, 0.0);
//     collisionObjectTable.setTransform(rotation, translation);
//     tableCollision.clear();
//     tableCollision.push_back(collisionObjectTable);

//     int nOfCollisionObjects = 6;
//     collisionObjects.clear();
//     collisionObjects.reserve(nOfCollisionObjects);
//     collisionObjects.emplace_back(collisionObject1);
//     collisionObjects.emplace_back(collisionObject2);
//     collisionObjects.emplace_back(collisionObject3);
//     collisionObjects.emplace_back(collisionObject4);
//     collisionObjects.emplace_back(collisionObject5);
//     collisionObjects.emplace_back(collisionObject6);

//     offsetCollisionObjects.resize(nOfCollisionObjects);
//     std::array<float, 3> offsetObject = {0, 0, 0};
//     offsetCollisionObjects[0][2] = -0.2;
//     offsetCollisionObjects[1][1] = 0.0;
//     offsetCollisionObjects[1][2] = +0.1734;
//     offsetCollisionObjects[4][1] = 0.055;
// }

namespace sharon
{

    double grooveLoss(double fValue, int n, double s, double c, double r)
    {
        // return pow(-1, n) * exp(-pow((fValue - s), 2) / (2 * c * c)) + r * pow((fValue - s), 4);
        return fValue;
    }

    double positionGoalMatchingObjectiveTerm(const KDL::Frame &currentPose, const double (&positionGoal)[3])
    {

        double sum = sqrt((currentPose.p.x() - positionGoal[0]) * (currentPose.p.x() - positionGoal[0]) +
                          (currentPose.p.y() - positionGoal[1]) * (currentPose.p.y() - positionGoal[1]) +
                          (currentPose.p.z() - positionGoal[2]) * (currentPose.p.z() - positionGoal[2]));
        // std::cout << "x: " << positionGoal[0] << " y: " << positionGoal[1] << " z: " << positionGoal[2] << " dist: " << sum << std::endl;
        // std::cout << "cx: " << currentPose.p.x() << " cy: " << currentPose.p.y() << " cz: " << currentPose.p.z() << " dist: " << sum << std::endl;
        sum = (sum - MIN_POS_DISTANCE) / (MAX_POS_DISTANCE - MIN_POS_DISTANCE);
        return sum;
    }

    double orientationGoalMatchingObjectiveTerm(const Eigen::Quaternionf &q, const Eigen::Quaternionf &qg)
    {
        double inner_product = q.x() * qg.x() + q.y() * qg.y() + q.z() * qg.z() + q.w() * qg.w();
        double angle = acos(2 * inner_product * inner_product - 1);
        auto disp = q.inverse().angularDistance(qg);
        Eigen::Quaternionf antipodalq(-q.w(), -q.x(), -q.y(), -q.z());

        auto dispAnti = antipodalq.inverse().angularDistance(qg);

        // std::cout << "disp: " << disp << " antidisp: " << dispAnti << std::endl;

        disp = (abs(disp) - MIN_ORI_DISTANCE) / (MAX_ORI_DISTANCE - MIN_ORI_DISTANCE);
        return (abs(angle) - MIN_ORI_DISTANCE) / (MAX_ORI_DISTANCE - MIN_ORI_DISTANCE);
    }

    double jointVelocityObjectiveTerm(const double *x, std::vector<std::array<double, NJoints>> &velocity, double dt, int iPose)
    {
        double sum = 0;
        // std::cout<<"iPose= "<<iPose<<std::endl;
        if (indexDiscrete == 0)
        {
            for (unsigned int j = 0; j < NJoints; j++)
                velocity[iPose][j] = 0.0;
        }
        if (indexDiscrete > 0)
        {
            for (unsigned int j = 0; j < NJoints; j++)
            {
                velocity[iPose][j] = (x[j] - xprev[(iPose - 1)][j]) / TIME_STEP;
                // std::cout<<"velocity_j: "<<velocity[iPose][j];
                sum += (abs(velocity[iPose][j]) - MIN_JVEL) / (MAX_JVEL - MIN_JVEL);
                // sum += velocity[iPose][j]*velocity[iPose][j];
            }
        }

        return sum / NJoints;
    }

    double jointAccelerationObjectiveTerm(const double *x, std::vector<std::array<double, NJoints>> &acceleration, double dt, int iPose)
    {
        double sum = 0;
        if (indexDiscrete == 0)
        {
            for (unsigned int j = 0; j < NJoints; j++)
            {
                acceleration[iPose][j] = 0.0;
            }
        }
        if (indexDiscrete == 1)
        {
            for (unsigned int j = 0; j < NJoints; j++)
            {
                double v0 = 0;
                double v1 = x[j] - xprev[2][j];
                acceleration[iPose][j] = v1 - v0;
                sum += abs(acceleration[iPose][j]) / (MAX_JACC);
            }
        }
        if (indexDiscrete > 1)
        {
            for (unsigned int j = 0; j < NJoints; j++)
            {
                double v0 = xprev[2][j] - xprev[1][j];
                double v1 = x[j] - xprev[2][j];
                acceleration[iPose][j] = v1 - v0;
                sum += abs(acceleration[iPose][j]) / (MAX_JACC);
            }
        }

        return sum / NJoints;
    }

    double jointJerkObjectiveTerm(const double *x, std::vector<std::array<double, NJoints>> &jerk, double dt, int iPose)
    {
        double sum = 0;
        if (indexDiscrete == 0)
        {
            for (unsigned int j = 0; j < NJoints; j++)
            {
                jerk[iPose][j] = 0;
            }
        }
        if (indexDiscrete == 1)
        {
            for (unsigned int j = 0; j < NJoints; j++)
            {
                double v0 = 0;
                double v1 = x[j] - xprev[2][j];
                double a0 = 0;
                double a1 = v1 - v0;
                jerk[iPose][j] = a1 - a0;
                // sum += jerk[iPose][j] * jerk[iPose][j];
                sum += abs(jerk[iPose][j]) / (MAX_JERK);
            }
        }
        if (indexDiscrete >= 2)
        {
            for (unsigned int j = 0; j < NJoints; j++)
            {
                double v1 = xprev[1][j] - xprev[0][j];
                double v2 = xprev[2][j] - xprev[1][j];
                double v3 = x[j] - xprev[2][j];
                double a2 = v2 - v1;
                double a3 = v3 - v2;
                jerk[iPose][j] = a3 - a2;
                // sum += jerk[iPose][j] * jerk[iPose][j];
                sum += abs(jerk[iPose][j]) / (MAX_JERK);
            }
        }
        return sum / NJoints;
    }

    // double selfCollisionObjectiveTerm(KDL::JntArray &q)
    // {
    //     double sum = 0.0;

    //     KDL::JntArray qcurrent(NJoints);
    //     for (int i = 0; i < NJoints; i++)
    //     {
    //         qcurrent(i) = q(i) * KDL::rad2deg;
    //     }

    //     checkSelfCollision->updateCollisionObjectsTransform(q);

    //     int k = 0;
    //     float b = 50.0;
    //     for (int i = 0; i < collisionObjects.size(); i++)
    //     {
    //         for (int j = i + 2; j < collisionObjects.size(); j++)
    //         {
    //             float c = sqrt(-pow(initialDistances[k], 4) / (2 * (log((1e-15) / b))));
    //             float d = checkSelfCollision->twoLinksDistance(qcurrent, i, j) * 1000;
    //             if (d < 0)
    //             {
    //                 d = 0;
    //             }
    //             // std::cout<<"i: "<<i<<"j: "<<j<<" d: "<<d<<" collide: "<<checkSelfCollision->twoLinksCollide(qcurrent, i, j)<<std::endl;
    //             // std::cout<<-(d*d*d*d)/(2*c*c)<<" exp: "<<std::exp(-(d*d*d*d)/(2*c*c))<<std::endl;

    //             sum += b * std::exp((-d * d * d * d) / (2 * c * c));
    //             k++;
    //         }
    //     }
    //     // std::cout<<"self-collision term: "<<sum<<std::endl;
    //     return sum;
    // }

    double endEffectorVelocityObjectiveTerm(const KDL::Frame &currentPose, const KDL::Frame &prevPose, double dt, int iPose)
    {
        if (iPose > 0)
        {
            return sqrt(pow((currentPose.p.x() - prevPose.p.x()), 2) + pow((currentPose.p.y() - prevPose.p.y()), 2) + pow((currentPose.p.z() - prevPose.p.z()), 2));
        }
        return 0.0;
    }

    double shoulderElbowGoalObjectiveTerm(const KDL::Frame &currentElbowPose, const KDL::Frame &currentShoulderPose, const Eigen::Vector3d &goalShoulderElbow)
    {
        // std::cout<<"goalSHoulderElbow: "<<goalShoulderElbow<<std::endl;
        Eigen::Vector3d pe(currentElbowPose.p.x(), currentElbowPose.p.y(), currentElbowPose.p.z());
        // std::cout<<pe<<std::endl;
        Eigen::Vector3d ps(currentShoulderPose.p.x(), currentShoulderPose.p.y(), currentShoulderPose.p.z());
        // std::cout<<ps<<std::endl;
        Eigen::Vector3d v_es = ps - pe;
        // std::cout<<"current shoulder elbow: "<<v_es<<std::endl;
        double cosAngle = v_es.dot(goalShoulderElbow) / (goalShoulderElbow.norm() * v_es.norm());

        // Eigen::Vector3d oppositev_es = -v_es;
        // Eigen::Vector3d oppositeGoalShoulderElbow = -goalShoulderElbow;
        // double auxCosAngle = v_es.dot(oppositeGoalShoulderElbow)/(v_es.norm()*oppositeGoalShoulderElbow.norm());
        // if(auxCosAngle > cosAngle){
        //     cosAngle = auxCosAngle;
        // }

        // auxCosAngle = oppositev_es.dot(goalShoulderElbow)/(oppositev_es.norm()*goalShoulderElbow.norm());
        // if(auxCosAngle > cosAngle){
        //     cosAngle = auxCosAngle;
        // }

        // std::cout<<"cosAngle: "<<cosAngle<<std::endl;

        double fvalue = (abs(acos(cosAngle)) - MIN_ORI_DISTANCE) / (MAX_ORI_DISTANCE - MIN_ORI_DISTANCE);
        return fvalue;
    }

    double elbowWristGoalObjectiveTerm(const KDL::Frame &currentElbowPose, const KDL::Frame &currentWristPose, const Eigen::Vector3d &goalElbowWrist)
    {

        Eigen::Vector3d pw(currentWristPose.p.x(), currentWristPose.p.y(), currentWristPose.p.z());
        Eigen::Vector3d pe(currentElbowPose.p.x(), currentElbowPose.p.y(), currentElbowPose.p.z());
        Eigen::Vector3d v_ew = pw - pe;
        // std::cout<<"goalElbowWrist: "<<goalElbowWrist<<std::endl;
        // std::cout<<"current ElbowWrist: "<<v_ew<<std::endl;
        double cosAngle = v_ew.dot(goalElbowWrist) / (goalElbowWrist.norm() * v_ew.norm());

        // Eigen::Vector3d oppositev_ew = -v_ew;
        // Eigen::Vector3d oppositeGoalElbowWrist = -goalElbowWrist;

        // double auxCosAngle = v_ew.dot(oppositeGoalElbowWrist)/(v_ew.norm()*oppositeGoalElbowWrist.norm());
        // if(auxCosAngle > cosAngle){
        //     cosAngle = auxCosAngle;
        // }

        // auxCosAngle = oppositev_ew.dot(goalElbowWrist)/(oppositev_ew.norm()*goalElbowWrist.norm());
        // if(auxCosAngle > cosAngle){
        //     cosAngle = auxCosAngle;
        // }
        // std::cout<<"cosAngle: "<<cosAngle<<std::endl;
        double fvalue = (abs(acos(cosAngle)) - MIN_ORI_DISTANCE) / (MAX_ORI_DISTANCE - MIN_ORI_DISTANCE);
        return fvalue;
    }

    //     double normalArmPlaneGoalObjectiveTerm(const KDL::Frame &currentWristPose, const KDL::Frame & currentElbowPose, const KDL::Frame & currentShoulderPose,
    //                                      const Eigen::Vector3d & normalArmPlaneGoal)
    //     {
    //         // First compute the current arm plane

    //         Eigen::Vector3d pw(currentWristPose.p.x(), currentWristPose.p.y(), currentWristPose.p.z());
    //         Eigen::Vector3d pe(currentElbowPose.p.x(), currentElbowPose.p.y(), currentElbowPose.p.z());
    //         Eigen::Vector3d ps(currentShoulderPose.p.x(), currentShoulderPose.p.y(), currentShoulderPose.p.z());

    //         std::cout<<"pw: "<<pw<<std::endl;
    //         std::cout<<"pe: "<<pe<<std::endl;
    //         std::cout<<"ps: "<<ps<<std::endl;

    //         Eigen::Vector3d v_es = ps - pe;
    //         Eigen::Vector3d v_ew = pw - pe;
    //         std::cout<<"v_es: "<<v_es<<std::endl;
    //         std::cout<<"v_ew: "<<v_ew<<std::endl;

    //         if(v_es.dot(v_ew)/(v_es.norm()*v_ew.norm()) == 1.0 || v_es.dot(v_ew)/(v_es.norm()*v_ew.norm())== -1.0){
    //             std::cout<<"Parallel vectors"<<std::endl;
    //         }

    //         Eigen::Vector3d normalCurrentArmPlane = v_es.cross(v_ew);

    //         std::cout<<"normalCurrentArmPlane: "<<normalCurrentArmPlane<<std::endl;
    //         std::cout<<"normalArmPlaneGoal: "<<normalArmPlaneGoal<<std::endl;

    //         Eigen::Vector3d oppositeNormalCurrentArmPlane = -normalCurrentArmPlane;
    //         Eigen::Vector3d oppositeNormalArmPlaneGoal = -normalArmPlaneGoal;
    //         double cosAngle = normalArmPlaneGoal.dot(normalCurrentArmPlane)/(normalArmPlaneGoal.norm()*normalCurrentArmPlane.norm());
    //   // Shoulder pose
    //                 KDL::Frame currentShoulderPose;

    //                 std::cout<<"q: "<<q(0)<<" "<<q(1)<<" "<<q(2)<<" "<<q(3)<<" "<<q(4)<<" "<<q(5)<<" "<<q(6)<<q(7)<<std::endl;

    //                 (*fksolver).JntToCart(q,currentShoulderPose, 3);

    //                 //Elbow pose
    //                 KDL::Frame currentElbowPose;
    //                 (*fksolver).JntToCart(q, currentElbowPose, 5);

    //                 //Wrist pose
    //                 KDL::Frame currentWristPose;
    //                 (*fksolver).JntToCart(q, currentWristPose, 7);

    //                 std::cout<<"shoulder: "<<currentShoulderPose.p.x()<<" "<<currentShoulderPose.p.y()<<" "<<currentShoulderPose.p.z()<<std::endl;
    //                 std::cout<<"elbow: "<<currentElbowPose.p.x()<<" "<<currentElbowPose.p.y()<<" "<<currentElbowPose.p.z()<<std::endl;
    //                 std::cout<<"wrist: "<<currentWristPose.p.x()<<" "<<currentWristPose.p.y()<<" "<<currentWristPose.p.z()<<std::endl;

    //         std::cout<<"cosAngle: "<<cosAngle<<std::endl;
    //         return 1-cosAngle;
    //     }

    double getObjectiveFunctionScore(int iPose, KDL::JntArray q, KDL::Frame currentPose, std::vector<int> usedFrames)
    {
        double sum = 0;
        // int realIndex = usedFrames[iPose];
        // double positionGoal[3] = {adjHandTrajectoryData[realIndex][0], adjHandTrajectoryData[realIndex][1], adjHandTrajectoryData[realIndex][2]};
        // double auxq[4];
        // currentPose.M.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);
        // Eigen::Quaternionf qc(auxq[3], auxq[0], auxq[1], auxq[2]);
        // double norm = sqrt(adjHandTrajectoryData[realIndex][3] * adjHandTrajectoryData[realIndex][3] + adjHandTrajectoryData[realIndex][4] * adjHandTrajectoryData[realIndex][4] + adjHandTrajectoryData[realIndex][5] * adjHandTrajectoryData[realIndex][5] + adjHandTrajectoryData[realIndex][6] * adjHandTrajectoryData[realIndex][6]);
        // KDL::Rotation rotKdl = KDL::Rotation::Quaternion(desiredDiscretePoses[realIndex][3] / norm, adjHandTrajectoryData[realIndex][4] / norm, adjHandTrajectoryData[realIndex][5] / norm, adjHandTrajectoryData[realIndex][6] / norm);
        // Eigen::Quaternionf qg(adjHandTrajectoryData[realIndex][3] / norm, adjHandTrajectoryData[realIndex][4] / norm, adjHandTrajectoryData[realIndex][5] / norm, adjHandTrajectoryData[realIndex][6] / norm);
        // // Shoulder pose
        // KDL::Frame currentShoulderPose;
        // // std::cout<<"q: "<<q(0)<<" "<<q(1)<<" "<<q(2)<<" "<<q(3)<<" "<<q(4)<<" "<<q(5)<<" "<<q(6)<<q(7)<<std::endl;
        // (*fksolver).JntToCart(q, currentShoulderPose, 3);

        // // Elbow pose
        // KDL::Frame currentElbowPose;
        // (*fksolver).JntToCart(q, currentElbowPose, 5);

        // // Wrist pose
        // KDL::Frame currentWristPose;
        // (*fksolver).JntToCart(q, currentWristPose, 7);

        // // std::cout<<"shoulder: "<<currentShoulderPose.p.x()<<" "<<currentShoulderPose.p.y()<<" "<<currentShoulderPose.p.z()<<std::endl;
        // // std::cout<<"elbow: "<<currentElbowPose.p.x()<<" "<<currentElbowPose.p.y()<<" "<<currentElbowPose.p.z()<<std::endl;
        // // std::cout<<"wrist: "<<currentWristPose.p.x()<<" "<<currentWristPose.p.y()<<" "<<currentWristPose.p.z()<<std::endl;

        // Eigen::Vector3d pw(desiredDiscreteWristPoses[(realIndex)][0], desiredDiscreteWristPoses[(realIndex)][1], desiredDiscreteWristPoses[(realIndex)][2]);
        // Eigen::Vector3d pe(desiredDiscreteElbowPoses[(realIndex)][0], desiredDiscreteElbowPoses[(realIndex)][1], desiredDiscreteElbowPoses[(realIndex)][2]);
        // Eigen::Vector3d ps(desiredDiscreteShoulderPoses[(realIndex)][0], desiredDiscreteShoulderPoses[(realIndex)][1], desiredDiscreteShoulderPoses[realIndex][2]);

        // // std::cout<<"wrist desired:"<<pw<<std::endl;
        // // std::cout<<"elbow desired:"<<pe<<std::endl;
        // // std::cout<<"shoulder desired:"<<ps<<std::endl;
        // Eigen::Vector3d v_es = ps - pe;
        // Eigen::Vector3d v_ew = pw - pe;

        // // Eigen::Vector3d normalArmPlaneGoal = v_es.cross(v_ew);
        // angleShoudlerElbow[iPose] = shoulderElbowGoalObjectiveTerm(currentElbowPose, currentShoulderPose, v_es);
        // angleElbowWrist[iPose] = elbowWristGoalObjectiveTerm(currentElbowPose, currentWristPose, v_ew);
        // sum += wes * angleShoudlerElbow[iPose] +
        //        wew * angleElbowWrist[iPose];
        // sum += wpos * positionGoalMatchingObjectiveTerm(currentPose, positionGoal) +
        //        wori * orientationGoalMatchingObjectiveTerm(qc, qg) +
        //        wjv * jointVelocityObjectiveTerm(qx, iPose, q);

        return sum;
    }

    // double jointVelocityObjectiveTerm(std::vector<std::array<double, NJoints>> qx, int iPose, KDL::JntArray q)
    // {
    //     double sum = 0;
    //     // std::cout<<"iPose= "<<iPose<<std::endl;

    //     if (iPose > 0)
    //     {
    //         for (unsigned int j = 0; j < NJoints; j++)
    //         {
    //             sum += pow((q(j) - qx[(iPose - 1)][j]) / TIME_STEP, 2);
    //             ;
    //         }
    //     }

    //     return sum / 2.0;
    // }

    double optimizationFunctionJoints(unsigned n, const double *x, double *grad, void *data)
    {
        iteration += 1;
        // std::cout << "optimization iter: " << iteration << std::endl;
        auto *voidToVector = reinterpret_cast<std::vector<double> *>(data);
        double sum = 0;
        double dt = 0.1;
        KDL::JntArray q(NJoints); // TODO correct in order to gather the q init from outside and also the size of the joints array

        // std::cout << "Se supone que estamos aqui" << std::endl;
        for (unsigned int j = 0; j < NJoints; j++)
        {
            q(j) = x[j];
            xprev[3][j] = x[j];
        }

        KDL::Frame currentPose;
        // std::cout << "Lets compute the forward kinematics" << std::endl;
        int foundfk = (*fksolver).JntToCart(q, currentPose);

        // std::cout << "Current Pose: " << currentPose.p.x() << " " << currentPose.p.y() << " " << currentPose.p.z() << " ";
        // Here we will need to change the index from when we gathered the desired positions
        double auxq[4];
        currentPose.M.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

        // std::cout << "Quat: " << auxq[0] << " " << auxq[1] << " " << auxq[2] << " " << auxq[3] << std::endl;
        //             KDL::Rotation rotKdl1 = KDL::Rotation::Quaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

        Eigen::Quaternionf qc(auxq[3], auxq[0], auxq[1], auxq[2]);

        double norm = sqrt((*voidToVector)[(indexDiscrete)*7 + 3] * (*voidToVector)[(indexDiscrete)*7 + 3] + (*voidToVector)[(indexDiscrete)*7 + 4] * (*voidToVector)[(indexDiscrete)*7 + 4] + (*voidToVector)[(indexDiscrete)*7 + 5] * (*voidToVector)[(indexDiscrete)*7 + 5] + (*voidToVector)[(indexDiscrete)*7 + 6] * (*voidToVector)[(indexDiscrete)*7 + 6]);
        KDL::Rotation rotKdl = KDL::Rotation::Quaternion((*voidToVector)[(indexDiscrete)*7 + 3] / norm, (*voidToVector)[(indexDiscrete)*7 + 4] / norm, (*voidToVector)[(indexDiscrete)*7 + 5] / norm, (*voidToVector)[(indexDiscrete)*7 + 6] / norm);
        rotKdl = rotKdl * KDL::Rotation::RotX(-KDL::PI / 2.0);
        rotKdl = rotKdl * KDL::Rotation::RotZ(KDL::PI / 4.0);
        //             // rotKdl = rotKdl * KDL::Rotation::RotX(KDL::PI/2.0);
        //             // rotKdl = rotKdl * KDL::Rotation::RotY(-KDL::PI / 2.0);
        //             // rotKdl = rotKdl*KDL::Rotation::RotX(-KDL::PI / 4.0);
        //             // rotKdl = rotKdl*KDL::Rotation::RotY(-KDL::PI / 5.0);
        //             // rotKdl = rotKdl*KDL::Rotation::RotZ(-KDL::PI / 10.0);

        rotKdl.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);
        Eigen::Quaternionf qg(auxq[3], auxq[0], auxq[1], auxq[2]);

        double positionGoal[3] = {(*voidToVector)[(indexDiscrete)*7], (*voidToVector)[(indexDiscrete)*7 + 1], (*voidToVector)[(indexDiscrete)*7 + 2]};
        //             std::cout << "wori: " << wori << std::endl;

        sum += wpos * positionGoalMatchingObjectiveTerm(currentPose, positionGoal) +
               wori * orientationGoalMatchingObjectiveTerm(qc, qg) +
               wjv * jointVelocityObjectiveTerm(x, velocity, dt, 3) +
               wja * jointAccelerationObjectiveTerm(x, acceleration, dt, 3) +
               wjj * jointJerkObjectiveTerm(x, acceleration, dt, 3);

        //                 //    wori * grooveLoss(orientationGoalMatchingObjectiveTerm(qc, qg), 1, 0, 0.1, 10.0) +
        //                 //    wv * grooveLoss(endEffectorVelocityObjectiveTerm(currentPose, prevPose, dt, i), 1, 0, 0.1, 10.0) +
        //                 //    wjj * grooveLoss(jointJerkObjectiveTerm(acceleration, jerk, dt, i), 1, 0, 0.1, 10.0) +
        //                 //    wsc * grooveLoss(selfCollisionObjectiveTerm(q), 1, 0.0, 0.1, 10.0);
        // #ifdef NOTONLYHAND

        Eigen::Vector3d pw(adjWristTrajectoryData[(indexDiscrete)][0], adjWristTrajectoryData[(indexDiscrete)][1], adjWristTrajectoryData[(indexDiscrete)][2]);
        Eigen::Vector3d pe(adjElbowTrajectoryData[(indexDiscrete)][0], adjElbowTrajectoryData[(indexDiscrete)][1], adjElbowTrajectoryData[(indexDiscrete)][2]);
        Eigen::Vector3d ps(adjShoulderTrajectoryData[(indexDiscrete)][0], adjShoulderTrajectoryData[(indexDiscrete)][1], adjShoulderTrajectoryData[(indexDiscrete)][2]);

        Eigen::Vector3d v_es = ps - pe;
        Eigen::Vector3d v_ew = pw - pe;

        std::cout << "wrist desired:" << pw << std::endl;
        std::cout << "elbow desired:" << pe << std::endl;
        std::cout << "shoulder desired:" << ps << std::endl;

        Eigen::Vector3d normalArmPlaneGoal = v_es.cross(v_ew);
        KDL::Frame currentShoulderPose;
        // std::cout<<"q: "<<q(0)<<" "<<q(1)<<" "<<q(2)<<" "<<q(3)<<" "<<q(4)<<" "<<q(5)<<" "<<q(6)<<q(7)<<std::endl;
        (*fksolver).JntToCart(q, currentShoulderPose, 3);

        // Elbow pose
        KDL::Frame currentElbowPose;
        (*fksolver).JntToCart(q, currentElbowPose, 5);

        // Wrist pose
        KDL::Frame currentWristPose;
        (*fksolver).JntToCart(q, currentWristPose, 7);


        double angleShoudlerElbow = shoulderElbowGoalObjectiveTerm(currentElbowPose, currentShoulderPose, v_es);
        double angleElbowWrist = elbowWristGoalObjectiveTerm(currentElbowPose, currentWristPose, v_ew);
        sum += wes * angleShoudlerElbow +
               wew * angleElbowWrist;

                    // Eigen::Vector3d normalArmPlaneGoal = v_es.cross(v_ew);

        //             // sum += wap*normalArmPlaneGoalObjectiveTerm(currentWristPose, currentElbowPose, currentWristPose, normalArmPlaneGoal);
        //             // std::cout<<"arm plane: "<<  wap*normalArmPlaneGoalObjectiveTerm(currentWristPose, currentElbowPose, currentShoulderPose, normalArmPlaneGoal)<<std::endl;
        // #endif
        prevPose = currentPose;

        // std::cout << "opt: " << sum << std::endl;
        return sum;
    }

    // void writeCsv(std::string filename, const double *x, const unsigned int &numPoses)
    // {

    //     std::ofstream myFile(filename);
    //     for (unsigned int i = 0; i < numPoses; i++)
    //     {
    //         double norm = sqrt(x[(i * 7) + 3] * x[i * 7 + 3] + x[i * 7 + 4] * x[i * 7 + 4] + x[i * 7 + 5] * x[i * 7 + 5] + x[i * 7 + 6] * x[i * 7 + 6]);
    //         myFile << i << "," << std::setprecision(14) << x[i * 7 + 0] << "," << x[i * 7 + 1] << "," << x[i * 7 + 2] << "," << x[i * 7 + 3] / norm << "," << x[i * 7 + 4] / norm << "," << x[i * 7 + 5] / norm << "," << x[i * 7 + 6] / norm << "\n";
    //     }
    //     myFile.close();
    // }

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

    void writeDataCsv(std::string filename, const std::vector<double> &positiondist, const std::vector<double> &orientatationdist, const std::vector<double> &angleShoudlerElbow, const std::vector<double> &angleElbowWrist)
    {
        std::ofstream myFile(filename);
        std::cout << positiondist.size() << " " << orientatationdist.size() << " " << angleShoudlerElbow.size() << " " << angleElbowWrist.size() << std::endl;
        for (unsigned int i = 0; i < orientatationdist.size(); i++)
        {
            myFile << i << " " << std::setprecision(10) << positiondist[i] << " " << orientatationdist[i] << " " << angleShoudlerElbow[i] << " " << angleElbowWrist[i] << "\n";
        }
        myFile.close();
    }

    void writePoseCsv(std::string filename, const std::vector<std::array<double, 7>> &poses)
    {
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
        auto *voidToVector = reinterpret_cast<std::vector<double> *>(data);

        for (unsigned int i = 0; i < NJoints; i++)
        {
            std::cout << x[i] << endl;
            std::cout << (*voidToVector)[i] << endl;
            result[i] = fabs(x[i] - (*voidToVector)[i]) / TIME_STEP - MAX_JVEL;
            std::cout << "result: " << result[i];
            // std::cout << "limit: " << fabs(x[i - NJoints] - x[i]) / TIME_STEP;
        }
    }

    KDL::Frame getFrame(std::vector<std::array<double, 7>> discreteHandPoses, int iPose)
    {
        KDL::Frame fPose;
        double norm = sqrt(discreteHandPoses[iPose][3] * discreteHandPoses[iPose][3] + discreteHandPoses[iPose][4] * discreteHandPoses[iPose][4] + discreteHandPoses[iPose][5] * discreteHandPoses[iPose][5] + discreteHandPoses[iPose][6] * discreteHandPoses[iPose][6]);
        KDL::Rotation rotKdl = KDL::Rotation::Quaternion(discreteHandPoses[iPose][3] / norm, discreteHandPoses[iPose][4] / norm, discreteHandPoses[iPose][5] / norm, discreteHandPoses[iPose][6] / norm);
        KDL::Vector posKdl = KDL::Vector(discreteHandPoses[iPose][0], discreteHandPoses[iPose][1], discreteHandPoses[iPose][2]);

        fPose.M = rotKdl;
        fPose.p = posKdl;
        return fPose;
    }
}

int main()
{
    KDL::JntArray qmin(8), qmax(8);
    makeQLimitsTeoTrunkAndRightArmKinematics(qmin, qmax);
    KDL::Chain chain = makeTeoTrunkAndRightArmKinematicsFromDH();
    std::cout << "chain: " << chain.segments.size() << std::endl;
    fksolver = new KDL::ChainFkSolverPos_recursive(chain);

    int nDemo = 1;
    string csvFile = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + to_string(nDemo) + "-smoothed.csv";
    vector<array<double, 7>> desiredTrajectoryData;
    vector<array<double, 7>> wristTrajectoryData;
    vector<array<double, 7>> elbowTrajectoryData;
    vector<array<double, 7>> shoulderTrajectoryData;
    vector<array<double, 7>> neckTrajectoryData;
    vector<array<double, 7>> hipTrajectoryData;
    HumanMotionData::getHumanData(csvFile, desiredTrajectoryData, wristTrajectoryData, elbowTrajectoryData, shoulderTrajectoryData, neckTrajectoryData, hipTrajectoryData);
    HumanMotionData::printHumanData(desiredTrajectoryData, wristTrajectoryData, elbowTrajectoryData, shoulderTrajectoryData, neckTrajectoryData, hipTrajectoryData);
    // HumanMotionData::getHumanTrajectoryPoses(desiredTrajectoryData, wristTrajectoryData, elbowTrajectoryData, shoulderTrajectoryData, neckTrajectoryData, hipTrajectoryData, 0.001);

    HumanMotionData::linkLengthAdjustementPoses(desiredTrajectoryData, wristTrajectoryData, elbowTrajectoryData, shoulderTrajectoryData, neckTrajectoryData, hipTrajectoryData,
                                                adjHipTrajectoryData, adjNeckTrajectoryData, adjShoulderTrajectoryData, adjElbowTrajectoryData, adjWristTrajectoryData, adjHandTrajectoryData);

    string csvLinkAdjFile = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + to_string(nDemo) + "-smoothed-link-adj.csv";

    HumanMotionData::writePosesCsv(csvLinkAdjFile, adjHandTrajectoryData, adjWristTrajectoryData, adjElbowTrajectoryData, adjShoulderTrajectoryData, adjNeckTrajectoryData, adjHipTrajectoryData);

    //  Now we will call the optimizer one pose at a time, taking into account the previous optimizations

    // std::reverse(adjHandTrajectoryData.begin(), adjHandTrajectoryData.end());

    xprev.resize(4);
    velocity.resize(4);
    acceleration.resize(4);
    jerk.resize(4);

    double x_one_by_one[NJoints];

    double lb_obo[NJoints];
    double ub_obo[NJoints];

    for (unsigned int j = 0; j < NJoints; j++)
    {
        lb_obo[j] = (qmin(j));
        ub_obo[j] = (qmax(j));
    }
    for (unsigned int j = 0; j < NJoints; j++)
    {
        x_one_by_one[j] = fRand(qmin(j), qmax(j));
    }
    double minf_obo = MAXFLOAT;

    // for (indexDiscrete = 1; indexDiscrete < numPoses - 3; indexDiscrete++)
    // {
    //     std::cout << "indexDiscrete: " << indexDiscrete << std::endl;
    //     std::cout << "numPoses: " << numPoses << std::endl;
    //     wori = (fabs(numPoses / 2.0 - indexDiscrete / 2.0) + 1) / float(numPoses / 2.0) * DEFAULT_WORI;
    // }
    std::vector<double> desiredDiscretePoses;

    for (int i = 0; i < adjHandTrajectoryData.size(); i++)
    {
        for (unsigned j = 0; j < 7; j++)
        {
            desiredDiscretePoses.push_back(adjHandTrajectoryData[i][j]);
        }
    }
    void *parameters = &desiredDiscretePoses;

    nlopt_opt opt_obo;
    opt_obo = nlopt_create(NLOPT_LN_COBYLA, NJoints);
    double tol_obo[NJoints];
    for (unsigned int i = 0; i < NJoints; i++)
        tol_obo[i] = 1e-4;

    std::vector<double> jprev;
    for (unsigned int i = 0; i < NJoints; i++)
        jprev.push_back(xprev[2][i]);

    void *data = &jprev;
    // nlopt_add_inequality_mconstraint(opt_obo, NJoints, sharon::jointVelocityLimit, data, tol_obo);
    nlopt_set_min_objective(opt_obo, sharon::optimizationFunctionJoints, parameters);
    nlopt_set_lower_bounds(opt_obo, lb_obo);
    nlopt_set_upper_bounds(opt_obo, ub_obo);

    std::vector<std::array<double, NJoints>> qTraj;
    for (indexDiscrete = 0; indexDiscrete <= 700; indexDiscrete++)
    {
        try
        {
            double maxTime = 100000;
            // nlopt_set_maxtime(opt, maxTime);
            nlopt_set_ftol_abs(opt_obo, 1e-10);
            // nlopt_set_ftol_rel(opt, 0.000001);
            for (int j = 0; j < NJoints; j++)
            {
                if (x_one_by_one[j] < qmin(j))
                {
                    std::cout << "j: " << j << " Outside Bounds!" << std::endl;
                    x_one_by_one[j] = x_one_by_one[j] + 0.0005;
                }
                if (x_one_by_one[j] > qmax(j))
                {
                    x_one_by_one[j] = x_one_by_one[j] - 0.0005;
                }
            }
            nlopt_result result = nlopt_optimize(opt_obo, x_one_by_one, &minf_obo);
            std::cout << "found minimum at f(x)=" << minf_obo << std::endl;
            std::array<double, NJoints> qArray;

            for (unsigned int joint = 0; joint < NJoints; joint++)
                qArray[joint] = x_one_by_one[joint] * KDL::rad2deg;
            qTraj.push_back(qArray);

            // ------ Poses ---------------- //
            KDL::JntArray q(NJoints); // TODO correct in order to gather the q init from outside and also the size of the joints array
            for (unsigned int j = 0; j < NJoints; j++)
            {
                q(j) = x_one_by_one[j];
                xprev[3][j] = x_one_by_one[j];
            }
            for (int i = 0; i < 3; i++)
            {
                for (unsigned int j = 0; j < NJoints; j++)
                {
                    xprev[i][j] = xprev[i + 1][j];
                    velocity[i][j] = velocity[i + 1][j];
                }
            }
            std::cout << "x_one_by_one[0]: " << x_one_by_one[0] << endl;
            std::cout << "xprev0: " << xprev[0][0] << " xprev1: " << xprev[1][0] << " xprev2: " << xprev[2][0] << " xprev3: " << xprev[3][0] << endl;
            std::cout << "velocity0: " << velocity[0][0] << " velocity1: " << velocity[1][0] << " velocity2: " << velocity[2][0] << " velocity3: " << velocity[3][0] << endl;

            KDL::Frame currentPose;
            int foundfk = (*fksolver).JntToCart(q, currentPose);
            double quat[4];
            currentPose.M.GetQuaternion(quat[0], quat[1], quat[2], quat[3]);
            std::array<double, 7> pose;
            pose[0] = currentPose.p.x();
            pose[1] = currentPose.p.y();
            pose[2] = currentPose.p.z();
            pose[3] = quat[0];
            pose[4] = quat[1];
            pose[5] = quat[2];
            pose[6] = quat[3];

            double norm = sqrt((desiredDiscretePoses)[(indexDiscrete)*7 + 3] * (desiredDiscretePoses)[(indexDiscrete)*7 + 3] + (desiredDiscretePoses)[(indexDiscrete)*7 + 4] * (desiredDiscretePoses)[(indexDiscrete)*7 + 4] + (desiredDiscretePoses)[(indexDiscrete)*7 + 5] * (desiredDiscretePoses)[(indexDiscrete)*7 + 5] + (desiredDiscretePoses)[(indexDiscrete)*7 + 6] * (desiredDiscretePoses)[(indexDiscrete)*7 + 6]);
            KDL::Rotation rotKdl = KDL::Rotation::Quaternion((desiredDiscretePoses)[(indexDiscrete)*7 + 3] / norm, (desiredDiscretePoses)[(indexDiscrete)*7 + 4] / norm, (desiredDiscretePoses)[(indexDiscrete)*7 + 5] / norm, (desiredDiscretePoses)[(indexDiscrete)*7 + 6] / norm);
            rotKdl = rotKdl * KDL::Rotation::RotX(-KDL::PI / 2.0);
            rotKdl = rotKdl * KDL::Rotation::RotZ(KDL::PI / 4.0);
            // rotKdl = rotKdl * KDL::Rotation::RotX(KDL::PI / 2.0);
            // rotKdl = rotKdl * KDL::Rotation::RotY(-KDL::PI / 2.0);
            // rotKdl = rotKdl * KDL::Rotation::RotX(-KDL::PI / 4.0);
            // rotKdl = rotKdl * KDL::Rotation::RotY(-KDL::PI / 5.0);
            // rotKdl = rotKdl*KDL::Rotation::RotZ(-KDL::PI / 10.0);
            double auxq[4];
            rotKdl.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

            Eigen::Quaternionf qg(auxq[3], auxq[0], auxq[1], auxq[2]);
            Eigen::Quaternionf qcurrent(quat[3], quat[0], quat[1], quat[2]);
            double positionGoal[3] = {(desiredDiscretePoses)[(indexDiscrete)*7], (desiredDiscretePoses)[(indexDiscrete)*7 + 1], (desiredDiscretePoses)[(indexDiscrete)*7 + 2]};
            double dist = sqrt((currentPose.p.x() - positionGoal[0]) * (currentPose.p.x() - positionGoal[0]) +
                               (currentPose.p.y() - positionGoal[1]) * (currentPose.p.y() - positionGoal[1]) +
                               (currentPose.p.z() - positionGoal[2]) * (currentPose.p.z() - positionGoal[2]));
            positiondist.push_back(dist);
            double inner_product = qcurrent.x() * qg.x() + qcurrent.y() * qg.y() + qcurrent.z() * qg.z() + qcurrent.w() * qg.w();
            double angle = acos(2 * inner_product * inner_product - 1);

            orientatationdist.push_back(angle);
            angleElbowWrist.push_back(0.0);
            angleShoudlerElbow.push_back(0.0);
            poses.push_back(pose);
            std::cout << "iii: " << indexDiscrete << std::endl;
        }
        catch (std::exception &e)
        {
            std::cout << "nlopt failed: " << e.what() << std::endl;
            // break;
        }
        std::cout << "iii: " << indexDiscrete << std::endl;
    }

    //     KDL::JntArray qmin(NJoints), qmax(NJoints);
    //     makeQLimitsTeoTrunkAndRightArmKinematics(qmin, qmax);
    //     createSelfCollisionObjects();
    //     KDL::Chain chain = makeTeoTrunkAndRightArmKinematicsFromDH();
    //     std::cout<<"chain: "<<chain.segments.size()<<std::endl;

    //     checkSelfCollision = new sharon::CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects, tableCollision);

    //     std::vector<int> usedFrames;
    //     #ifdef NOTONLYHAND
    //         std::cout<<"Not hand only"<<std::endl;
    //         std::string csvFile = "/home/elisabeth/repos/teo-sharon/programs/GenerateManipulationTrajectories/trajectories/shoulderElbowWirstHandTest/prueba"+std::to_string(nDemo)+"-smoothed.csv";
    //         std::vector<std::array<double, 8>> desiredTrajectoryData;
    //         std::vector<std::array<double, 7>> wristTrajectoryData;
    //         std::vector<std::array<double, 7>> elbowTrajectoryData;
    //         std::vector<std::array<double, 7>> shoulderTrajectoryData;
    //         sharon::getHumanData(csvFile, desiredTrajectoryData, wristTrajectoryData, elbowTrajectoryData, shoulderTrajectoryData);
    //         sharon::printHumanData(desiredTrajectoryData, wristTrajectoryData, elbowTrajectoryData, shoulderTrajectoryData);
    //         sharon::getHumanTrajectoryPoses(desiredTrajectoryData, wristTrajectoryData, elbowTrajectoryData, shoulderTrajectoryData, (float) 0.01, desiredDiscretePoses,
    //                                         desiredDiscreteWristPoses, desiredDiscreteElbowPoses, desiredDiscreteShoulderPoses, numPoses, usedFrames);
    //     #else
    //         std::string csvFile = "/home/elisabeth/repos/teo-sharon/programs/GenerateManipulationTrajectories/trajectories/graspcup2/test-right-arm-motion-smooth"+std::to_string(nDemo)+"-reaching.csv";
    //         std::vector<std::array<double, NJoints>> desiredTrajectoryData = sharon::getTrajectoryFromCsvFile(csvFile);
    //         std::cout<<"Print Human Data"<<std::endl;
    //         sharon::printTrajectoryData(desiredTrajectoryData);
    //         std::vector<double> desiredDiscretePoses;
    //         sharon::getTrajectoryPoses(desiredTrajectoryData, (float)0.2, desiredDiscretePoses, numPoses);
    //     #endif

    //     std::cout << "numPoses: " << numPoses << std::endl;
    //     std::cout << "usedFrames: "<< usedFrames.back()<<std::endl;

    //     // Subsampled poses are in desiredDiscretePoses (human hand), desiredDiscreteWristPoses (human wrist), desiredDiscreteElbowPoses(human elbow), desiredDiscreteShoulderPoses (Human Shoulder)

    //     // Get random joint configuration for each of the human hand poses

    //     iksolverv = new KDL::ChainIkSolverVel_pinv(chain);
    //     fksolver = new KDL::ChainFkSolverPos_recursive(chain);
    //     int solverMaxIter = 1000;
    //     float eps = 1e-1;
    //     iksolver = new KDL::ChainIkSolverPos_NR_JL(chain, qmin, qmax, *fksolver, *iksolverv, solverMaxIter, eps);
    //     qx.resize(usedFrames.size());
    //     angleShoudlerElbow.resize(usedFrames.size());
    //     angleElbowWrist.resize(usedFrames.size());
    //     for(int iPose=0; iPose < usedFrames.size(); iPose++){
    //         std::cout<<"iPose: "<<iPose<<std::endl;
    //         double score = std::numeric_limits<double>::max();

    //         KDL::Frame fPose = sharon::getFrame(desiredDiscretePoses, usedFrames[iPose]);
    //         for(int kk=0; kk<80; kk++){
    //             // For each pose lets get a -0.001random configuration
    //             KDL::JntArray  qInit(NJoints);
    //             KDL::JntArray  q(NJoints);

    //             if(iPose != 0 && kk == 0){
    //                 std::cout<<"same qInit: ";
    //                 for(unsigned int j = 0; j<NJoints; j++){
    //                     qInit(j) = qx[iPose-1][j];
    //                     std::cout<<qInit(j)<<" ";
    //                 }
    //                 std::cout<<std::endl;

    //             }
    //             else{
    //                 std::cout<<"qInit: ";
    //                 for(unsigned int j = 0; j<NJoints; j++){
    //                     qInit(j) = fRand(qmin(j),qmax(j));
    //                     std::cout<<qInit(j)<<" ";
    //                 }
    //                 std::cout<<std::endl;
    //             }

    //             bool solutionOutsideBounds = false;
    //             int foundik = (*iksolver).CartToJnt(qInit, fPose, q);
    //             if (foundik == 0) //ik solution found
    //             {
    //                 std::cout<<"Found solution for iPose: "<<iPose<<std::endl;
    //                 for(int j=0; j<NJoints; j++)
    //                 {
    //                     if(q(j)>=(qmax(j)+aux) || q(j)<=(qmin(j)-aux)){
    //                         std::cout<<"solution outside bounds q: "<<q(j)<<" min: "<<qmin(j)<<" max: "<<qmax(j)<<std::endl;
    //                         solutionOutsideBounds = true;
    //                         break;
    //                     }
    //                 }

    //                 if(!solutionOutsideBounds){
    //                     KDL::Frame currentPose;
    //                     int foundfk = (*fksolver).JntToCart(q, currentPose);

    //                     double new_score = sharon::getObjectiveFunctionScore(iPose, q, currentPose, usedFrames);
    //                     if (new_score < score){
    //                         score = new_score;
    //                         for(int j=0; j<NJoints; j++)
    //                             qx[iPose][j] = q(j);
    //                     }
    //                 }

    //             }
    //         }
    //     }

    //     for(int i=0; i<qx.size(); i++){
    //         std::cout<<"i: "<<i<<"q: "<<qx[i][0]<<" "<<qx[i][1]<<" "<<qx[i][2]<<" "<<qx[i][3]<<" "<<qx[i][4]<<" "<<qx[i][5]<<" "<<qx[i][6]<<" "<<qx[i][7]<<std::endl;
    //         std::cout<<"x: "<<desiredDiscretePoses[usedFrames[i]][0]<<" "<<desiredDiscretePoses[usedFrames[i]][1]<<" "<<desiredDiscretePoses[usedFrames[i]][2]<<std::endl;
    //         std::cout<<"ori: "<<desiredDiscretePoses[usedFrames[i]][3]<<" "<<desiredDiscretePoses[usedFrames[i]][4]<<" "<<desiredDiscretePoses[usedFrames[i]][5]<<" "<<desiredDiscretePoses[usedFrames[i]][6]<<std::endl;
    //         KDL::Frame currentPose;
    //         KDL::JntArray q(NJoints);
    //         for(int j=0; j<NJoints; j++)
    //             q(j) = qx[i][j];
    //         (*fksolver).JntToCart(q, currentPose);
    //         double quat[4];
    //         currentPose.M.GetQuaternion(quat[0], quat[1], quat[2],quat[3]);
    //         std::cout<<"xcurrent: "<<currentPose.p.x()<<" "<<currentPose.p.y()<<" "<<currentPose.p.z()<<std::endl;
    //         std::cout<<"ori current: "<<quat[0]<<" "<<quat[1]<<" "<<quat[2]<<" "<<quat[3]<<std::endl;
    //     }
    //     //Now I have the poses initialized
    //     int nFirstOptimization = desiredDiscretePoses.size();

    //     // xprev.reserve(nFirstOptimization);
    //     // velocity.reserve(nFirstOptimization);
    //     // acceleration.reserve(nFirstOptimization);
    //     // jerk.reserve(nFirstOptimization);
    //     // angleElbowWrist.resize(numPoses);
    //     // angleShoudlerElbow.resize(numPoses);

    //     // // joint velocity, acceleration, jerk are set to zero at t=0 for all the joints
    //     // for (unsigned int j = 0; j < NJoints; j++)
    //     // {
    //     //     velocity[0][j] = 0.0;
    //     //     acceleration[0][j] = 0.0;
    //     //     acceleration[1][j] = 0.0;
    //     //     jerk[0][j] = 0.0;
    //     //     jerk[1][j] = 0.0;
    //     //     jerk[2][j] = 0.0;
    //     // }

    //     // double x[nFirstOptimization * NJoints];

    //     // int i=0;
    //     // for(int iPose=0; iPose<nFirstOptimization; iPose++){
    //     //     for(int j=0; j<NJoints; j++){
    //     //         x[i] = qx[iPose][j];
    //     //     }
    //     // }
    //     // void *parameters = &desiredDiscretePoses;

    //     // nlopt_opt opt;
    //     // opt = nlopt_create(NLOPT_LN_BOBYQA, nFirstOptimization * NJoints);
    //     // indexDiscrete = 0;
    //     // nlopt_set_min_objective(opt, sharon::optimizationFunctionJoints, parameters);
    //     // double lb[nFirstOptimization * NJoints];
    //     // double ub[nFirstOptimization * NJoints];

    //     // for (unsigned int i = 0; i < nFirstOptimization; i++)
    //     // {
    //     //     for (unsigned int j = 0; j < NJoints; j++)
    //     //     {
    //     //         lb[i * NJoints + j] = (qmin(j));
    //     //         ub[i * NJoints + j] = (qmax(j));
    //     //     }
    //     // }

    //     // nlopt_set_lower_bounds(opt, lb);
    //     // nlopt_set_upper_bounds(opt, ub);
    //     // double tol[nFirstOptimization * NJoints];
    //     // for (unsigned int i = 0; i < nFirstOptimization * NJoints; i++)
    //     //     tol[i] = 1e-6;

    //     // // nlopt_add_inequality_mconstraint(opt, nFirstOptimization * NJoints, sharon::jointVelocityLimit, NULL, tol);

    //     // double minf=MAXFLOAT;
    //     // std::vector<std::array<double, NJoints>> qTraj;
    //     // std::array<double, NJoints> qArray;
    //     // std::vector<int> results;

    //     // double maxTime = 100000;
    //     // // nlopt_set_maxtime(opt, maxTime);
    //     // nlopt_set_ftol_abs(opt, 1e-10);
    //     // // nlopt_set_ftol_rel(opt, 0.000001);
    //     // nlopt_result result = nlopt_optimize(opt, x, &minf);
    //     // std::cout << "found minimum at f(x)=" << minf << std::endl;

    //     // std::fill_n(x, nFirstOptimization * NJoints, 0.05);

    //     return 0;
    // }
    //     //---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------//
    //     // Start optimizing first 4 poses
    //     // int nFirstOptimization = 10;

    //     // xprev.reserve(nFirstOptimization);
    //     // velocity.reserve(nFirstOptimization);
    //     // acceleration.reserve(nFirstOptimization);
    //     // jerk.reserve(nFirstOptimization);
    //     // angleElbowWrist.resize(numPoses);
    //     // angleShoudlerElbow.resize(numPoses);

    //     // // joint velocity, acceleration, jerk are set to zero at t=0 for all the joints
    //     // for (unsigned int j = 0; j < NJoints; j++)
    //     // {
    //     //     velocity[0][j] = 0.0;
    //     //     acceleration[0][j] = 0.0;
    //     //     acceleration[1][j] = 0.0;
    //     //     jerk[0][j] = 0.0;
    //     //     jerk[1][j] = 0.0;
    //     //     jerk[2][j] = 0.0;
    //     // }

    //     // double x[nFirstOptimization * NJoints];
    //     // std::fill_n(x, nFirstOptimization * NJoints, 0.05);

    //     // void *parameters = &desiredDiscretePoses;

    //     // fksolver = new KDL::ChainFkSolverPos_recursive(chain);
    //     // std::cout<<"N segments: "<<chain.getNrOfSegments()<<std::endl;

    //     // // Shoulder segment 1
    //     // // Elbow segment 4
    //     // // Wrist segment 6
    //     // // std::cout<<"x: "<<chain.getSegment(1).getFrameToTip().p.x()<<" y: "<<chain.getSegment(1).getFrameToTip().p.y()<<" z: "<<chain.getSegment(1).getFrameToTip().p.z()<<std::endl;
    //     // // std::cout<<"x: "<<chain.getSegment(4).getFrameToTip().p.x()<<" y: "<<chain.getSegment(4).getFrameToTip().p.y()<<" z: "<<chain.getSegment(4).getFrameToTip().p.z()<<std::endl;
    //     // // std::cout<<"x: "<<chain.getSegment(6).getFrameToTip().p.x()<<" y: "<<chain.getSegment(6).getFrameToTip().p.y()<<" z: "<<chain.getSegment(6).getFrameToTip().p.z()<<std::endl;

    //     // nlopt_opt opt;
    //     // opt = nlopt_create(NLOPT_LN_BOBYQA, nFirstOptimization * NJoints);
    //     // indexDiscrete = 0;
    //     // nlopt_set_min_objective(opt, sharon::optimizationFunctionJoints, parameters);
    //     // double lb[nFirstOptimization * NJoints];
    //     // double ub[nFirstOptimization * NJoints];

    //     // for (unsigned int i = 0; i < nFirstOptimization; i++)
    //     // {
    //     //     for (unsigned int j = 0; j < NJoints; j++)
    //     //     {
    //     //         lb[i * NJoints + j] = (qmin(j)+0.001)* KDL::deg2rad;
    //     //         ub[i * NJoints + j] = (qmax(j)-0.001)* KDL::deg2rad;
    //     //     }
    //     // }

    //     // // wori = DEFAULT_WORI;

    //     // std::cout << x[0];
    //     // nlopt_set_lower_bounds(opt, lb);
    //     // nlopt_set_upper_bounds(opt, ub);
    //     // double tol[nFirstOptimization * NJoints];
    //     // for (unsigned int i = 0; i < nFirstOptimization * NJoints; i++)
    //     //     tol[i] = 1e-6;

    //     // nlopt_add_inequality_mconstraint(opt, nFirstOptimization * NJoints, sharon::jointVelocityLimit, NULL, tol);

    //     // double minf=MAXFLOAT;
    //     // std::vector<std::array<double, NJoints>> qTraj;
    //     // std::array<double, NJoints> qArray;
    //     // std::vector<int> results;

    //     // double maxTime = 100000;
    //     // // nlopt_set_maxtime(opt, maxTime);
    //     // nlopt_set_ftol_abs(opt, 1e-10);
    //     // // nlopt_set_ftol_rel(opt, 0.000001);
    //     // nlopt_result result = nlopt_optimize(opt, x, &minf);
    //     // std::cout << "found minimum at f(x)=" << minf << std::endl;

    //     // for (unsigned int i = 0; i < nFirstOptimization; i++)
    //     // {
    //     //     for (unsigned int joint = 0; joint < NJoints; joint++)
    //     //         qArray[joint] = x[i * NJoints + joint] * KDL::rad2deg;
    //     //     qTraj.push_back(qArray);
    //     // }

    //     // // This is just for having also the path in cartersian space
    //     // for (unsigned int i = 0; i < nFirstOptimization; i++){

    //     //     KDL::JntArray q(NJoints); // TODO correct in order to gather the q init from outside and also the size of the joints array
    //     //     for (unsigned int j = 0; j < NJoints; j++)
    //     //     {
    //     //         q(j) = x[i * NJoints + j];
    //     //         // q(j) = 0.0;
    //     //     }

    //     //     KDL::Frame currentPose;
    //     //     int foundfk = (*fksolver).JntToCart(q, currentPose);
    //     //     double quat[4];
    //     //     currentPose.M.GetQuaternion(quat[0], quat[1], quat[2], quat[3]);    //     // std::reverse(poses.begin(), poses.end());

    //     //     pose[2] = currentPose.p.z()+0.894;
    //     //     pose[3] = quat[0];
    //     //     pose[4] = quat[1];
    //     //     pose[5] = quat[2];
    //     //     pose[6] = quat[3];
    //     //     std::cout<<"quat: "<<quat[0]<<" "<<quat[1]<<" "<<quat[2]<<" "<<quat[3]<<std::endl;

    //     //     poses.push_back(pose);

    //     //     double norm = sqrt((desiredDiscretePoses)[(indexDiscrete + i) * 7 + 3] * (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 3] + (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 4] * (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 4] + (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 5] * (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 5] + (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 6] * (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 6]);
    //     //     KDL::Rotation rotKdl = KDL::Rotation::Quaternion((desiredDiscretePoses)[(indexDiscrete + i) * 7 + 3] / norm, (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 4] / norm, (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 5] / norm, (desiredDiscretePoses)[(indexDiscrete + i) * 7 + 6] / norm);
    //     //         rotKdl = rotKdl * KDL::Rotation::RotX(KDL::PI/2.0);
    //     //         rotKdl = rotKdl * KDL::Rotation::RotY(-KDL::PI / 2.0);
    //     //         rotKdl = rotKdl*KDL::Rotation::RotX(-KDL::PI / 4.0);
    //     //         rotKdl = rotKdl*KDL::Rotation::RotY(-KDL::PI / 5.0);
    //     //         // rotKdl = rotKdl*KDL::Rotation::RotZ(KDL::PI / 10.0);

    //     //     double auxq[4];
    //     //     rotKdl.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

    //     //     Eigen::Quaternionf qg(auxq[3], auxq[0], auxq[1], auxq[2]);
    //     //     Eigen::Quaternionf qcurrent(quat[3], quat[0], quat[1], quat[2]);

    //     //     orientatationdist.push_back(sharon::orientationGoalMatchingObjectiveTerm(qcurrent, qg));

    //     // }

    //     // for (unsigned int i = 0; i < nFirstOptimization; i++)
    //     // {
    //     //     std::cout << "xprev: ";
    //     //     for (unsigned int j = 0; j < NJoints; j++)
    //     //     {
    //     //         std::cout << xprev[i][j] << " ";
    //     //     }
    //     //     std::cout << std::endl;
    //     //     std::cout << "velocity: ";
    //     //     for (unsigned int j = 0; j < NJoints; j++)
    //     //     {
    //     //         std::cout << velocity[i][j] << " ";
    //     //     }
    //     //     std::cout << std::endl;
    //     //     std::cout << "acceleration: ";
    //     //     for (unsigned int j = 0; j < NJoints; j++)
    //     //     {
    //     //         std::cout << acceleration[i][j] << " ";
    //     //     }
    //     //     std::cout << std::endl;
    //     //     std::cout << "jerk: ";
    //     //     for (unsigned int j = 0; j < NJoints; j++)
    //     //     {
    //     //         std::cout << jerk[i][j] << " ";
    //     //     }
    //     //     std::cout << std::endl;
    //     // }

    //     // // Now we will call the optimizer one pose at a time, taking into account the previous optimizations
    //     // double x_one_by_one[NJoints];

    //     // double lb_obo[NJoints];
    //     // double ub_obo[NJoints];

    //     // for (unsigned int j = 0; j < NJoints; j++)
    //     // {
    //     //     lb_obo[j] = (qmin(j)) * KDL::deg2rad;
    //     //     ub_obo[j] = (qmax(j)) * KDL::deg2rad;
    //     // }

    //     // for (indexDiscrete = 1; indexDiscrete < numPoses-3; indexDiscrete++)
    //     // {
    //     //     double minf_obo=MAXFLOAT;
    //     //     std::cout<<"indexDiscrete: "<< indexDiscrete<<std::endl;
    //     //     std::cout << "numPoses: " << numPoses << std::endl;
    //     //     wori = (fabs(numPoses/2.0-indexDiscrete/2.0)+1)/float(numPoses/2.0)*DEFAULT_WORI;

    //     //     // Move to the left one position the elements of the xprev, velocity, acceleration and jerk
    //     //     for (unsigned int i = 0; i < 3; i++)
    //     //     {
    //     //         for (unsigned int j = 0; j < NJoints; j++)
    //     //         {
    //     //             xprev[i][j] = xprev[i + 1][j];
    //     //             velocity[i][j] = velocity[i + 1][j];
    //     //             // std::cout<<"vel: "<<velocity[i][j]<<" ";
    //     //             acceleration[i][j] = acceleration[i + 1][j];
    //     //             jerk[i][j] = jerk[i + 1][j];
    //     //         }
    //     //         // std::cout<<std::endl;

    //     //     }

    //     //     for (unsigned int j = 0; j < NJoints; j++)
    //     //     {
    //     //         x_one_by_one[j] = xprev[3][j];
    //     //         std::cout<<x_one_by_one[j]<<" lb: "<<lb_obo[j]<<" ub: "<<ub_obo[j]<<std::endl;
    //     //         if(x_one_by_one[j] <= lb_obo[j])
    //     //             x_one_by_one[j] = lb_obo[j]+0.005;
    //     //         else if(x_one_by_one[j] >= ub_obo[j])
    //     //             x_one_by_one[j] = ub_obo[j]-0.005;
    //     //     }
    //     //     std::cout<<std::endl;

    //     //     nlopt_opt opt_obo;
    //     //     opt_obo = nlopt_create(NLOPT_LN_BOBYQA, NJoints);
    //     //     nlopt_set_min_objective(opt_obo, sharon::optimizationFunctionJoints, parameters);
    //     //     nlopt_set_lower_bounds(opt_obo, lb_obo);
    //     //     nlopt_set_upper_bounds(opt_obo, ub_obo);
    //     //     double tol_obo[NJoints];
    //     //     for (unsigned int i = 0; i < NJoints; i++)
    //     //         tol_obo[i] = 1e-4;

    //     //     nlopt_add_inequality_mconstraint(opt_obo, NJoints, sharon::jointVelocityLimit, NULL, tol_obo);

    //     //     try
    //     //     {
    //     //         double maxTime = 100000;
    //     //         // nlopt_set_maxtime(opt, maxTime);
    //     //         nlopt_set_ftol_abs(opt_obo, 1e-10);
    //     //         // nlopt_set_ftol_rel(opt, 0.000001);
    //     //         nlopt_result result = nlopt_optimize(opt_obo, x_one_by_one, &minf_obo);
    //     //         std::cout << "found minimum at f(x)=" << minf_obo << std::endl;

    //     //         for (unsigned int joint = 0; joint < NJoints; joint++)
    //     //             qArray[joint] = x_one_by_one[joint] * KDL::rad2deg;
    //     //         qTraj.push_back(qArray);

    //     //         // ------ Poses ---------------- //
    //     //         KDL::JntArray q(NJoints); // TODO correct in order to gather the q init from outside and also the size of the joints array
    //     //         for (unsigned int j = 0; j < NJoints; j++)
    //     //         {
    //     //             q(j) = x_one_by_one[j];
    //     //         }

    //     //         KDL::Frame currentPose;
    //     //         int foundfk = (*fksolver).JntToCart(q, currentPose);
    //     //         double quat[4];
    //     //         currentPose.M.GetQuaternion(quat[0], quat[1], quat[2], quat[3]);
    //     //         std::array<double, 7> pose;
    //     //         pose[0] = currentPose.p.x();
    //     //         pose[1] = currentPose.p.y();
    //     //         pose[2] = currentPose.p.z() + TrunkHeight;
    //     //         pose[3] = quat[0];
    //     //         pose[4] = quat[1];
    //     //         pose[5] = quat[2];
    //     //         pose[6] = quat[3];

    //     //         double norm = sqrt((desiredDiscretePoses)[(indexDiscrete+3) * 7 + 3] * (desiredDiscretePoses)[(indexDiscrete+3) * 7 + 3] + (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 4] * (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 4] + (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 5] * (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 5] + (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 6] * (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 6]);
    //     //         KDL::Rotation rotKdl = KDL::Rotation::Quaternion((desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 3] / norm, (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 4] / norm, (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 5] / norm, (desiredDiscretePoses)[(indexDiscrete + 3) * 7 + 6] / norm);
    //     //         rotKdl = rotKdl * KDL::Rotation::RotX(KDL::PI/2.0);
    //     //         rotKdl = rotKdl * KDL::Rotation::RotY(-KDL::PI / 2.0);
    //     //         rotKdl = rotKdl*KDL::Rotation::RotX(-KDL::PI / 4.0);
    //     //         rotKdl = rotKdl*KDL::Rotation::RotY(-KDL::PI / 5.0);
    //     //         // rotKdl = rotKdl*KDL::Rotation::RotZ(-KDL::PI / 10.0);
    //     //         double auxq[4];
    //     //         rotKdl.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

    //     //         Eigen::Quaternionf qg(auxq[3], auxq[0], auxq[1], auxq[2]);
    //     //         Eigen::Quaternionf qcurrent(quat[3], quat[0], quat[1], quat[2]);

    //     //         orientatationdist.push_back(sharon::orientationGoalMatchingObjectiveTerm(qcurrent, qg));
    //     //         poses.push_back(pose);
    //     //         std::cout<<"iii: "<<indexDiscrete<<std::endl;

    //     //     }
    //     //     catch (std::exception &e)
    //     //     {
    //     //         std::cout << "nlopt failed: " << e.what() << std::endl;
    //     //         break;
    //     //     }
    //     //     std::cout<<"iii: "<<indexDiscrete<<std::endl;

    //     // }
    //     // std::reverse(qTraj.begin(), qTraj.end());
    std::string csvQFileWrite = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + to_string(nDemo) + "-q-positions-optimization.csv";
    sharon::writeQCsv(csvQFileWrite, qTraj);
    //     // std::cout<<"qTraj"<<std::endl;
    //     // std::reverse(poses.begin(), poses.end());
    std::string csvPoseFileWrite = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + to_string(nDemo) + "-poses-optimization.csv";
    sharon::writePoseCsv(csvPoseFileWrite, poses);
    //     // std::cout<<"poses"<<std::endl;
    //     // std::reverse(orientatationdist.begin(), orientatationdist.end());
    //     // std::cout<<"angle"<<std::endl;
    //     // std::reverse(angleElbowWrist.begin(), angleElbowWrist.end());
    //     // std::cout<<"angle"<<std::endl;
    //     // std::reverse(angleShoudlerElbow.begin(), angleShoudlerElbow.end());
    //     // std::cout<<"angle"<<std::endl;

    std::string csvDataFileWrite = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + to_string(nDemo) + "-smoothed-data-optimization.csv";
    sharon::writeDataCsv(csvDataFileWrite, positiondist, orientatationdist, angleShoudlerElbow, angleElbowWrist);

    return 0;
}