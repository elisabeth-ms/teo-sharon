#include "SplineBasedMotionRetargeting.hpp"

#include <iostream>
#include <fstream>
#include <sstream>
#include <nlopt.hpp>
#include <math.h>
#include <iomanip> // std::setprecision
#include <limits>  // std::numeric_limits
#define DEFAULT_WORI 2.0
#define MAX_POS_DISTANCE 1.0
#define MIN_POS_DISTANCE 0.0
#define MAX_ORI_DISTANCE M_PI
#define MIN_ORI_DISTANCE 0.0
#define MAX_JVEL 40.0 * KDL::deg2rad
#define MIN_JVEL 0.0
#define MAX_JACC 50.0 * KDL::deg2rad
#define MAX_JERK 90.0 * KDL::deg2rad
#define MAX_DIFF_POSITION 0.1
#define SPLINE_ORDER 4
#define NPOINTS 100
#define NSPLINES 1
#define TIME_STEP 1.0
float aux = 0.001;

KDL::ChainIkSolverPos_NR_JL *iksolver;
KDL::ChainFkSolverPos_recursive *fksolver;
KDL::ChainIkSolverVel_pinv *iksolverv;
KDL::ChainJntToJacSolver *jntToJacSolver;

std::vector<double> qTrajectory;
// std::vector<fcl::CollisionObjectf> collisionObjects;
// std::vector<std::array<float, 3>> offsetCollisionObjects;
// typedef std::shared_ptr<fcl::CollisionGeometryf> CollisionGeometryPtr_t;
// sharon::CheckSelfCollision *checkSelfCollision;
// std::vector<fcl::CollisionObjectf> tableCollision;

unsigned int numPoses = 0;
// unsigned int indexDiscrete = 0;
int nDemo = 2;
unsigned int iteration = 0;
int nColsData = 29;

// std::vector<std::array<double, 7>> desiredDiscreteWristPoses;
// std::vector<std::array<double, 7>> desiredDiscreteElbowPoses;
// std::vector<std::array<double, 7>> desiredDiscreteShoulderPoses;
KDL::JntArray qmin(NJoints), qmax(NJoints);

vector<array<double, 7>> adjHandTrajectoryData;
vector<array<double, 7>> adjWristTrajectoryData;
vector<array<double, 7>> adjElbowTrajectoryData;
vector<array<double, 7>> adjShoulderTrajectoryData;
vector<array<double, 7>> adjNeckTrajectoryData;
vector<array<double, 7>> adjHipTrajectoryData;

float wpos = 10.0; // weight for end-effector position matching objective term
float wori = 5.0;  // weight for end-effector orientation matching objective term
float wjv = 15.0;  // weight for joint velocity objective term
float wja = 12.0;  // weight for joint acceleration objective term
float wjj = 12.0;  // weight for joint jerk objective term
float wv = 10.0;   // weight for end-effector position velocity objective term
float wsc = 0.0;   // weight for self-collision avoidance objective term
float wes = 5.0;
float wew = 5.0;
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

    // double jointVelocityObjectiveTerm(const double *x, std::vector<std::array<double, NJoints>> &velocity, double dt, int iPose)
    // {
    //     double sum = 0;
    //     // std::cout<<"iPose= "<<iPose<<std::endl;
    //     if (indexDiscrete == 0)
    //     {
    //         for (unsigned int j = 0; j < NJoints; j++)
    //             velocity[iPose][j] = 0.0;
    //     }
    //     if (indexDiscrete > 0)
    //     {
    //         for (unsigned int j = 0; j < NJoints; j++)
    //         {
    //             velocity[iPose][j] = (x[j] - xprev[(iPose - 1)][j]) / TIME_STEP;
    //             // std::cout<<"velocity_j: "<<velocity[iPose][j];
    //             sum += (abs(velocity[iPose][j]) - MIN_JVEL) / (MAX_JVEL - MIN_JVEL);
    //             // sum += velocity[iPose][j]*velocity[iPose][j];
    //         }
    //     }

    //     return sum / NJoints;
    // }

    // double jointAccelerationObjectiveTerm(const double *x, std::vector<std::array<double, NJoints>> &acceleration, double dt, int iPose)
    // {
    //     double sum = 0;
    //     if (indexDiscrete == 0)
    //     {
    //         for (unsigned int j = 0; j < NJoints; j++)
    //         {
    //             acceleration[iPose][j] = 0.0;
    //         }
    //     }
    //     if (indexDiscrete == 1)
    //     {
    //         for (unsigned int j = 0; j < NJoints; j++)
    //         {
    //             double v0 = 0;
    //             double v1 = (x[j] - xprev[2][j]) / TIME_STEP;
    //             acceleration[iPose][j] = (v1 - v0) / TIME_STEP;
    //             sum += abs(acceleration[iPose][j]) / (MAX_JACC);
    //         }
    //     }
    //     if (indexDiscrete > 1)
    //     {
    //         for (unsigned int j = 0; j < NJoints; j++)
    //         {
    //             double v0 = (xprev[2][j] - xprev[1][j]) / TIME_STEP;
    //             double v1 = (x[j] - xprev[2][j]) / TIME_STEP;
    //             acceleration[iPose][j] = (v1 - v0) / TIME_STEP;
    //             sum += abs(acceleration[iPose][j]) / (MAX_JACC);
    //         }
    //     }

    //     return sum / NJoints;
    // }

    // double jointJerkObjectiveTerm(const double *x, std::vector<std::array<double, NJoints>> &jerk, double dt, int iPose)
    // {
    //     double sum = 0;
    //     if (indexDiscrete == 0)
    //     {
    //         for (unsigned int j = 0; j < NJoints; j++)
    //         {
    //             jerk[iPose][j] = 0;
    //         }
    //     }
    //     if (indexDiscrete == 1)
    //     {
    //         for (unsigned int j = 0; j < NJoints; j++)
    //         {
    //             double v0 = 0;
    //             double v1 = (x[j] - xprev[2][j]) / TIME_STEP;
    //             double a0 = 0;
    //             double a1 = (v1 - v0) / TIME_STEP;
    //             jerk[iPose][j] = (a1 - a0) / TIME_STEP;
    //             // sum += jerk[iPose][j] * jerk[iPose][j];
    //             sum += abs(jerk[iPose][j]) / (MAX_JERK);
    //         }
    //     }
    //     if (indexDiscrete >= 2)
    //     {
    //         for (unsigned int j = 0; j < NJoints; j++)
    //         {
    //             double v1 = (xprev[1][j] - xprev[0][j]) / TIME_STEP;
    //             double v2 = (xprev[2][j] - xprev[1][j]) / TIME_STEP;
    //             double v3 = (x[j] - xprev[2][j]) / TIME_STEP;
    //             double a2 = (v2 - v1) / TIME_STEP;
    //             double a3 = (v3 - v2) / TIME_STEP;
    //             jerk[iPose][j] = (a3 - a2) / TIME_STEP;
    //             // sum += jerk[iPose][j] * jerk[iPose][j];
    //             sum += abs(jerk[iPose][j]) / (MAX_JERK);
    //         }
    //     }
    //     return sum / NJoints;
    // }

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

            double sum = sqrt(pow((currentPose.p.x() - prevPose.p.x()), 2) + pow((currentPose.p.y() - prevPose.p.y()), 2) + pow((currentPose.p.z() - prevPose.p.z()), 2));
            sum = (sum - MAX_DIFF_POSITION) / (MAX_DIFF_POSITION - MIN_POS_DISTANCE);
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

    double optimizationFunctionJoints(unsigned n, const double *x, double *grad, void *data)
    {
        iteration += 1;
        // std::cout << "optimization iter: " << iteration << std::endl;
        auto *voidToVector = reinterpret_cast<std::vector<double> *>(data);
        double sum = 0;
        double dt = 0.1;

        double numMaxPoint = NPOINTS * NSPLINES;

        for (int currentSpline = 0; currentSpline < NSPLINES; currentSpline++)
        {

            for (int npoint = 0; npoint <= NPOINTS; npoint = npoint + 2)
            {
                KDL::JntArray q(NJoints);
                double t = npoint / double(NPOINTS);
                // std::cout<<"npoint: "<<npoint<<"t: "<<t<<std::endl;
                for (unsigned int j = 0; j < NJoints; j++)
                {
                    double aux_q = 0;
                    for (unsigned int i = 0; i <= SPLINE_ORDER; i++)
                    {
                        aux_q += x[i + (SPLINE_ORDER + 1) * j] * pow(t, i);
                    }
                    q(j) = aux_q;
                    // cout<<"q[joint="<<j<<"]: "<<q(j)<<std::endl;
                }

                KDL::Frame currentPose;
                // std::cout << "Lets compute the forward kinematics" << std::endl;

                int foundfk = (*fksolver).JntToCart(q, currentPose);
                double positionGoal[3] = {(*voidToVector)[(npoint)*7], (*voidToVector)[(npoint)*7 + 1], (*voidToVector)[(npoint)*7 + 2]};
                double auxq[4];
                currentPose.M.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

                // std::cout << "Quat: " << auxq[0] << " " << auxq[1] << " " << auxq[2] << " " << auxq[3] << std::endl;
                //             KDL::Rotation rotKdl1 = KDL::Rotation::Quaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

                Eigen::Quaternionf qc(auxq[3], auxq[0], auxq[1], auxq[2]);

                double norm = sqrt((*voidToVector)[(npoint)*7 + 3] * (*voidToVector)[(npoint)*7 + 3] + (*voidToVector)[(npoint)*7 + 4] * (*voidToVector)[(npoint)*7 + 4] + (*voidToVector)[(npoint)*7 + 5] * (*voidToVector)[(npoint)*7 + 5] + (*voidToVector)[(npoint)*7 + 6] * (*voidToVector)[(npoint)*7 + 6]);
                KDL::Rotation rotKdl = KDL::Rotation::Quaternion((*voidToVector)[(npoint)*7 + 3] / norm, (*voidToVector)[(npoint)*7 + 4] / norm, (*voidToVector)[(npoint)*7 + 5] / norm, (*voidToVector)[(npoint)*7 + 6] / norm);
                rotKdl = rotKdl * KDL::Rotation::RotX(-KDL::PI / 2.0);
                rotKdl = rotKdl * KDL::Rotation::RotZ(KDL::PI / 4.0);
                //             // rotKdl = rotKdl * KDL::Rotation::RotX(KDL::PI/2.0);
                //             // rotKdl = rotKdl * KDL::Rotation::RotY(-KDL::PI / 2.0);
                //             // rotKdl = rotKdl*KDL::Rotation::RotX(-KDL::PI / 4.0);
                //             // rotKdl = rotKdl*KDL::Rotation::RotY(-KDL::PI / 5.0);
                //             // rotKdl = rotKdl*KDL::Rotation::RotZ(-KDL::PI / 10.0);

                rotKdl.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);
                Eigen::Quaternionf qg(auxq[3], auxq[0], auxq[1], auxq[2]);
                sum += wpos * positionGoalMatchingObjectiveTerm(currentPose, positionGoal) +
                       wori * orientationGoalMatchingObjectiveTerm(qc, qg);

                Eigen::Vector3d pw(adjWristTrajectoryData[(npoint)][0], adjWristTrajectoryData[(npoint)][1], adjWristTrajectoryData[(npoint)][2]);
                Eigen::Vector3d pe(adjElbowTrajectoryData[(npoint)][0], adjElbowTrajectoryData[(npoint)][1], adjElbowTrajectoryData[(npoint)][2]);
                Eigen::Vector3d ps(adjShoulderTrajectoryData[(npoint)][0], adjShoulderTrajectoryData[(npoint)][1], adjShoulderTrajectoryData[(npoint)][2]);

                Eigen::Vector3d v_es = ps - pe;
                Eigen::Vector3d v_ew = pw - pe;

                // std::cout << "wrist desired:" << pw << std::endl;
                // std::cout << "elbow desired:" << pe << std::endl;
                // std::cout << "shoulder desired:" << ps << std::endl;

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

                double currentAngleShoudlerElbow = shoulderElbowGoalObjectiveTerm(currentElbowPose, currentShoulderPose, v_es);
                double currentAngleElbowWrist = elbowWristGoalObjectiveTerm(currentElbowPose, currentWristPose, v_ew);
                sum += wes * currentAngleShoudlerElbow +
                       wew * currentAngleElbowWrist;
            }
        }
        std::cout << "Sum: " << sum << std::endl;
        for (int j = 0; j < NJoints; j++)
        {
            for (int nparam = 0; nparam <= SPLINE_ORDER; nparam++)
                std::cout << "param(" << nparam << ", j=" << j << ")= " << x[nparam + (SPLINE_ORDER + 1) * j];
            std::cout << std::endl;
        }
        // }
        // for (int npoint = 0; npoint <= numMaxPoint; npoint = npoint + 2)
        // {
        //     KDL::JntArray q(NJoints);
        //     double t = npoint / numMaxPoint;
        //     // std::cout<<"npoint: "<<npoint<<"t: "<<t<<std::endl;
        //     for (unsigned int j = 0; j < NJoints; j++)
        //     {
        //         double aux_q = 0;
        //         for (unsigned int i = 0; i <= SPLINE_ORDER; i++)
        //         {
        //             aux_q += x[(spline_order + 1) * NJoints * current_spline + i + (SPLINE_ORDER + 1) * j] * pow(t, i);
        //         }
        //         q(j) = aux_q;
        //     }

        //     KDL::Frame currentPose;
        //     // std::cout << "Lets compute the forward kinematics" << std::endl;

        //     int foundfk = (*fksolver).JntToCart(q, currentPose);
        //     double positionGoal[3] = {(*voidToVector)[(npoint)*7], (*voidToVector)[(npoint)*7 + 1], (*voidToVector)[(npoint)*7 + 2]};
        //     double auxq[4];
        //     currentPose.M.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

        //     // std::cout << "Quat: " << auxq[0] << " " << auxq[1] << " " << auxq[2] << " " << auxq[3] << std::endl;
        //     //             KDL::Rotation rotKdl1 = KDL::Rotation::Quaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

        //     Eigen::Quaternionf qc(auxq[3], auxq[0], auxq[1], auxq[2]);

        //     double norm = sqrt((*voidToVector)[(npoint)*7 + 3] * (*voidToVector)[(npoint)*7 + 3] + (*voidToVector)[(npoint)*7 + 4] * (*voidToVector)[(npoint)*7 + 4] + (*voidToVector)[(npoint)*7 + 5] * (*voidToVector)[(npoint)*7 + 5] + (*voidToVector)[(npoint)*7 + 6] * (*voidToVector)[(npoint)*7 + 6]);
        //     KDL::Rotation rotKdl = KDL::Rotation::Quaternion((*voidToVector)[(npoint)*7 + 3] / norm, (*voidToVector)[(npoint)*7 + 4] / norm, (*voidToVector)[(npoint)*7 + 5] / norm, (*voidToVector)[(npoint)*7 + 6] / norm);
        //     rotKdl = rotKdl * KDL::Rotation::RotX(-KDL::PI / 2.0);
        //     rotKdl = rotKdl * KDL::Rotation::RotZ(KDL::PI / 4.0);
        //     //             // rotKdl = rotKdl * KDL::Rotation::RotX(KDL::PI/2.0);
        //     //             // rotKdl = rotKdl * KDL::Rotation::RotY(-KDL::PI / 2.0);
        //     //             // rotKdl = rotKdl*KDL::Rotation::RotX(-KDL::PI / 4.0);
        //     //             // rotKdl = rotKdl*KDL::Rotation::RotY(-KDL::PI / 5.0);
        //     //             // rotKdl = rotKdl*KDL::Rotation::RotZ(-KDL::PI / 10.0);

        //     rotKdl.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);
        //     Eigen::Quaternionf qg(auxq[3], auxq[0], auxq[1], auxq[2]);
        //     sum += wpos * positionGoalMatchingObjectiveTerm(currentPose, positionGoal) +
        //            wori * orientationGoalMatchingObjectiveTerm(qc, qg);

        //     Eigen::Vector3d pw(adjWristTrajectoryData[(npoint)][0], adjWristTrajectoryData[(npoint)][1], adjWristTrajectoryData[(npoint)][2]);
        //     Eigen::Vector3d pe(adjElbowTrajectoryData[(npoint)][0], adjElbowTrajectoryData[(npoint)][1], adjElbowTrajectoryData[(npoint)][2]);
        //     Eigen::Vector3d ps(adjShoulderTrajectoryData[(npoint)][0], adjShoulderTrajectoryData[(npoint)][1], adjShoulderTrajectoryData[(npoint)][2]);

        //     Eigen::Vector3d v_es = ps - pe;
        //     Eigen::Vector3d v_ew = pw - pe;

        //     // std::cout << "wrist desired:" << pw << std::endl;
        //     // std::cout << "elbow desired:" << pe << std::endl;
        //     // std::cout << "shoulder desired:" << ps << std::endl;

        //     Eigen::Vector3d normalArmPlaneGoal = v_es.cross(v_ew);
        //     KDL::Frame currentShoulderPose;
        //     // std::cout<<"q: "<<q(0)<<" "<<q(1)<<" "<<q(2)<<" "<<q(3)<<" "<<q(4)<<" "<<q(5)<<" "<<q(6)<<q(7)<<std::endl;
        //     (*fksolver).JntToCart(q, currentShoulderPose, 3);

        //     // Elbow pose
        //     KDL::Frame currentElbowPose;
        //     (*fksolver).JntToCart(q, currentElbowPose, 5);

        //     // Wrist pose
        //     KDL::Frame currentWristPose;
        //     (*fksolver).JntToCart(q, currentWristPose, 7);

        //     double currentAngleShoudlerElbow = shoulderElbowGoalObjectiveTerm(currentElbowPose, currentShoulderPose, v_es);
        //     double currentAngleElbowWrist = elbowWristGoalObjectiveTerm(currentElbowPose, currentWristPose, v_ew);
        //     sum += wes * currentAngleShoudlerElbow +
        //            wew * currentAngleElbowWrist;
        // }
        // std::cout << "Sum: " << sum << std::endl;
        // for (int j = 0; j < NJoints; j++)
        // {
        //     for (int nparam = 0; nparam <= SPLINE_ORDER; nparam++)
        //         std::cout << "param(" << nparam << ", j=" << j << ")= " << x[nparam + (SPLINE_ORDER + 1) * j];
        //     std::cout << std::endl;
        // }
        // // std::cout << "Current Pose: " << currentPose.p.x() << " " << currentPose.p.y() << " " << currentPose.p.z() << " ";
        // // Here we will need to change the index from when we gathered the desired positions
        // double auxq[4];
        // currentPose.M.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

        // // std::cout << "Quat: " << auxq[0] << " " << auxq[1] << " " << auxq[2] << " " << auxq[3] << std::endl;
        // //             KDL::Rotation rotKdl1 = KDL::Rotation::Quaternion(auxq[0], auxq[1], auxq[2], auxq[3]);

        // Eigen::Quaternionf qc(auxq[3], auxq[0], auxq[1], auxq[2]);

        // double norm = sqrt((*voidToVector)[(indexDiscrete)*7 + 3] * (*voidToVector)[(indexDiscrete)*7 + 3] + (*voidToVector)[(indexDiscrete)*7 + 4] * (*voidToVector)[(indexDiscrete)*7 + 4] + (*voidToVector)[(indexDiscrete)*7 + 5] * (*voidToVector)[(indexDiscrete)*7 + 5] + (*voidToVector)[(indexDiscrete)*7 + 6] * (*voidToVector)[(indexDiscrete)*7 + 6]);
        // KDL::Rotation rotKdl = KDL::Rotation::Quaternion((*voidToVector)[(indexDiscrete)*7 + 3] / norm, (*voidToVector)[(indexDiscrete)*7 + 4] / norm, (*voidToVector)[(indexDiscrete)*7 + 5] / norm, (*voidToVector)[(indexDiscrete)*7 + 6] / norm);
        // rotKdl = rotKdl * KDL::Rotation::RotX(-KDL::PI / 2.0);
        // rotKdl = rotKdl * KDL::Rotation::RotZ(KDL::PI / 4.0);
        // //             // rotKdl = rotKdl * KDL::Rotation::RotX(KDL::PI/2.0);
        // //             // rotKdl = rotKdl * KDL::Rotation::RotY(-KDL::PI / 2.0);
        // //             // rotKdl = rotKdl*KDL::Rotation::RotX(-KDL::PI / 4.0);
        // //             // rotKdl = rotKdl*KDL::Rotation::RotY(-KDL::PI / 5.0);
        // //             // rotKdl = rotKdl*KDL::Rotation::RotZ(-KDL::PI / 10.0);

        // rotKdl.GetQuaternion(auxq[0], auxq[1], auxq[2], auxq[3]);
        // Eigen::Quaternionf qg(auxq[3], auxq[0], auxq[1], auxq[2]);

        // double positionGoal[3] = {(*voidToVector)[(indexDiscrete)*7], (*voidToVector)[(indexDiscrete)*7 + 1], (*voidToVector)[(indexDiscrete)*7 + 2]};
        // //             std::cout << "wori: " << wori << std::endl;

        // sum += wpos * positionGoalMatchingObjectiveTerm(currentPose, positionGoal) +
        //        wori * orientationGoalMatchingObjectiveTerm(qc, qg) +
        //        wjv * jointVelocityObjectiveTerm(x, velocity, dt, 3) +
        //        wja * jointAccelerationObjectiveTerm(x, acceleration, dt, 3) +
        //        wjj * jointJerkObjectiveTerm(x, acceleration, dt, 3) +
        //        wv * endEffectorVelocityObjectiveTerm(currentPose, prevPose, dt, 3);

        // //                 //    wori * grooveLoss(orientationGoalMatchingObjectiveTerm(qc, qg), 1, 0, 0.1, 10.0) +
        // //                 //    wv * grooveLoss(endEffectorVelocityObjectiveTerm(currentPose, prevPose, dt, i), 1, 0, 0.1, 10.0) +
        // //                 //    wjj * grooveLoss(jointJerkObjectiveTerm(acceleration, jerk, dt, i), 1, 0, 0.1, 10.0) +
        // //                 //    wsc * grooveLoss(selfCollisionObjectiveTerm(q), 1, 0.0, 0.1, 10.0);
        // // #ifdef NOTONLYHAND

        // // Eigen::Vector3d normalArmPlaneGoal = v_es.cross(v_ew);

        // //             // sum += wap*normalArmPlaneGoalObjectiveTerm(currentWristPose, currentElbowPose, currentWristPose, normalArmPlaneGoal);
        // //             // std::cout<<"arm plane: "<<  wap*normalArmPlaneGoalObjectiveTerm(currentWristPose, currentElbowPose, currentShoulderPose, normalArmPlaneGoal)<<std::endl;
        // // #endif
        // prevPose = currentPose;

        // std::cout << "opt: " << sum << std::endl;
        return sum;
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

    // void jointVelocityLimit(unsigned m, double *result, unsigned n, const double *x, double *grad, void *data)
    // {
    //     std::cout << "constraint" << std::endl;
    //     auto *voidToVector = reinterpret_cast<std::vector<double> *>(data);

    //     for (unsigned int i = 0; i < NJoints; i++)
    //     {
    //         std::cout << x[i] << endl;
    //         std::cout << (*voidToVector)[i] << endl;
    //         result[i] = fabs(x[i] - (*voidToVector)[i]) / TIME_STEP - MAX_JVEL;
    //         std::cout << "result: " << result[i];
    //         // std::cout << "limit: " << fabs(x[i - NJoints] - x[i]) / TIME_STEP;
    //     }
    // }

    void qPositionMinLimit(unsigned m, double *result, unsigned n, const double *x, double *grad, void *data)
    {
        std::cout << "constraint" << std::endl;
        // GET THE MAXIMUM IN THE TRAJECTORY FOR EACH JOINT
        //  double maxQ[NJoints];
        double minQ[NJoints] = {MAXFLOAT, MAXFLOAT, MAXFLOAT, MAXFLOAT, MAXFLOAT, MAXFLOAT, MAXFLOAT, MAXFLOAT};
        double maxQ[NJoints] = {-MAXFLOAT, -MAXFLOAT, -MAXFLOAT, -MAXFLOAT, -MAXFLOAT, -MAXFLOAT, -MAXFLOAT, -MAXFLOAT};

        for (unsigned int j = 0; j < NJoints; j++)
        {
            for (int npoint = 0; npoint <= NPOINTS; npoint = npoint + 2)
            {
                double t = npoint / NPOINTS;
                // std::cout<<"npoint: "<<npoint<<"t: "<<t<<std::endl;

                double aux_q = 0;
                for (unsigned int i = 0; i <= SPLINE_ORDER; i++)
                {
                    aux_q += x[i + (SPLINE_ORDER + 1) * j] * pow(t, i);
                }
                if (aux_q < minQ[j])
                {
                    minQ[j] = aux_q;
                }
                if (aux_q > maxQ[j])
                    maxQ[j] = aux_q;
                // cout<<"q[joint="<<j<<"]: "<<q(j)<<std::endl;
            }

            std::cout << "minQ: " << minQ[j] << endl;
        }

        for (unsigned int i = 0; i < NJoints; i++)
        {
            result[i] = qmin(i) - minQ[i];
            result[i + NJoints] = maxQ[i] - qmax(i);
            std::cout << "joint: " << i << " MinQ: " << minQ[i] << "qmin: " << qmin(i) << std::endl;
            std::cout << "joint: " << i << " MaxQ: " << maxQ[i] << "qmin: " << qmax(i) << std::endl;
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
    double computeConditionNumber(const KDL::JntArray &q_in, KDL::Jacobian &jac)
    {
        int error = jntToJacSolver->JntToJac(q_in, jac);
        // std::cout<<"error: "<<error<<std::endl;
        Eigen::Matrix<double, 6, -1> H;
        Eigen::JacobiSVD<Eigen::Matrix<double, 6, -1>> svd(jac.data);
        // std::cout<<"Singular Values: "<<svd.singularValues();

        double conditionNumber = svd.singularValues().coeff(5) / svd.singularValues().coeff(0);
        // std::cout<<"condition Number: "<<conditionNumber<<std::endl;
        return conditionNumber;
    }

    double kinematicSingularityConstraint(unsigned n, const double *x, double *grad, void *data)
    {
        KDL::Jacobian jac(8);
        KDL::JntArray q(NJoints); // rotKdl = rotKdl * KDL::Rotation::RotX(-KDL::PI / 2.0);
                                  // rotKdl = rotKdl * KDL::Rotation::RotZ(KDL::PI / 4.0);
        // return 0.0325029-computeConditionNumber(q, jac);
        return 0.05 - computeConditionNumber(q, jac);
    }
}

int main()
{
    makeQLimitsTeoTrunkAndRightArmKinematics(qmin, qmax);
    KDL::Chain chain = makeTeoTrunkAndRightArmKinematicsFromDH();
    std::cout << "chain: " << chain.segments.size() << std::endl;
    fksolver = new KDL::ChainFkSolverPos_recursive(chain);
    jntToJacSolver = new KDL::ChainJntToJacSolver(chain);

    int nDemo = 1;
    string csvFile = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + to_string(nDemo) + "-smoothed.csv";
    vector<array<double, 7>> desiredTrajectoryData;
    vector<array<double, 7>> wristTrajectoryData;
    vector<array<double, 7>> elbowTrajectoryData;
    vector<array<double, 7>> shoulderTrajectoryData;
    vector<array<double, 7>> neckTrajectoryData;
    vector<array<double, 7>> hipTrajectoryData;
    HumanMotionData::getHumanData(csvFile, desiredTrajectoryData, wristTrajectoryData, elbowTrajectoryData, shoulderTrajectoryData, neckTrajectoryData, hipTrajectoryData);
    // HumanMotionData::printHumanData(desiredTrajectoryData, wristTrajectoryData, elbowTrajectoryData, shoulderTrajectoryData, neckTrajectoryData, hipTrajectoryData);
    // HumanMotionData::getHumanTrajectoryPoses(desiredTrajectoryData, wristTrajectoryData, elbowTrajectoryData, shoulderTrajectoryData, neckTrajectoryData, hipTrajectoryData, 0.001);

    HumanMotionData::linkLengthAdjustementPoses(desiredTrajectoryData, wristTrajectoryData, elbowTrajectoryData, shoulderTrajectoryData, neckTrajectoryData, hipTrajectoryData,
                                                adjHipTrajectoryData, adjNeckTrajectoryData, adjShoulderTrajectoryData, adjElbowTrajectoryData, adjWristTrajectoryData, adjHandTrajectoryData);

    string csvLinkAdjFile = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + to_string(nDemo) + "-smoothed-link-adj.csv";

    HumanMotionData::writePosesCsv(csvLinkAdjFile, adjHandTrajectoryData, adjWristTrajectoryData, adjElbowTrajectoryData, adjShoulderTrajectoryData, adjNeckTrajectoryData, adjHipTrajectoryData);

    int spline_order = SPLINE_ORDER;
    double x[(spline_order + 1) * NJoints * NSPLINES];

    double lb[(spline_order + 1) * NJoints * NSPLINES];
    double ub[(spline_order + 1) * NJoints * NSPLINES];

    // This is for the first spline
    for (int j = 0; j < NJoints; j++)
    {
        for (int i = 0; i <= spline_order; i++)
        {
            lb[int(i + (SPLINE_ORDER + 1) * j)] = (qmin(j)) + 0.001;
            ub[i + (SPLINE_ORDER + 1) * j] = (qmax(j)) - 0.001;
            if (i == 1)
            {
                lb[int(i + (SPLINE_ORDER + 1) * j)] = -0.000001;
                ub[i + (SPLINE_ORDER + 1) * j] = 0.0000001;
                x[i + (SPLINE_ORDER + 1) * j] = 0;
            }
            else if (i == 2)
            {
                lb[int(i + (SPLINE_ORDER + 1) * j)] = -0.0000001;
                ub[i + (SPLINE_ORDER + 1) * j] = 0.00000001;
                x[i + (SPLINE_ORDER + 1) * j] = 0;
            }
            else if (i == 3)
            {
                ub[i + (SPLINE_ORDER + 1) * j] = MAX_JACC;
                lb[i + (SPLINE_ORDER + 1) * j] = -MAX_JACC;
                while (x[i + (SPLINE_ORDER + 1) * j] == 0)
                {
                    x[i + (SPLINE_ORDER + 1) * j] = fRand(lb[int(i + (SPLINE_ORDER + 1) * j)], ub[i + (SPLINE_ORDER + 1) * j]);
                }
            }
            else
            {
                while (x[i + (SPLINE_ORDER + 1) * j] == 0)
                {
                    // ub[i + (SPLINE_ORDER + 1) * j] = MAX_JACC;
                    // lb[i + (SPLINE_ORDER + 1) * j] = -MAX_JACC;
                    x[i + (SPLINE_ORDER + 1) * j] = fRand(lb[int(i + (SPLINE_ORDER + 1) * j)], ub[i + (SPLINE_ORDER + 1) * j]);
                }
            }
        }
    }
    // if (NSPLINES > 1)
    // {
    //     // Check the second spline
    //     for (int j = 0; j < NJoints; j++)
    //     {
    //         for (int i = 0; i <= spline_order; i++)
    //         {
    //             lb[(spline_order + 1) * NJoints * (NSPLINES - 1) + (i + (SPLINE_ORDER + 1) * j)] = (qmin(j)) + 0.001;
    //             ub[(spline_order + 1) * NJoints * (NSPLINES - 1) + (i + (SPLINE_ORDER + 1) * j)] = (qmax(j)) - 0.001;
    //             if (i == 0)
    //             {
    //                 x[(spline_order + 1) * NJoints * (NSPLINES - 1) + (i + (SPLINE_ORDER + 1) * j)] = 0;
    //                 for (int prev_i = 0; prev_i <= spline_order; prev_i++)
    //                     x[(spline_order + 1) * NJoints * (NSPLINES - 1) + (i + (SPLINE_ORDER + 1) * j)] += x[prev_i + (SPLINE_ORDER + 1) * j];
    //                 std::cout << "a0: " << x[(spline_order + 1) * NJoints * (NSPLINES - 1) + (i + (SPLINE_ORDER + 1) * j)] << endl;
    //             }
    //             if (i == 1)
    //             {
    //                 lb[(spline_order + 1) * NJoints * (NSPLINES - 1) + (i + (SPLINE_ORDER + 1) * j)] = -MAX_JVEL;
    //                 ub[(spline_order + 1) * NJoints * (NSPLINES - 1) + (i + (SPLINE_ORDER + 1) * j)] = MAX_JVEL;
    //                 // x[(spline_order + 1) * NJoints*(NSPLINES-1)+(i + (SPLINE_ORDER + 1) * j)] =fRand(lb[(spline_order + 1) * NJoints*(NSPLINES-1)+(i + (SPLINE_ORDER + 1) * j)],  ub[(spline_order + 1) * NJoints*(NSPLINES-1)+(i + (SPLINE_ORDER + 1) * j)]);
    //                 x[(spline_order + 1) * NJoints * (NSPLINES - 1) + (i + (SPLINE_ORDER + 1) * j)] = 0;
    //                 for (int prev_i = 1; prev_i <= spline_order; prev_i++)
    //                     x[(spline_order + 1) * NJoints * (NSPLINES - 1) + (i + (SPLINE_ORDER + 1) * j)] += prev_i * x[prev_i + (SPLINE_ORDER + 1) * j];
    //                 std::cout << "a1: " << x[(spline_order + 1) * NJoints * (NSPLINES - 1) + (i + (SPLINE_ORDER + 1) * j)] << endl;
    //             }
    //             else if (i == 2)
    //             {
    //                 lb[(spline_order + 1) * NJoints * (NSPLINES - 1) + (i + (SPLINE_ORDER + 1) * j)] = -MAX_JACC;
    //                 ub[(spline_order + 1) * NJoints * (NSPLINES - 1) + (i + (SPLINE_ORDER + 1) * j)] = MAX_JACC;
    //                 for (int prev_i = 2; prev_i <= spline_order; prev_i++)
    //                     x[(spline_order + 1) * NJoints * (NSPLINES - 1) + (i + (SPLINE_ORDER + 1) * j)] += (prev_i - 1) * prev_i * x[prev_i + (SPLINE_ORDER + 1) * j];
    //                 x[(spline_order + 1) * NJoints * (NSPLINES - 1) + (i + (SPLINE_ORDER + 1) * j)] /= 2.0;
    //             }
    //             else if (i == 3)
    //             {
    //                 ub[i + (SPLINE_ORDER + 1) * j] = MAX_JACC;
    //                 lb[i + (SPLINE_ORDER + 1) * j] = -MAX_JACC;
    //                 for (int prev_i = 3; prev_i <= spline_order; prev_i++)
    //                     x[(spline_order + 1) * NJoints * (NSPLINES - 1) + (i + (SPLINE_ORDER + 1) * j)] += (prev_i - 2) * (prev_i - 1) * prev_i * x[prev_i + (SPLINE_ORDER + 1) * j];
    //                 x[(spline_order + 1) * NJoints * (NSPLINES - 1) + (i + (SPLINE_ORDER + 1) * j)] /= 6.0;
    //             }
    //             else
    //             {
    //                 while (x[i + (SPLINE_ORDER + 1) * j] == 0)
    //                 {
    //                     // ub[i + (SPLINE_ORDER + 1) * j] = MAX_JACC;
    //                     // lb[i + (SPLINE_ORDER + 1) * j] = -MAX_JACC;
    //                     x[i + (SPLINE_ORDER + 1) * j] = fRand(lb[int(i + (SPLINE_ORDER + 1) * j)], ub[i + (SPLINE_ORDER + 1) * j]);
    //                 }
    //             }
    //         }
    //     }
    //     // Lets check that is well initialized
    //     // first spline t = 1

    //     double t_end = 1;
    //     double t_init = 0;

    //     int current_spline = 2;
    //     for (unsigned int j = 0; j < NJoints; j++)
    //     {
    //         double aux_q = 0;
    //         double start_q = 0;
    //         double v_end = 0;
    //         double v_start = 0;
    //         double a_end = 0;
    //         double a_start = 0;
    //         double j_end = 0;
    //         double j_start = 0;
    //         for (unsigned int i = 0; i <= SPLINE_ORDER; i++)
    //         {
    //             aux_q += x[i + (SPLINE_ORDER + 1) * j] * pow(t_end, i);
    //             start_q += x[(spline_order + 1) * NJoints * (current_spline - 1) + (i + (SPLINE_ORDER + 1) * j)] * pow(t_init, i);
    //             if (i > 0)
    //             {
    //                 v_end += i * x[i + (SPLINE_ORDER + 1) * j] * pow(t_end, i - 1);
    //                 v_start += i * x[(spline_order + 1) * NJoints * (current_spline - 1) + (i + (SPLINE_ORDER + 1) * j)] * pow(t_init, i - 1);
    //             }
    //             if (i > 1)
    //             {
    //                 a_end += (i - 1) * i * x[i + (SPLINE_ORDER + 1) * j] * pow(t_end, i - 2);
    //                 a_start += (i - 1) * i * x[(spline_order + 1) * NJoints * (current_spline - 1) + (i + (SPLINE_ORDER + 1) * j)] * pow(t_init, i - 2);
    //             }
    //             if (i > 2)
    //             {
    //                 j_end += (i - 2) * (i - 1) * i * x[i + (SPLINE_ORDER + 1) * j] * pow(t_end, i - 3);
    //                 j_start += (i - 2) * (i - 1) * i * x[(spline_order + 1) * NJoints * (current_spline - 1) + (i + (SPLINE_ORDER + 1) * j)] * pow(t_init, i - 3);
    //             }
    //         }

    //         cout << "q[t=1,joint=" << j << "]: " << aux_q << std::endl;
    //         cout << "q[t=0,joint=" << j << "]: " << start_q << std::endl;
    //         cout << "v[t=1,joint=" << j << "]: " << v_end << std::endl;
    //         cout << "v[t=0,joint=" << j << "]: " << v_start << std::endl;
    //         cout << "a[t=1,joint=" << j << "]: " << a_end << std::endl;
    //         cout << "a[t=0,joint=" << j << "]: " << a_start << std::endl;
    //         cout << "j[t=1,joint=" << j << "]: " << j_end << std::endl;
    //         cout << "j[t=0,joint=" << j << "]: " << j_start << std::endl;
    //     }
    // }
    int j = 0;
    while (j < NJoints)
    {
        double minQ = MAXFLOAT;
        for (int npoint = 0; npoint <= NPOINTS; npoint = npoint + 2)
        {
            double t = npoint / NPOINTS;
            // std::cout<<"npoint: "<<npoint<<"t: "<<t<<std::endl;

            double aux_q = 0;
            for (unsigned int i = 0; i <= SPLINE_ORDER; i++)
            {
                aux_q += x[i + (SPLINE_ORDER + 1) * j] * pow(t, i);
            }
            if (aux_q < minQ)
            {
                minQ = aux_q;
            }
            // cout<<"q[joint="<<j<<"]: "<<q(j)<<std::endl;
        }
        std::cout << minQ << " " << qmin(j) << endl;
        if (minQ < qmin(j))
        {
            int i = 0;
            x[i + (SPLINE_ORDER + 1) * j] = fRand(lb[int(i + (SPLINE_ORDER + 1) * j)], ub[i + (SPLINE_ORDER + 1) * j]);
            i = 3;
            x[i + (SPLINE_ORDER + 1) * j] = fRand(lb[int(i + (SPLINE_ORDER + 1) * j)], ub[i + (SPLINE_ORDER + 1) * j]);
            i = 4;
            x[i + (SPLINE_ORDER + 1) * j] = fRand(lb[int(i + (SPLINE_ORDER + 1) * j)], ub[i + (SPLINE_ORDER + 1) * j]);
            std::cout << minQ << std::endl;
        }
        else
        {
            j++;
        }
    }

    double minf = MAXFLOAT;

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

    // Suppose that we are using the first 100 points.

    nlopt_opt opt;
    opt = nlopt_create(NLOPT_LN_COBYLA, (spline_order + 1) * NJoints);
    double tol[NJoints];
    for (unsigned int i = 0; i < (spline_order + 1) * NJoints; i++)
        tol[i] = 1e-6;

    // std::vector<double> jprev;
    // for (unsigned int i = 0; i < NJoints; i++)
    //     jprev.push_back(xprev[2][i]);

    // void *data = &jprev;
    // nlopt_add_inequality_mconstraint(opt_obo, NJoints, sharon::jointVelocityLimit, data, tol_obo);
    for (int j = 0; j < NJoints; j++)
    {
        for (int nparam = 0; nparam <= SPLINE_ORDER; nparam++)
            std::cout << "param(" << nparam << ", j=" << j << ")= " << x[nparam + (spline_order + 1) * j] << " ";
        std::cout << std::endl;
    }
    nlopt_add_inequality_mconstraint(opt, 2 * NJoints, sharon::qPositionMinLimit, NULL, tol);
    nlopt_set_min_objective(opt, sharon::optimizationFunctionJoints, parameters);
    nlopt_set_lower_bounds(opt, lb);
    nlopt_set_upper_bounds(opt, ub);
    nlopt_result result = nlopt_optimize(opt, x, &minf);
    std::cout << "found minimum at f(x)=" << minf << std::endl;

    for (int j = 0; j < NJoints; j++)
    {
        for (int nparam = 0; nparam <= SPLINE_ORDER; nparam++)
            std::cout << "param(" << nparam << ", j=" << j << ")= " << x[nparam + (spline_order + 1) * j] << " ";
        std::cout << std::endl;
    }
    std::vector<std::array<double, NJoints>> qTraj;
    for (int npoint = 0; npoint <= NPOINTS; npoint = npoint + 1)
    {
        std::array<double, NJoints> qArray;
        KDL::JntArray q(NJoints);
        double t = (float)npoint / float(NPOINTS);
        for (unsigned int joint = 0; joint < NJoints; joint++)
        {
            double aux_q = 0;
            for (int i = 0; i <= spline_order; i++)
            {
                aux_q += x[i + (SPLINE_ORDER + 1) * joint] * pow(t, i);
            }
            qArray[joint] = aux_q;
            cout << "q[joint=" << joint << "]: " << qArray[joint] << std::endl;
            q(joint) = qArray[joint];
            qArray[joint] = qArray[joint] * KDL::rad2deg;
        }
        qTraj.push_back(qArray);

        KDL::Frame currentPose;
        int foundfk = (*fksolver).JntToCart(q, currentPose);
        double quat[4];
        currentPose.M.GetQuaternion(quat[0], quat[1], quat[2], quat[3]);
        std::array<double, 7> pose;
        pose[0] = currentPose.p.x();
        pose[1] = currentPose.p.y();
        pose[2] = currentPose.p.z();
        std::cout << "x: " << desiredDiscretePoses[(npoint)*7] << " y: " << desiredDiscretePoses[(npoint)*7 + 1] << " z: " << desiredDiscretePoses[(npoint)*7 + 2] << std::endl;
        std::cout << "cx: " << currentPose.p.x() << " cy: " << currentPose.p.y() << " cz: " << currentPose.p.z() << std::endl;
        pose[3] = quat[0];
        pose[4] = quat[1];
        pose[5] = quat[2];
        pose[6] = quat[3];
        poses.push_back(pose);

        currentPose.M.GetQuaternion(quat[0], quat[1], quat[2], quat[3]);

        double norm = sqrt((desiredDiscretePoses)[(npoint)*7 + 3] * (desiredDiscretePoses)[(npoint)*7 + 3] + (desiredDiscretePoses)[(npoint)*7 + 4] * (desiredDiscretePoses)[(npoint)*7 + 4] + (desiredDiscretePoses)[(npoint)*7 + 5] * (desiredDiscretePoses)[(npoint)*7 + 5] + (desiredDiscretePoses)[(npoint)*7 + 6] * (desiredDiscretePoses)[(npoint)*7 + 6]);
        KDL::Rotation rotKdl = KDL::Rotation::Quaternion((desiredDiscretePoses)[(npoint)*7 + 3] / norm, (desiredDiscretePoses)[(npoint)*7 + 4] / norm, (desiredDiscretePoses)[(npoint)*7 + 5] / norm, (desiredDiscretePoses)[(npoint)*7 + 6] / norm);
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
        double positionGoal[3] = {(desiredDiscretePoses)[(npoint)*7], (desiredDiscretePoses)[(npoint)*7 + 1], (desiredDiscretePoses)[(npoint)*7 + 2]};
        double dist = sqrt((currentPose.p.x() - positionGoal[0]) * (currentPose.p.x() - positionGoal[0]) +
                           (currentPose.p.y() - positionGoal[1]) * (currentPose.p.y() - positionGoal[1]) +
                           (currentPose.p.z() - positionGoal[2]) * (currentPose.p.z() - positionGoal[2]));
        positiondist.push_back(dist);
        double inner_product = qcurrent.x() * qg.x() + qcurrent.y() * qg.y() + qcurrent.z() * qg.z() + qcurrent.w() * qg.w();
        double angle = acos(2 * inner_product * inner_product - 1);

        orientatationdist.push_back(angle);

        Eigen::Vector3d goal_pw(adjWristTrajectoryData[(npoint)][0], adjWristTrajectoryData[(npoint)][1], adjWristTrajectoryData[(npoint)][2]);
        Eigen::Vector3d goal_pe(adjElbowTrajectoryData[(npoint)][0], adjElbowTrajectoryData[(npoint)][1], adjElbowTrajectoryData[(npoint)][2]);
        Eigen::Vector3d goal_ps(adjShoulderTrajectoryData[(npoint)][0], adjShoulderTrajectoryData[(npoint)][1], adjShoulderTrajectoryData[(npoint)][2]);

        Eigen::Vector3d goalShoulderElbow = goal_ps - goal_pe;
        Eigen::Vector3d goalElbowWrist = goal_pw - goal_pe;

        // std::cout << "wrist desired:" << pw << std::endl;
        // std::cout << "elbow desired:" << pe << std::endl;
        // std::cout << "shoulder desired:" << ps << std::endl;

        KDL::Frame currentShoulderPose;
        // std::cout<<"q: "<<q(0)<<" "<<q(1)<<" "<<q(2)<<" "<<q(3)<<" "<<q(4)<<" "<<q(5)<<" "<<q(6)<<q(7)<<std::endl;
        (*fksolver).JntToCart(q, currentShoulderPose, 3);

        // Elbow pose
        KDL::Frame currentElbowPose;
        (*fksolver).JntToCart(q, currentElbowPose, 5);

        // Wrist pose
        KDL::Frame currentWristPose;
        (*fksolver).JntToCart(q, currentWristPose, 7);

        Eigen::Vector3d pe(currentElbowPose.p.x(), currentElbowPose.p.y(), currentElbowPose.p.z());
        // std::cout<<pe<<std::endl;
        Eigen::Vector3d ps(currentShoulderPose.p.x(), currentShoulderPose.p.y(), currentShoulderPose.p.z());
        // std::cout<<ps<<std::endl;
        Eigen::Vector3d v_es = ps - pe;
        // std::cout<<"current shoulder elbow: "<<v_es<<std::endl;
        angleShoudlerElbow.push_back(acos(v_es.dot(goalShoulderElbow) / (goalShoulderElbow.norm() * v_es.norm())));

        Eigen::Vector3d pw(currentWristPose.p.x(), currentWristPose.p.y(), currentWristPose.p.z());
        Eigen::Vector3d v_ew = pw - pe;
        // std::cout<<"goalElbowWrist: "<<goalElbowWrist<<std::endl;
        // std::cout<<"current ElbowWrist: "<<v_ew<<std::endl;
        angleElbowWrist.push_back(acos(v_ew.dot(goalElbowWrist) / (goalElbowWrist.norm() * v_ew.norm())));
    }

    std::string csvQFileWrite = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + to_string(nDemo) + "-q-spline-optimization.csv";
    sharon::writeQCsv(csvQFileWrite, qTraj);
    std::string csvPoseFileWrite = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + to_string(nDemo) + "-poses-spline-optimization.csv";
    sharon::writePoseCsv(csvPoseFileWrite, poses);

    std::string csvDataFileWrite = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + to_string(nDemo) + "-spline-data-optimization.csv";
    sharon::writeDataCsv(csvDataFileWrite, positiondist, orientatationdist, angleShoudlerElbow, angleElbowWrist);
    return 0;
}
