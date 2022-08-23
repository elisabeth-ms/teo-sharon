#include "MapHumanToRobotMotionPerFrameIK.hpp"

#define EPS 1e-4

int maxSteps;
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
    qmin(0) = -31.0* KDL::deg2rad;
    qmin(1) = -10.1* KDL::deg2rad;
    qmin(2) = -98.1* KDL::deg2rad;
    qmin(3) = -75.5* KDL::deg2rad;
    qmin(4) = -80.1* KDL::deg2rad;
    qmin(5) = -99.6* KDL::deg2rad;
    qmin(6) = -80.4* KDL::deg2rad;
    qmin(7) = -115.1* KDL::deg2rad;
    qmax(0) = 31.0* KDL::deg2rad;
    qmax(1) = 25.5* KDL::deg2rad;
    qmax(2) = 106.0* KDL::deg2rad;
    qmax(3) = 22.4* KDL::deg2rad;
    qmax(4) = 57.0* KDL::deg2rad;
    qmax(5) = 98.4* KDL::deg2rad;
    qmax(6) = 99.6* KDL::deg2rad;
    qmax(7) = 44.7* KDL::deg2rad;
}

void writeQCsv(std::string filename, const std::vector<std::array<double, 8>> &qTraj)
{
    std::ofstream myFile(filename);
    for (unsigned int i = 0; i < qTraj.size(); i++)
    {
        myFile << i << " " << std::setprecision(10) << qTraj[i][0] << " " << qTraj[i][1] << " " << qTraj[i][2] << " " << qTraj[i][3] << " " << qTraj[i][4]
                   << " " << qTraj[i][5] << " " << qTraj[i][6] << " " << qTraj[i][7] << "\n";
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





void MapHumanToRobotMotionPerFrameIK::getCurrentPoses(const KDL::JntArray &q, KDL::Frame &currentShoulderPose, KDL::Frame &currentElbowPose, KDL::Frame &currentWristPose, KDL::Frame &currentPose)
{
    (*_fksolver).JntToCart(q, currentPose);

    (*_fksolver).JntToCart(q,currentShoulderPose, 3);

    (*_fksolver).JntToCart(q, currentElbowPose, 5);

    (*_fksolver).JntToCart(q, currentWristPose, 7);

}

// double MapHumanToRobotMotionPerFrameIK::computeSwivelAngle(Eigen::Vector3d & shoulderPosition, Eigen::Vector3d & elbowPosition, Eigen::Vector3d & wristPosition){
//     Eigen::Vector3d v_es = shoulderPosition - elbowPosition;
//     Eigen::Vector3d v_ew = elbowPosition - wristPosition;
// //         std::cout<<"v_es: "<<v_es<<std::endl;
// //         std::cout<<"v_ew: "<<v_ew<<std::endl;

//     if(v_es.dot(v_ew)/(v_es.norm()*v_ew.norm()) == 1.0 || v_es.dot(v_ew)/(v_es.norm()*v_ew.norm())== -1.0){
//         // std::cout<<"Parallel vectors"<<std::endl;
//         // Just compute the angle between the vector that goes through the arm and the vector in the sagitall plane(x)
//         Eigen::Vector3d x_axes = {-1.0, 0.0, 0.0};
//         return angleBetweenVectors(v_es, x_axes);
//     }
//     else{

//         Eigen::Vector3d normalArmPlane = v_es.cross(v_ew);

//         Eigen::Vector3d normalToSagitalPlane = {0, -1.0, 0.0};

//         return angleBetweenVectors(normalArmPlane, normalToSagitalPlane);
//     }

// //         std::cout<<"normalCurrentArmPlane: "<<normalCurrentArmPlane<<std::endl;
// //         std::cout<<"normalArmPlaneGoal: "<<normalArmPlaneGoal<<std::endl;
// }


// double MapHumanToRobotMotionPerFrameIK::shoulderElbowAngleDistance(const KDL::Frame & currentElbowPose, const KDL::Frame & currentShoulderPose, const Eigen::Vector3d & goalShoulderElbow){

//     Eigen::Vector3d pe(currentElbowPose.p.x(), currentElbowPose.p.y(), currentElbowPose.p.z());
//         // std::cout<<pe<<std::endl;
//     Eigen::Vector3d ps(currentShoulderPose.p.x(), currentShoulderPose.p.y(), currentShoulderPose.p.z());
//         // std::cout<<ps<<std::endl;
//     Eigen::Vector3d v_es = ps - pe;
//     double n1 = v_es.norm();
//     double n2 = goalShoulderElbow.norm();
//     Eigen::Vector3d new_v_es(v_es.x()/n1, v_es.y()/n1, v_es.z()/n1);
//     Eigen::Vector3d new_goalShoulderElbow(goalShoulderElbow.x()/n2, goalShoulderElbow.y()/n2, goalShoulderElbow.z()/n2);
    
//     double fvalue;
//     if((new_v_es.x() == new_goalShoulderElbow.x()) && new_v_es.y() == new_goalShoulderElbow.y() && new_v_es.z() == new_goalShoulderElbow.z()){
//         fvalue = 0.0;
//     }
//     else{
//         double cosAngle = v_es.dot(goalShoulderElbow)/(goalShoulderElbow.norm()*v_es.norm());
//         fvalue = acos(cosAngle);
//     }
//     return fvalue;
// }


int MapHumanToRobotMotionPerFrameIK::ik(const KDL::JntArray &qInit, const KDL::Frame &fpose, KDL::JntArray &q){
    int foundfk = (*_iksolver).CartToJnt(qInit, fpose, q, _bounds);
    cout<<foundfk<<endl;
    cout<<"qInit: ";
    for(unsigned int j = 0; j<8; j++)
        cout<<qInit(j)<<" ";
    cout<<endl;
    cout<<"q: ";
    for(unsigned int j = 0; j<8; j++)
        cout<<q(j)<<" ";
    cout<<endl;
    double quat[4];
    fpose.M.GetQuaternion(quat[0],quat[1],quat[2],quat[3]);
    cout<<"fpose.p: ";
    for(unsigned int j = 0; j<3; j++)
        cout<<fpose.p[j]<<" ";
    cout<<"fpose.q: ";
    for(unsigned int j = 0; j<4; j++)
        cout<<quat[j]<<" ";
    cout<<endl;
    return foundfk;
    
}

int MapHumanToRobotMotionPerFrameIK::fk(const KDL::JntArray &q, KDL::Frame &fpose){
    _fksolver->JntToCart(q, fpose);
}

void MapHumanToRobotMotionPerFrameIK::handPosesToFPose(int index, KDL::Frame &fGoal){
    double norm = sqrt(_discreteHandPoses[index][3] * _discreteHandPoses[index][3] + _discreteHandPoses[index][4]*_discreteHandPoses[index][4] + _discreteHandPoses[index][5]*_discreteHandPoses[index][5] + _discreteHandPoses[index][6] * _discreteHandPoses[index][6]);
    KDL::Rotation rotKdl = KDL::Rotation::Quaternion(_discreteHandPoses[index][3] / norm, _discreteHandPoses[index][4] / norm, _discreteHandPoses[index][5] / norm, _discreteHandPoses[index][6] / norm);
    KDL::Vector posKdl = KDL::Vector(_discreteHandPoses[index][0], _discreteHandPoses[index][1], _discreteHandPoses[index][2]);
    // rotKdl = rotKdl * KDL::Rotation::RotX(-KDL::PI / 2.0);
    // rotKdl = rotKdl * KDL::Rotation::RotZ(- KDL::PI / 4.0);
    fGoal.M = rotKdl;
    fGoal.p = posKdl;

}

double MapHumanToRobotMotionPerFrameIK::angleBetweenVectors(Eigen::Vector3d a, Eigen::Vector3d b){
    double angle = 0.0;

    angle = acos(a.dot(b)/a.norm()*b.norm());

    return angle;
}




int main(int argc, char **argv)
{

    KDL::JntArray qmin(8), qmax(8);
    makeQLimitsTeoTrunkAndRightArmKinematics(qmin, qmax);
    KDL::Chain chain = makeTeoTrunkAndRightArmKinematicsFromDH();
    std::cout<<"chain: "<<chain.segments.size()<<std::endl;
    MapHumanToRobotMotionPerFrameIK mapHumanToRobotMotionPerFrameIK(chain, qmin, qmax, 8000, EPS);


    // double mean, stdDev;
    // mapHumanToRobotMotion.getMeanAndStdDevConditionNumber(mean, stdDev, 5000);

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------//
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
    
    vector<array<double, 7>> adjHandTrajectoryData;
    vector<array<double, 7>> adjWristTrajectoryData;
    vector<array<double, 7>> adjElbowTrajectoryData;
    vector<array<double, 7>> adjShoulderTrajectoryData;
    vector<array<double, 7>> adjNeckTrajectoryData;
    vector<array<double, 7>> adjHipTrajectoryData;
    HumanMotionData::linkLengthAdjustementPoses(desiredTrajectoryData,wristTrajectoryData,elbowTrajectoryData,shoulderTrajectoryData,neckTrajectoryData, hipTrajectoryData,
                                         adjHipTrajectoryData,adjNeckTrajectoryData,adjShoulderTrajectoryData,adjElbowTrajectoryData,adjWristTrajectoryData,adjHandTrajectoryData);

    string csvLinkAdjFile =  "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + to_string(nDemo) + "-smoothed-link-adj.csv";

    HumanMotionData::writePosesCsv(csvLinkAdjFile, adjHandTrajectoryData, adjWristTrajectoryData, adjElbowTrajectoryData, adjShoulderTrajectoryData, adjNeckTrajectoryData, adjHipTrajectoryData);
    KDL::JntArray  qInit(8);
    KDL::JntArray  q(8);
    for(unsigned int j = 0; j<8; j++){
        qInit(j) = fRand(qmin(j),qmax(j));
        // qInit(j) = 0.0;
    }
    std::vector<std::array<double, 8>> qTraj;

    for(int i=0; i<adjHandTrajectoryData.size(); i++){
        cout<<"i: "<<i<<endl;
        KDL::Rotation rotKdl = KDL::Rotation::Quaternion(adjHandTrajectoryData[i][3], adjHandTrajectoryData[i][4], adjHandTrajectoryData[i][5], adjHandTrajectoryData[i][6]);
        rotKdl = rotKdl * KDL::Rotation::RotX(-KDL::PI / 2.0);
        rotKdl = rotKdl * KDL::Rotation::RotZ(KDL::PI / 4.0);
        KDL::Frame fpose = KDL::Frame(rotKdl, KDL::Vector(adjHandTrajectoryData[i][0], adjHandTrajectoryData[i][1], adjHandTrajectoryData[i][2]));
        if(mapHumanToRobotMotionPerFrameIK.ik(qInit, fpose, q) >= 0)
        {
            printf("Solution found\n");
            qInit = q;
            std::array<double, 8> qArray = {q(0)*KDL::rad2deg, q(1)*KDL::rad2deg, q(2)*KDL::rad2deg, q(3)*KDL::rad2deg, q(4)*KDL::rad2deg, q(5)*KDL::rad2deg, q(6)*KDL::rad2deg, q(7)*KDL::rad2deg};
            std::cout<<"qArray: "<<qArray[0]<<" "<<qArray[1]<<" "<<qArray[2]<<" "<<qArray[3]<<" "<<qArray[4]<<" "<<qArray[5]<<" "<<qArray[6]<<" "<<qArray[7]<<std::endl;
            KDL::Frame fposeCheck;
            mapHumanToRobotMotionPerFrameIK.fk(q, fposeCheck);
            double qx,qy,qz,qw;
            fpose.M.GetQuaternion(qx,qy,qz,qw);
            std::cout<<"fpose: "<<fpose.p.x()<<" "<<fpose.p.y()<<" "<<fpose.p.z()<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<std::endl;
            fposeCheck.M.GetQuaternion(qx,qy,qz,qw);
            std::cout<<"fposeCheck: "<<fposeCheck.p.x()<<" "<<fposeCheck.p.y()<<" "<<fposeCheck.p.z()<<" "<<qx<<" "<<qy<<" "<<qz<<" "<<qw<<std::endl;
            qTraj.push_back(qArray);
        }
        else{
            cout<<"Solution NOT FOUND"<<endl;
        }
    }
    cout<<qTraj.size()<<endl;
    string csvFileIK = "/home/elisabeth/repos/teo-sharon/programs/MapHumanToRobotMotionPerFrameIK/trajectories/prueba" + to_string(nDemo) + "-smoothed-ik-joints.csv";
    HumanMotionData::writeQCsv(csvFileIK, qTraj);

//     std::vector<float> angleEW;
//     for(unsigned int i = 0; i<listMinCostPath.size(); i++){
//         xmin.push_back(listMinCostPath[i].currentPosition[0]);
//         ymin.push_back(listMinCostPath[i].currentPosition[1]);
//         zmin.push_back(listMinCostPath[i].currentPosition[2]);

//         q0.push_back(listMinCostPath[i].q[0]);
//         q1.push_back(listMinCostPath[i].q[1]);
//         q2.push_back(listMinCostPath[i].q[2]);
//         q3.push_back(listMinCostPath[i].q[3]);
//         q4.push_back(listMinCostPath[i].q[4]);
//         q5.push_back(listMinCostPath[i].q[5]);       
//         q6.push_back(listMinCostPath[i].q[6]);        
//         q7.push_back(listMinCostPath[i].q[7]);
//         swivelAngle.push_back(listMinCostPath[i].swivelAngle);
//         goalSwivelAngle.push_back(listMinCostPath[i].goalSwivelAngle);
//         // angleSE.push_back(listMinCostPath[i].angleShoulderElbow);
//         // angleEW.push_back(listMinCostPath[i].angleElbowWrist);                
//         indexes.push_back(i);
//     }
//     for(unsigned int i = 0; i<desiredTrajectoryData.size(); i++){

//         x.push_back(desiredTrajectoryData[i][1]);//mapHumanToRobotMotion.getDiscreteHandPoses()[i][0]);
//         y.push_back(desiredTrajectoryData[i][2]);
//         z.push_back(desiredTrajectoryData[i][3]-TrunkHeight);

//     }

//     // for(unsigned int i = 0; i<maxSteps; i++){

//     //     xmin.push_back(mapHumanToRobotMotion.getDiscreteHandPoses()[i][0]);
//     //     ymin.push_back(mapHumanToRobotMotion.getDiscreteHandPoses()[i][1]);
//     //     zmin.push_back(mapHumanToRobotMotion.getDiscreteHandPoses()[i][2]+TrunkHeight);

//     // }
//     std::map<std::string, std::string> keywords;
//     keywords.insert(std::pair<std::string, std::string>("label", "min cost") );

//     std::map<std::string, std::string> keywordsgoal;
//     keywordsgoal.insert(std::pair<std::string, std::string>("label", "goal position") );
//     plt::plot3(x, y, z,1, keywordsgoal);
//     plt::plot3(xmin, ymin, zmin, 1, keywords);
//     plt::legend();
//     plt::show();

//     plt::figure_size(1200, 780);
//     plt::subplot(4, 2, 1); // 2 rows, 4 column, first plot
//     plt::ylim(qmin(0), qmax(0));
//     plt::plot(indexes, q0);
//     plt::subplot(4, 2, 2); // 2 rows, 4 column, second plot
//     plt::ylim(qmin(1), qmax(1));
//     plt::plot(indexes, q1);
//     plt::subplot(4, 2, 3); // 2 rows, 4 column, second plot
//     plt::ylim(qmin(2), qmax(2));
//     plt::plot(indexes, q2);
//     plt::subplot(4, 2, 4); // 2 rows, 4 column, first plot
//     plt::ylim(qmin(3), qmax(3));
//     plt::plot(indexes, q3);
//     plt::subplot(4, 2, 5); // 2 rows, 4 column, second plot
//     plt::ylim(qmin(4), qmax(4));
//     plt::plot(indexes, q4);
//     plt::subplot(4, 2, 6); // 2 rows, 4 column, second plot
//     plt::ylim(qmin(5), qmax(5));
//     plt::plot(indexes, q5);
//     plt::subplot(4, 2, 7); // 2 rows, 4 column, second plot
//     plt::ylim(qmin(6), qmax(6));
//     plt::plot(indexes, q6);
//     plt::subplot(4, 2, 8); // 2 rows, 4 column, second plot
//     plt::ylim(qmin(7), qmax(7));
//     plt::plot(indexes, q7);
//     plt::show();

//     plt::figure_size(1200, 780);
//     // plt::subplot(1, 2, 1); // 2 rows, 4 column, second plot
//     std::map<std::string, std::string> keywordsSwivelAngle;
//     keywordsSwivelAngle.insert(std::pair<std::string, std::string>("label", "Swivel Angle") );
//     plt::plot(indexes, swivelAngle, keywordsSwivelAngle);
//     std::map<std::string, std::string> keywordsGoalSwivelAngle;
//     keywordsGoalSwivelAngle.insert(std::pair<std::string, std::string>("label", "Goal Swivel Angle") );
//     plt::plot(indexes, goalSwivelAngle, keywordsGoalSwivelAngle);
//     plt::legend();

//     // plt::subplot(1, 2, 2); // 2 rows, 4 column, second plot
//     // plt::plot(indexes, angleEW);
//     plt::show();



//     std::vector<std::array<double, NJoints>> qTraj;
//     std::array<double, NJoints> qArray;
//     KDL::JntArray q(NJoints); 
//     std::vector<std::array<double, 7>> poses;

//     for(unsigned int i=0; i<listMinCostPath.size(); i++){
//         for(unsigned int j=0; j<NJoints; j++){
//             qArray[j] = listMinCostPath[i].q[j]*KDL::rad2deg;
//             q(j) = listMinCostPath[i].q[j];
//         }
//         KDL::Frame currentPose;
//         mapHumanToRobotMotion.getFKSolver().JntToCart(q, currentPose);
//         double quat[4];
//         currentPose.M.GetQuaternion(quat[0], quat[1], quat[2], quat[3]);
//         std::array<double, 7> pose;
//         pose[0] = currentPose.p.x();
//         pose[1] = currentPose.p.y();
//         pose[2] = currentPose.p.z()+TrunkHeight;
//         pose[3] = quat[0];
//         pose[4] = quat[1];
//         pose[5] = quat[2];
//         pose[6] = quat[3];
//         poses.push_back(pose);
//         qTraj.push_back(qArray);
//     }
//     // std::reverse(qTraj.begin(), qTraj.end());
//     // std::reverse(poses.begin(), poses.end());
//     string resultsCsvFile = "/home/elisabeth/repos/teo-sharon/programs/GenerateManipulationTrajectories/trajectories/shoulderElbowWirstHandTest/prueba" + to_string(nDemo) + "-graph-joints.csv";
//     writeQCsv(resultsCsvFile, qTraj);
//     string posesCsvFile = "/home/elisabeth/repos/teo-sharon/programs/GenerateManipulationTrajectories/trajectories/shoulderElbowWirstHandTest/prueba" + to_string(nDemo) + "-graph-poses.csv";
//     writePoseCsv(posesCsvFile, poses);

//     cout<<mapHumanToRobotMotion.getDiscreteHandPoses().size()<<endl;

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------//


    return EXIT_SUCCESS;
}

