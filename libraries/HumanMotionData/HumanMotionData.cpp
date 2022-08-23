#include "HumanMotionData.hpp"

namespace HumanMotionData
{
    void getHumanData(const string &filename, vector<array<double, 7>> &handTrajectory, 
                          vector<array<double, 7>> &wristTrajectory, vector<array<double, 7>> &elbowTrajectory,
                          vector<array<double, 7>> &shoulderTrajectory,vector<array<double, 7>> &neckTrajectory,
                          vector<array<double, 7>> &hipTrajectory)
    {

        cout << filename << endl;

        ifstream csvFile(filename);

        if (!csvFile.is_open())
            throw runtime_error("Could not open csv file");

        double val;
        string line;
        while (getline(csvFile, line))
        {
            stringstream ss(line);
            array<double, 7> poseHand;
            array<double, 7> poseWrist;
            array<double, 7> poseElbow;
            array<double, 7> poseShoulder;
            array<double, 7> poseNeck;
            array<double,7> poseHip;

            unsigned int colIdx = 0;
            while (ss >> val)
            {

                if (colIdx > 0 && colIdx < 8)
                {
                    poseHand[colIdx - 1] = val;
                }
                else if (colIdx >= 8 && colIdx < 15)
                {
                    poseWrist[colIdx - 8] = val;
                }
                else if (colIdx >= 15 && colIdx < 22)
                {
                    poseElbow[colIdx - 15] = val;
                }
                else if (colIdx >= 22 && colIdx < 29)
                {
                    poseShoulder[colIdx - 22] = val;
                }
                else if (colIdx >= 29 && colIdx < 36)
                {
                    poseNeck[colIdx - 29] = val;
                }
                else if (colIdx >= 36 && colIdx < 43)
                {
                    poseHip[colIdx - 36] = val;
                }

                if (ss.peek() == ',')
                {
                    ss.ignore();
                }
                colIdx++;
            }

            handTrajectory.push_back(poseHand);
            wristTrajectory.push_back(poseWrist);
            elbowTrajectory.push_back(poseElbow);
            shoulderTrajectory.push_back(poseShoulder);
            neckTrajectory.push_back(poseNeck);
            hipTrajectory.push_back(poseHip);
        }

        csvFile.close();
    }


    void linkLengthAdjustementPoses(const vector<array<double, 7>> &handTrajectory, const vector<array<double, 7>> &wristTrajectory,
                                    const vector<array<double, 7>> &elbowTrajectory, const vector<array<double, 7>> &shoulderTrajectory,
                                    vector<array<double, 7>> &neckTrajectory, vector<array<double, 7>> &hipTrajectory, 
                                    vector<array<double, 7>> &adjustedHipTrajectory, vector<array<double, 7>> &adjustedNeckTrajectory,
                                    vector<array<double, 7>> &adjustedShoulderTrajectory, vector<array<double, 7>> &adjustedElbowTrajectory,
                                    vector<array<double, 7>> &adjustedWristTrajectory, vector<array<double, 7>> &adjustedHandTrajectory)
    {

        for (int iPose = 0; iPose < handTrajectory.size(); iPose++)
        {

            double norm = sqrt(shoulderTrajectory[iPose][3] * shoulderTrajectory[iPose][3] + shoulderTrajectory[iPose][4] * shoulderTrajectory[iPose][4] + shoulderTrajectory[iPose][5] * shoulderTrajectory[iPose][5] + shoulderTrajectory[iPose][6] * shoulderTrajectory[iPose][6]);
            KDL::Rotation rotKdl = KDL::Rotation::Quaternion(shoulderTrajectory[iPose][3] / norm, shoulderTrajectory[iPose][4] / norm, shoulderTrajectory[iPose][5] / norm, shoulderTrajectory[iPose][6] / norm);
            KDL::Frame frameShoulderWorld(rotKdl, KDL::Vector(shoulderTrajectory[iPose][0], shoulderTrajectory[iPose][1], shoulderTrajectory[iPose][2]));

            norm = sqrt(neckTrajectory[iPose][3]*neckTrajectory[iPose][3] + neckTrajectory[iPose][4]*neckTrajectory[iPose][4] + neckTrajectory[iPose][5]*neckTrajectory[iPose][5] + neckTrajectory[iPose][6]*neckTrajectory[iPose][6]);
            rotKdl = KDL::Rotation::Quaternion(neckTrajectory[iPose][3]/norm, neckTrajectory[iPose][4]/norm, neckTrajectory[iPose][5]/norm, neckTrajectory[iPose][6]/norm);
            KDL::Frame frameNeckWorld(rotKdl, KDL::Vector(neckTrajectory[iPose][0], neckTrajectory[iPose][1], neckTrajectory[iPose][2]));

            norm = sqrt(hipTrajectory[iPose][3]*hipTrajectory[iPose][3] + hipTrajectory[iPose][4]*hipTrajectory[iPose][4] + hipTrajectory[iPose][5]*hipTrajectory[iPose][5] + hipTrajectory[iPose][6]*hipTrajectory[iPose][6]);
            rotKdl = KDL::Rotation::Quaternion(hipTrajectory[iPose][3]/norm, hipTrajectory[iPose][4]/norm, hipTrajectory[iPose][5]/norm, hipTrajectory[iPose][6]/norm);
            KDL::Rotation rotKDLHip;
            KDL::Frame frameHipWorld(rotKDLHip, KDL::Vector(hipTrajectory[iPose][0], hipTrajectory[iPose][1], hipTrajectory[iPose][2]));
        
            norm = sqrt(elbowTrajectory[iPose][3] * elbowTrajectory[iPose][3] + elbowTrajectory[iPose][4] * elbowTrajectory[iPose][4] + elbowTrajectory[iPose][5] * elbowTrajectory[iPose][5] + elbowTrajectory[iPose][6] * elbowTrajectory[iPose][6]);
            rotKdl = KDL::Rotation::Quaternion(elbowTrajectory[iPose][3] / norm, elbowTrajectory[iPose][4] / norm, elbowTrajectory[iPose][5] / norm, elbowTrajectory[iPose][6] / norm);
            KDL::Frame frameElbowWorld(rotKdl, KDL::Vector(elbowTrajectory[iPose][0], elbowTrajectory[iPose][1], elbowTrajectory[iPose][2]));
            
            norm = sqrt(wristTrajectory[iPose][3] * wristTrajectory[iPose][3] + wristTrajectory[iPose][4] * wristTrajectory[iPose][4] + wristTrajectory[iPose][5] * wristTrajectory[iPose][5] + wristTrajectory[iPose][6] * wristTrajectory[iPose][6]);
            rotKdl = KDL::Rotation::Quaternion(wristTrajectory[iPose][3] / norm, wristTrajectory[iPose][4] / norm, wristTrajectory[iPose][5] / norm, wristTrajectory[iPose][6] / norm);
            KDL::Frame frameWristWorld(rotKdl, KDL::Vector(wristTrajectory[iPose][0], wristTrajectory[iPose][1], wristTrajectory[iPose][2]));

            norm = sqrt(handTrajectory[iPose][3] * handTrajectory[iPose][3] + handTrajectory[iPose][4] * handTrajectory[iPose][4] + handTrajectory[iPose][5] * handTrajectory[iPose][5] + handTrajectory[iPose][6] * handTrajectory[iPose][6]);
            rotKdl = KDL::Rotation::Quaternion(handTrajectory[iPose][3] / norm, handTrajectory[iPose][4] / norm, handTrajectory[iPose][5] / norm, handTrajectory[iPose][6] / norm);
            KDL::Frame frameHandWorld(rotKdl, KDL::Vector(handTrajectory[iPose][0], handTrajectory[iPose][1], handTrajectory[iPose][2]));

            KDL::Frame frameNeckHip = frameHipWorld.Inverse() * frameNeckWorld;

            KDL::Frame adjustedFrameNeckHip(frameNeckHip.M,TrunkToShoulderHeight/ frameNeckHip.p.Norm() * frameNeckHip.p);
            double qx,qy,qz,qw;
            adjustedFrameNeckHip.M.GetQuaternion(qx,qy,qz,qw);
            std::array<double, 7> adjustedNeckPose = {adjustedFrameNeckHip.p.x(), adjustedFrameNeckHip.p.y(), adjustedFrameNeckHip.p.z()+HipToTrunkHeight, qx, qy, qz, qw};
            adjustedNeckTrajectory.push_back(adjustedNeckPose);

            KDL::Frame frameShoulderNeck = frameNeckWorld.Inverse() * frameShoulderWorld;
            KDL::Frame adjustedFrameShoulderNeck(frameShoulderNeck.M, TrunkToShoulderWidth / frameShoulderNeck.p.Norm() * frameShoulderNeck.p);
            KDL::Frame adjustedFrameShoulderHip = adjustedFrameNeckHip * adjustedFrameShoulderNeck;
            adjustedFrameShoulderHip.M.GetQuaternion(qx,qy,qz,qw);
            std::array<double, 7> adjustedShoulderPose = {adjustedFrameShoulderHip.p.x(), adjustedFrameShoulderHip.p.y(), adjustedFrameShoulderHip.p.z()+HipToTrunkHeight, qx, qy, qz, qw};
            adjustedShoulderTrajectory.push_back(adjustedShoulderPose);

            KDL::Frame frameElbowShoulder = frameShoulderWorld.Inverse() * frameElbowWorld;
            KDL::Frame adjustedFrameElbowShoulder(frameElbowShoulder.M, ShoulderToElbowHeight / frameElbowShoulder.p.Norm() * frameElbowShoulder.p);
            KDL::Frame adjustedFrameElbowHip = adjustedFrameShoulderHip * adjustedFrameElbowShoulder;
            adjustedFrameElbowHip.M.GetQuaternion(qx,qy,qz,qw);
            std::array<double,7> adjustedElbowPose = {adjustedFrameElbowHip.p.x(), adjustedFrameElbowHip.p.y(), adjustedFrameElbowHip.p.z()+HipToTrunkHeight, qx, qy, qz, qw};
            adjustedElbowTrajectory.push_back(adjustedElbowPose);

            KDL::Frame frameWristElbow = frameElbowWorld.Inverse() * frameWristWorld;
            KDL::Frame adjustedFrameWristElbow(frameWristElbow.M, ElbowToWristHeight / frameWristElbow.p.Norm() * frameWristElbow.p);
            KDL::Frame adjustedFrameWristHip = adjustedFrameElbowHip * adjustedFrameWristElbow;
            adjustedFrameWristHip.M.GetQuaternion(qx,qy,qz,qw);
            std::array<double,7> adjustedWristPose = {adjustedFrameWristHip.p.x(), adjustedFrameWristHip.p.y(), adjustedFrameWristHip.p.z()+HipToTrunkHeight, qx, qy, qz, qw};
            adjustedWristTrajectory.push_back(adjustedWristPose);

            KDL::Frame frameHandWrist = frameWristWorld.Inverse() * frameHandWorld;
            KDL::Frame adjustedFrameHandWrist(frameHandWrist.M, WristToHandHeight / frameHandWrist.p.Norm() * frameHandWrist.p);
            KDL::Frame adjustedFrameHandHip = adjustedFrameWristHip * adjustedFrameHandWrist;
            adjustedFrameHandHip.M.GetQuaternion(qx,qy,qz,qw);
            std::array<double,7> adjustedHandPose = {adjustedFrameHandHip.p.x(), adjustedFrameHandHip.p.y(), adjustedFrameHandHip.p.z()+HipToTrunkHeight, qx, qy, qz, qw};
            adjustedHandTrajectory.push_back(adjustedHandPose);
        
            frameHipWorld.M.GetQuaternion(qx,qy,qz,qw);
            std::array<double,7> adjustedHipPose = {0, 0, HipToTrunkHeight, qx, qy, qz, qw};
            adjustedHipTrajectory.push_back(adjustedHipPose);
        }
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

    KDL::Frame getFrame(std::vector<std::array<double, 7>> discretePoses, int iPose)
    {
        KDL::Frame fPose;
        double norm = sqrt(discretePoses[iPose][3] * discretePoses[iPose][3] + discretePoses[iPose][4] * discretePoses[iPose][4] + discretePoses[iPose][5] * discretePoses[iPose][5] + discretePoses[iPose][6] * discretePoses[iPose][6]);
        KDL::Rotation rotKdl = KDL::Rotation::Quaternion(discretePoses[iPose][3] / norm, discretePoses[iPose][4] / norm, discretePoses[iPose][5] / norm, discretePoses[iPose][6] / norm);
        KDL::Vector posKdl = KDL::Vector(discretePoses[iPose][0], discretePoses[iPose][1], discretePoses[iPose][2]);

        fPose.M = rotKdl;
        fPose.p = posKdl;
        return fPose;
    }

    void printHumanData(const vector<array<double, 7>> &HandData, const vector<array<double, 7>> &wristData, 
                            const vector<array<double, 7>> &elbowData, const vector<array<double, 7>> &shoulderData,
                            const vector<array<double, 7>> &neckData,const vector<array<double, 7>> &hipData)
    {
        for (unsigned int i = 0; i < HandData.size(); i++)
        {
            cout << "Frame: " << i << endl;
            cout << "Hand: "
                 << " x: " << HandData[i][0] << " y: " << HandData[i][1] << " z: " << HandData[i][2] << " qx: " << HandData[i][3] << " qy: " << HandData[i][4] << " qz: " << HandData[i][5] << " qw: " << HandData[i][6] << endl;
            cout << "Wrist: "
                 << " x: " << wristData[i][0] << " y: " << wristData[i][1] << " z: " << wristData[i][2] << " qx: " << wristData[i][3] << " qy: " << wristData[i][4] << " qz: " << wristData[i][5] << " qw: " << wristData[i][6] << endl;
            cout << "Elbow: "
                 << " x: " << elbowData[i][0] << " y: " << elbowData[i][1] << " z: " << elbowData[i][2] << " qx: " << elbowData[i][3] << " qy: " << elbowData[i][4] << " qz: " << elbowData[i][5] << " qw: " << elbowData[i][6] << endl;
            cout << "Shoulder: "
                 << " x: " << shoulderData[i][0] << " y: " << shoulderData[i][1] << " z: " << shoulderData[i][2] << " qx: " << shoulderData[i][3] << " qy: " << shoulderData[i][4] << " qz: " << shoulderData[i][5] << " qw: " << shoulderData[i][6] << endl;
            cout << "Neck: "
                << " x: " << neckData[i][0] << " y: " << neckData[i][1] << " z: " << neckData[i][2] << " qx: " << neckData[i][3] << " qy: " << neckData[i][4] << " qz: " << neckData[i][5] << " qw: " << neckData[i][6] << endl;
            cout << "Hip: "
                << "x: "<< hipData[i][0]<< " y: " << hipData[i][1] << " z: " << hipData[i][2] << " qx: " << hipData[i][3] << " qy: " << hipData[i][4] << " qz: " << hipData[i][5] << " qw: " << hipData[i][6] << endl;
        }
    }

    void getHumanTrajectoryPoses(vector<array<double, 7>> &trajectoryHandData, vector<array<double, 7>> &trajectoryWristData, 
                                     vector<array<double, 7>> &trajectoryElbowData, vector<array<double, 7>> &trajectoryShoulderData,
                                     vector<array<double, 7>> &trajectoryNeckData,vector<array<double, 7>> &trajectoryHipData, const float &distBtwPoses)
    {

        vector<array<double, 7>> auxTrajectoryHandData(trajectoryHandData);
        vector<array<double, 7>> auxTrajectoryWristData(trajectoryWristData);
        vector<array<double, 7>> auxTrajectoryElbowData(trajectoryElbowData);
        vector<array<double, 7>> auxTrajectoryShoulderData(trajectoryShoulderData);

        float dist = numeric_limits<float>::max();
        for (unsigned int i = 0; i < trajectoryHandData.size(); i++)
        {

            if (i == 0 || i == (trajectoryHandData.size() - 1))
            {
                array<double, 7> handArray;
                array<double, 7> wristArray;
                array<double, 7> elbowArray;
                array<double, 7> shoulderArray;

                for (unsigned int j = 1; j < trajectoryHandData[i].size(); j++)
                {
                    if (j == 3)
                    {
                        handArray[j - 1] = trajectoryHandData[i][j - 1];
                        wristArray[j - 1] = trajectoryWristData[i][j - 1];
                        elbowArray[j - 1] = trajectoryElbowData[i][j - 1] ;
                        shoulderArray[j - 1] = trajectoryShoulderData[i][j - 1];
                    }
                    else
                    {
                        handArray[j - 1] = trajectoryHandData[i][j - 1];
                        wristArray[j - 1] = trajectoryWristData[i][j - 1];
                        elbowArray[j - 1] = trajectoryElbowData[i][j - 1];
                        shoulderArray[j - 1] = trajectoryShoulderData[i][j - 1];
                    }
                }
                auxTrajectoryHandData.push_back(handArray);
                auxTrajectoryWristData.push_back(wristArray);
                auxTrajectoryElbowData.push_back(elbowArray);
                auxTrajectoryShoulderData.push_back(shoulderArray);
            }
            else
            {

                dist = sqrt(pow((trajectoryHandData[i][0] - auxTrajectoryHandData.back()[0]), 2) + pow((trajectoryHandData[i - 1][1] - auxTrajectoryHandData.back()[1]), 2) + pow(((trajectoryHandData[i][2] - TrunkHeight) - auxTrajectoryHandData.back()[2]), 2));
                if (dist >= distBtwPoses)
                {
                    array<double, 7> handArray;
                    array<double, 7> wristArray;
                    array<double, 7> elbowArray;
                    array<double, 7> shoulderArray;

                    for (unsigned int j = 1; j < trajectoryHandData[i].size(); j++)
                    {
                        if (j == 3)
                        {
                            handArray[j - 1] = trajectoryHandData[i][j - 1];
                            wristArray[j - 1] = trajectoryWristData[i][j - 1];
                            elbowArray[j - 1] = trajectoryElbowData[i][j - 1];
                            shoulderArray[j - 1] = trajectoryShoulderData[i][j - 1];
                        }
                        else
                        {
                            handArray[j - 1] = trajectoryHandData[i][j - 1];
                            wristArray[j - 1] = trajectoryWristData[i][j - 1];
                            elbowArray[j - 1] = trajectoryElbowData[i][j - 1];
                            shoulderArray[j - 1] = trajectoryShoulderData[i][j - 1];
                        }
                    }
                    auxTrajectoryHandData.push_back(handArray);
                    auxTrajectoryWristData.push_back(wristArray);
                    auxTrajectoryElbowData.push_back(elbowArray);
                    auxTrajectoryShoulderData.push_back(shoulderArray);
                }
            }
        }
        trajectoryHandData.clear();
        trajectoryElbowData.clear();
        trajectoryWristData.clear();
        trajectoryShoulderData.clear();

        copy(auxTrajectoryHandData.begin(), auxTrajectoryHandData.end(), back_inserter(trajectoryHandData));
        copy(auxTrajectoryWristData.begin(), auxTrajectoryWristData.end(), back_inserter(trajectoryWristData));
        copy(auxTrajectoryElbowData.begin(), auxTrajectoryElbowData.end(), back_inserter(trajectoryElbowData));
        copy(auxTrajectoryShoulderData.begin(), auxTrajectoryShoulderData.end(), back_inserter(trajectoryShoulderData));
    }

    void writePosesCsv(std::string filename, const vector<array<double, 7>> &handTrajectory, 
                          vector<array<double, 7>> &wristTrajectory, vector<array<double, 7>> &elbowTrajectory,
                          vector<array<double, 7>> &shoulderTrajectory, vector<array<double, 7>> &neckTrajectory,
                          vector<array<double, 7>> &hipTrajectory){

     std::ofstream myFile(filename);
        for (unsigned int i = 0; i < handTrajectory.size(); i++)
        {
            myFile << i << " " << std::setprecision(10) << handTrajectory[i][0] << " " << handTrajectory[i][1] << " " << handTrajectory[i][2] << " " << handTrajectory[i][3] << " " << handTrajectory[i][4]
                   << " " << handTrajectory[i][5] << " " << handTrajectory[i][6] << " "<< wristTrajectory[i][0]
                   <<" "<< wristTrajectory[i][1]<<" "<< wristTrajectory[i][2]<<" "<< wristTrajectory[i][3]<<" "<< wristTrajectory[i][4]
                   <<" "<< wristTrajectory[i][5]<<" "<< wristTrajectory[i][6]<<" "<< elbowTrajectory[i][0]<<" "<< elbowTrajectory[i][1]
                   <<" "<< elbowTrajectory[i][2]<<" "<< elbowTrajectory[i][3]<<" "<< elbowTrajectory[i][4]
                   <<" "<< elbowTrajectory[i][5]<<" "<< elbowTrajectory[i][6]<<" "<< shoulderTrajectory[i][0]
                   <<" "<< shoulderTrajectory[i][1]<<" "<< shoulderTrajectory[i][2]<<" "<< shoulderTrajectory[i][3]
                   <<" "<< shoulderTrajectory[i][4]<<" "<< shoulderTrajectory[i][5]<<" "<< shoulderTrajectory[i][6]
                   <<" "<< neckTrajectory[i][0] <<" "<<neckTrajectory[i][1]<<" "<<neckTrajectory[i][2]
                   <<" "<<neckTrajectory[i][3]<<" "<<neckTrajectory[i][4]<<" "<<neckTrajectory[i][5]<<" "<<neckTrajectory[i][6]
                   <<" "<<hipTrajectory[i][0]<<" "<<hipTrajectory[i][1]<<" "<<hipTrajectory[i][2]
                   <<" "<<hipTrajectory[i][3]<<" "<<hipTrajectory[i][4]<<" "<<hipTrajectory[i][5]<<" "<<hipTrajectory[i][6]<<"\n";
        }
        myFile.close();
    }
} // namespace HumanMotionData