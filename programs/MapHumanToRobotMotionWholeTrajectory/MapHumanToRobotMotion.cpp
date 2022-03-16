#include "MapHumanToRobotMotion.hpp"
#include <boost/graph/graphviz.hpp>


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
    qmin.resize(NJoints);
    qmax.resize(NJoints);
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



void MapHumanToRobotMotion::getHumanData(const string &filename, vector<array<double, 8>> &handTrajectory, vector<array<double, 7>> &wristTrajectory, vector<array<double, 7>> &elbowTrajectory, vector<array<double, 7>> &shoulderTrajectory)
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
        array<double, 8> poseHand;
        array<double, 7> poseWrist;
        array<double, 7> poseElbow;
        array<double, 7> poseShoulder;

        unsigned int colIdx = 0;
        while (ss >> val)
        {
            if (colIdx < 8)
            {
                poseHand[colIdx] = val;
            }
            else if (colIdx >= 8 && colIdx < 15)
            {
                poseWrist[colIdx - 8] = val;
            }
            else if (colIdx >= 15 && colIdx < 22)
            {
                poseElbow[colIdx - 15] = val;
            }
            else
            {
                poseShoulder[colIdx - 22] = val;
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
    }

    csvFile.close();
}

void MapHumanToRobotMotion::printHumanData(const vector<array<double, 8>> &HandData, const vector<array<double, 7>> &wristData,
                                           const vector<array<double, 7>> &elbowData, const vector<array<double, 7>> &shoulderData)
{
    for (unsigned int i = 0; i < HandData.size(); i++)
    {
        cout << "Frame: " << HandData[i][0] << endl;
        cout << "Hand: "
             << " x: " << HandData[i][1] << " y: " << HandData[i][2] << " z: " << HandData[i][3] << " qx: " << HandData[i][4] << " qy: " << HandData[i][5] << " qz: " << HandData[i][6] << " qw: " << HandData[i][7] << endl;
        cout << "Wrist: "
             << " x: " << wristData[i][0] << " y: " << wristData[i][1] << " z: " << wristData[i][2] << " qx: " << wristData[i][3] << " qy: " << wristData[i][4] << " qz: " << wristData[i][5] << " qw: " << wristData[i][6] << endl;
        cout << "Elbow: "
             << " x: " << elbowData[i][0] << " y: " << elbowData[i][1] << " z: " << elbowData[i][2] << " qx: " << elbowData[i][3] << " qy: " << elbowData[i][4] << " qz: " << elbowData[i][5] << " qw: " << elbowData[i][6] << endl;
        cout << "Shoulder: "
             << " x: " << shoulderData[i][0] << " y: " << shoulderData[i][1] << " z: " << shoulderData[i][2] << " qx: " << shoulderData[i][3] << " qy: " << shoulderData[i][4] << " qz: " << shoulderData[i][5] << " qw: " << shoulderData[i][6] << endl;
    }
}




void MapHumanToRobotMotion::getHumanTrajectoryPoses(vector<array<double, 8>> &trajectoryHandData, vector<array<double, 7>> &trajectoryWristData,
                                                    vector<array<double, 7>> &trajectoryElbowData, vector<array<double, 7>> &trajectoryShoulderData,
                                                    const float &distBtwPoses)
{

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
                    handArray[j - 1] = trajectoryHandData[i][j] - TrunkHeight;
                    wristArray[j - 1] = trajectoryWristData[i][j - 1] - TrunkHeight;
                    elbowArray[j - 1] = trajectoryElbowData[i][j - 1] - TrunkHeight;
                    shoulderArray[j - 1] = trajectoryShoulderData[i][j - 1] - TrunkHeight;
                }
                else
                {
                    handArray[j - 1] = trajectoryHandData[i][j];
                    wristArray[j - 1] = trajectoryWristData[i][j - 1];
                    elbowArray[j - 1] = trajectoryElbowData[i][j - 1];
                    shoulderArray[j - 1] = trajectoryShoulderData[i][j - 1];
                }
            }
            _discreteHandPoses.push_back(handArray);
            _discreteWristPoses.push_back(wristArray);
            _discreteElbowPoses.push_back(elbowArray);
            _discreteShoulderPoses.push_back(shoulderArray);
        }
        else
        {

            dist = sqrt(pow((trajectoryHandData[i][1] - _discreteHandPoses.back()[0]),2) + pow((trajectoryHandData[i-1][2] - _discreteHandPoses.back()[1]),2) + pow(((trajectoryHandData[i][3] - TrunkHeight) - _discreteHandPoses.back()[2]), 2));
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
                        handArray[j - 1] = trajectoryHandData[i][j] - TrunkHeight;
                        wristArray[j - 1] = trajectoryWristData[i][j - 1] - TrunkHeight;
                        elbowArray[j - 1] = trajectoryElbowData[i][j - 1] - TrunkHeight;
                        shoulderArray[j - 1] = trajectoryShoulderData[i][j - 1] - TrunkHeight;

                    }
                    else
                    {
                        handArray[j-1] = trajectoryHandData[i][j];
                        wristArray[j - 1] = trajectoryWristData[i][j - 1];
                        elbowArray[j - 1] = trajectoryElbowData[i][j - 1];
                        shoulderArray[j - 1] = trajectoryShoulderData[i][j - 1];
                    }
                }
                _discreteHandPoses.push_back(handArray);
                _discreteWristPoses.push_back(wristArray);
                _discreteElbowPoses.push_back(elbowArray);
                _discreteShoulderPoses.push_back(shoulderArray);
            }
        }
    }
}

void MapHumanToRobotMotion::addFirstLayerToGraph(int numberOfNodes,  const KDL::Frame &fpose){
    
    unsigned int n = 0;
    while(n<numberOfNodes){
        KDL::JntArray  qInit(NJoints);
        KDL::JntArray  q(NJoints);
        for(unsigned int j = 0; j<NJoints; j++){
            qInit(j) = fRand(_qmin(j),_qmax(j));
        }
        int foundik = (*_iksolver).CartToJnt(qInit, fpose, q);
        if (foundik == 0) //ik solution found
        {
            VertexProperty vp;
            vp.name = "v_0_"+to_string(n);
            cout << vp.name<<endl;
            for(unsigned int j = 0; j<NJoints; j++){
                vp.q[j] = q(j);
                cout << vp.q[j]<<" ";    
            }
            cout<<endl;
            vp.goalPosition[0] = fpose.p[0];
            vp.goalPosition[1] = fpose.p[1];
            vp.goalPosition[2] = fpose.p[2];

            KDL::Frame currentShoulderPose, currentElbowPose, currentWristPose, currentPose;
            getCurrentPoses(q, currentShoulderPose, currentElbowPose, currentWristPose, currentPose);
            vp.currentPosition[0] = currentPose.p[0];
            vp.currentPosition[1] = currentPose.p[1];
            vp.currentPosition[2] = currentPose.p[2];
            vp.currentDistanteToGoalPosition = sqrt(pow(fpose.p[0]-currentPose.p[0],2)+pow(fpose.p[1]-currentPose.p[1],2)+pow(fpose.p[2]-currentPose.p[2],2));

            boost::add_vertex(vp, _graph);
            n++;
        }
    }
}

void MapHumanToRobotMotion::getCurrentPoses(const KDL::JntArray &q, KDL::Frame &currentShoulderPose, KDL::Frame &currentElbowPose, KDL::Frame &currentWristPose, KDL::Frame &currentPose)
{
    (*_fksolver).JntToCart(q, currentPose);

    (*_fksolver).JntToCart(q,currentShoulderPose, 3);

    (*_fksolver).JntToCart(q, currentElbowPose, 5);

    (*_fksolver).JntToCart(q, currentWristPose, 7);

}
void MapHumanToRobotMotion::addNextLayersToGraph(int numberOfNodesPreviousLayer,  const KDL::Frame &fpose, const int step){
    
    cout<<"next layer: "<<step<<endl;

    cout<<"size: "<<_graph.m_vertices.size()<<endl;

    KDL::JntArray  qInit(NJoints);
    KDL::JntArray  q(NJoints);

    unsigned int n = 0;
    unsigned int nodesGraph = _graph.m_vertices.size();
    for( unsigned int vd = nodesGraph-numberOfNodesPreviousLayer; vd< nodesGraph; vd++){
        const VertexProperty&  vp = _graph[vd];  // vp is the vertex property
        cout << "vertex " << vd << ": name is " << vp.name << endl;
        cout << "qInit: ";
        for(unsigned int j=0; j<NJoints; j++){
            qInit(j) = vp.q[j];
            cout<<vp.q[j]<<" ";
        }
        cout<<endl;

        int foundik = (*_iksolver).CartToJnt(qInit, fpose, q);

        if(foundik == 0){
            VertexProperty vp;
            vp.name = "v_"+to_string(step)+"_"+to_string(n);
            cout << vp.name<<endl;
            for(unsigned int j = 0; j<NJoints; j++){
                vp.q[j] = q(j);
                cout << vp.q[j]<<" ";    
            }
            cout<<endl;
            n++;

            vp.goalPosition[0] = fpose.p[0];
            vp.goalPosition[1] = fpose.p[1];
            vp.goalPosition[2] = fpose.p[2];

            KDL::Frame currentShoulderPose, currentElbowPose, currentWristPose, currentPose;
            getCurrentPoses(q, currentShoulderPose, currentElbowPose, currentWristPose, currentPose);
            vp.currentPosition[0] = currentPose.p[0];
            vp.currentPosition[1] = currentPose.p[1];
            vp.currentPosition[2] = currentPose.p[2];
            vp.currentDistanteToGoalPosition = sqrt(pow(fpose.p[0]-currentPose.p[0],2)+pow(fpose.p[1]-currentPose.p[1],2)+pow(fpose.p[2]-currentPose.p[2],2));
            cout<<"current distance: "<<vp.currentDistanteToGoalPosition<<endl;
            boost::add_vertex(vp, _graph);


        }
    }
    cout<<"nodes previous layer: "<<numberOfNodesPreviousLayer<<endl;
    cout<<"nodes current layer: "<<n<<endl;
    unsigned int edges = 0;
    for(unsigned int vd1 = _graph.m_vertices.size()-numberOfNodesPreviousLayer-n; vd1< _graph.m_vertices.size()-n; vd1++){
        edges = 0;
        for(unsigned int vd2 = _graph.m_vertices.size()-n; vd2< _graph.m_vertices.size(); vd2++){
            const VertexProperty&  vp1 = _graph[vd1];
            const VertexProperty&  vp2 = _graph[vd2];
            // cout << "Let's try to connect vertex " << vd1 << ": name is " << vp1.name << " with vertex "<<vd2<<" :name is "<<vp2.name<<endl;

            if(feasibleJointsVel(vp1, vp2)){
                EdgeProperty ep; // the edge connecting vertex 0 and 1
                ep.weight = vp2.currentDistanteToGoalPosition*_wpos;
                edges++;
                boost::add_edge(vd1,vd2,ep,_graph);
            }

        }
        cout<<"connected edges: "<<edges<<endl;
        

    }
    
    
//     // in the returned value 'status', status.first will be the edge descriptor if status.second == ture
//     pair<EdgeDescriptor, bool> status01 = boost::add_edge(vd0, vd1, ep01, graph);
//     EdgeDescriptor ed01 = status01.first; // this is how to access the returned edge descriptor
//     if (status01.second)
//         cout << "added edge " << ed01 << " connecting vertices " << vd0 << " and " << vd1 << endl;
//     else
//         cout << "failed to add an edge connecting vertices" << vd0 << " and " << vd1 << endl;

}

bool MapHumanToRobotMotion::feasibleJointsVel(const VertexProperty&  vp1, const VertexProperty&  vp2)
{
    for(unsigned int j=0; j<NJoints; j++){
        double vel = abs(vp1.q[j]-vp2.q[j])/TIME_STEP*KDL::rad2deg;
        if(vel >= MAX_JVEL){
            return false;
        }
    }
    return true;

}

void MapHumanToRobotMotion::ik(const KDL::JntArray &qInit, const KDL::Frame &fpose, KDL::JntArray &q){
    int foundfk = (*_iksolver).CartToJnt(qInit, fpose, q);
    cout<<"qInit: ";
    for(unsigned int j = 0; j<NJoints; j++)
        cout<<qInit(j)<<" ";
    cout<<endl;
    cout<<"q: ";
    for(unsigned int j = 0; j<NJoints; j++)
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
    // cout<<fpose<<endl;
    // cout<<q<<endl;

}

void MapHumanToRobotMotion::findShortestPathInGraph(){
    

    auto g =  boost::make_reverse_graph(_graph);

    std::vector<VertexDescriptor> predecessors(boost::num_vertices(g)); // To store parents
    std::vector<double> distances(boost::num_vertices(g)); // To store distances

    IndexMap indexMap; // = boost::get(boost::vertex_index, g);

    PredecessorMap predecessorMap(&predecessors[0], indexMap);
    DistanceMap distanceMap(&distances[0], indexMap);


    boost::dijkstra_shortest_paths(g.m_g, 160, boost::weight_map(get(&EdgeProperty::weight, g.m_g)).predecessor_map(predecessors.data()));

    std::cout << "distances and parents:" << std::endl;
    VertexIterator vi, vend;
    for (tie(vi, vend) = vertices(g); vi != vend; ++vi) {
        std::cout << "distance(" << (*vi) << ") = " << ", ";
        std::cout << "parent(" << (*vi) << ") = " << predecessors[*vi] << std::endl;
    }
    std::cout << std::endl;

    std::ofstream dot_file("figs/dijkstra-eg.dot");

  dot_file << "digraph D {\n"
    << "  rankdir=LR\n"
    << "  size=\"4,3\"\n"
    << "  ratio=\"fill\"\n"
    << "  edge[style=\"bold\"]\n" << "  node[shape=\"circle\"]\n";

  EdgeIterator ei, ei_end;
  for (tie(ei, ei_end) = edges(g.m_g); ei != ei_end; ++ei) {
        EdgeDescriptor e = *ei;
        VertexDescriptor u = source(e, g.m_g), v = target(e, g.m_g);
        dot_file << u << " -> " << v << "[label=\"" << "get(weightmap, e)" << "\"";
        if (predecessors[v] == u)
            dot_file << ", color=\"black\"";
        else
        dot_file << ", color=\"grey\"";
        dot_file << "]";
    }
    dot_file << "}";
                          


}



int main(int argc, char **argv)
{

    KDL::JntArray qmin(NJoints), qmax(NJoints);
    makeQLimitsTeoTrunkAndRightArmKinematics(qmin, qmax);
    KDL::Chain chain = makeTeoTrunkAndRightArmKinematicsFromDH();
    std::cout<<"chain: "<<chain.segments.size()<<std::endl;

    MapHumanToRobotMotion mapHumanToRobotMotion(chain, qmin, qmax, 1000, 1e-4);
    int nDemo = 10;
    string csvFile = "/home/elisabeth/repos/teo-sharon/programs/GenerateManipulationTrajectories/trajectories/shoulderElbowWirstHandTest/prueba" + to_string(nDemo) + "-smoothed.csv";
    vector<array<double, 8>> desiredTrajectoryData;
    vector<array<double, 7>> wristTrajectoryData;
    vector<array<double, 7>> elbowTrajectoryData;
    vector<array<double, 7>> shoulderTrajectoryData;
    mapHumanToRobotMotion.getHumanData(csvFile, desiredTrajectoryData, wristTrajectoryData, elbowTrajectoryData, shoulderTrajectoryData);
    mapHumanToRobotMotion.printHumanData(desiredTrajectoryData, wristTrajectoryData, elbowTrajectoryData, shoulderTrajectoryData);
    mapHumanToRobotMotion.getHumanTrajectoryPoses(desiredTrajectoryData, wristTrajectoryData, elbowTrajectoryData, shoulderTrajectoryData, 0.001);
    
    
    KDL::JntArray qInit(NJoints);
    KDL::JntArray q(NJoints); 
    KDL::Frame fGoal;
    double norm = sqrt(mapHumanToRobotMotion.getDiscreteHandPoses()[0][3] * mapHumanToRobotMotion.getDiscreteHandPoses()[0][3] + mapHumanToRobotMotion.getDiscreteHandPoses()[0][4]*mapHumanToRobotMotion.getDiscreteHandPoses()[0][4] + mapHumanToRobotMotion.getDiscreteHandPoses()[0][5]*mapHumanToRobotMotion.getDiscreteHandPoses()[0][5] + mapHumanToRobotMotion.getDiscreteHandPoses()[0][6] * mapHumanToRobotMotion.getDiscreteHandPoses()[0][6]);
    KDL::Rotation rotKdl = KDL::Rotation::Quaternion(mapHumanToRobotMotion.getDiscreteHandPoses()[0][3] / norm, mapHumanToRobotMotion.getDiscreteHandPoses()[0][4] / norm, mapHumanToRobotMotion.getDiscreteHandPoses()[0][5] / norm, mapHumanToRobotMotion.getDiscreteHandPoses()[0][6] / norm);
    KDL::Vector posKdl = KDL::Vector(mapHumanToRobotMotion.getDiscreteHandPoses()[0][0], mapHumanToRobotMotion.getDiscreteHandPoses()[0][1], mapHumanToRobotMotion.getDiscreteHandPoses()[0][2]);
    rotKdl = rotKdl * KDL::Rotation::RotX(-KDL::PI / 2.0);
    rotKdl = rotKdl * KDL::Rotation::RotZ(-KDL::PI / 2.0 - KDL::PI / 4.0);
    fGoal.M = rotKdl;
    fGoal.p = posKdl;

    mapHumanToRobotMotion.addFirstLayerToGraph(20, fGoal);


    for (unsigned int step = 1; step<10; step++){
        norm = sqrt(mapHumanToRobotMotion.getDiscreteHandPoses()[step][3] * mapHumanToRobotMotion.getDiscreteHandPoses()[step][3] + mapHumanToRobotMotion.getDiscreteHandPoses()[step][4]*mapHumanToRobotMotion.getDiscreteHandPoses()[step][4] + mapHumanToRobotMotion.getDiscreteHandPoses()[step][5]*mapHumanToRobotMotion.getDiscreteHandPoses()[step][5] + mapHumanToRobotMotion.getDiscreteHandPoses()[1][6] * mapHumanToRobotMotion.getDiscreteHandPoses()[step][6]);
        rotKdl = KDL::Rotation::Quaternion(mapHumanToRobotMotion.getDiscreteHandPoses()[step][3] / norm, mapHumanToRobotMotion.getDiscreteHandPoses()[step][4] / norm, mapHumanToRobotMotion.getDiscreteHandPoses()[step][5] / norm, mapHumanToRobotMotion.getDiscreteHandPoses()[step][6] / norm);
        posKdl = KDL::Vector(mapHumanToRobotMotion.getDiscreteHandPoses()[step][0], mapHumanToRobotMotion.getDiscreteHandPoses()[step][1], mapHumanToRobotMotion.getDiscreteHandPoses()[step][2]);
        rotKdl = rotKdl * KDL::Rotation::RotX(-KDL::PI / 2.0);
        rotKdl = rotKdl * KDL::Rotation::RotZ(-KDL::PI / 2.0 - KDL::PI / 4.0);
        fGoal.M = rotKdl;
        fGoal.p = posKdl;
        mapHumanToRobotMotion.addNextLayersToGraph(20,fGoal,step);
    }
    

    mapHumanToRobotMotion.findShortestPathInGraph();
    
    cout<<mapHumanToRobotMotion.getDiscreteHandPoses().size()<<endl;
    return EXIT_SUCCESS;
}

// // This example shows how to use the powerful boost graph library

// int main(int /*argc*/, char** /*argv*/) {
//     // create an instance of Graph
//     Graph graph;

//     // we first add some vertices (in this simple example only three vertices),
//     // then we connect them by an edge. The graph looks like:
//     //
//     //    v0     10
//     //    ------------ v1
//     //                /
//     //              /
//     //            /  20
//     //          /
//     //         v2

//     //-----------------------------------------------------------------------
//     //  add a vertex to the Graph: add_vertex()
//     //-----------------------------------------------------------------------
//     VertexProperty vp0;
//     vp0.value = 0;
//     vp0.name = "v0";
//     VertexDescriptor vd0 = boost::add_vertex(vp0, graph);
//     cout << "added vertex " << vd0 << endl;

//     VertexProperty vp1;
//     vp1.value = 1;
//     vp1.name = "v1";
//     VertexDescriptor vd1 = boost::add_vertex(vp1, graph);
//     cout << "added vertex " << vd1 << endl;

//     VertexProperty vp2;
//     vp2.value = 2;
//     vp2.name = "v2";
//     VertexDescriptor vd2 = boost::add_vertex(vp2, graph);
//     cout << "added vertex " << vd2 << endl;

//     //-----------------------------------------------------------------------
//     //  add an edge to the Graph: add_vertex()
//     //-----------------------------------------------------------------------
//     EdgeProperty ep01; // the edge connecting vertex 0 and 1
//     ep01.weight = 10.0f;
//     // in the returned value 'status', status.first will be the edge descriptor if status.second == ture
//     pair<EdgeDescriptor, bool> status01 = boost::add_edge(vd0, vd1, ep01, graph);
//     EdgeDescriptor ed01 = status01.first; // this is how to access the returned edge descriptor
//     if (status01.second)
//         cout << "added edge " << ed01 << " connecting vertices " << vd0 << " and " << vd1 << endl;
//     else
//         cout << "failed to add an edge connecting vertices" << vd0 << " and " << vd1 << endl;

//     EdgeProperty ep12; // the edge connecting vertex 1 and 2
//     ep12.weight = 20.0f;
//     pair<EdgeDescriptor, bool> status12 = boost::add_edge(vd1, vd2, ep12, graph);
//     EdgeDescriptor ed12 = status12.first; // this is how to access the returned edge descriptor
//     if (status12.second)
//         cout << "added edge " << ed12 << " connecting vertices " << vd1 << " and " << vd2 << endl;
//     else
//         cout << "failed to add an edge connecting vertices" << vd1 << " and " << vd2 << endl;

//     //-----------------------------------------------------------------------
//     //  query the number of vertices in a graph
//     //-----------------------------------------------------------------------
//     cout << "number of vertices in the graph: " << boost::num_vertices(graph) << endl;

//     //-----------------------------------------------------------------------
//     //  query the number of edges in a graph
//     //-----------------------------------------------------------------------
//     cout << "number of edges in the graph: " << boost::num_edges(graph) << endl;

//     //-----------------------------------------------------------------------
//     //  traverse all the vertices of a graph
//     //-----------------------------------------------------------------------
//     pair<VertexIterator, VertexIterator> vi = boost::vertices(graph);
//     for (VertexIterator vit = vi.first; vit != vi.second; ++vit) {
//         VertexDescriptor vd = *vit;             // vd is the vertex descriptor
//         //-----------------------------------------------------------------------
//         //  access the vertex property given the vertex descriptor
//         //-----------------------------------------------------------------------
//         const VertexProperty&  vp = graph[vd];  // vp is the vertex property
//         cout << "vertex " << vd << ": name is " << vp.name << ", value is " << vp.value << endl;
//      }

//     //-----------------------------------------------------------------------
//     //  traverse all the edges of a graph
//     //-----------------------------------------------------------------------
//     pair<EdgeIterator, EdgeIterator> ei = boost::edges(graph);
//     for (EdgeIterator eit = ei.first; eit != ei.second; ++eit) {
//        EdgeDescriptor ed = *eit;               // ed is the edge descriptor
//        //-----------------------------------------------------------------------
//        //  access the edge property given the edge descriptor
//        //-----------------------------------------------------------------------
//        const EdgeProperty&  ep = graph[ed];     // ep is the edge property
//        cout << "edge " << ed << ": weight is " << ep.weight << endl;
//     }

//     //-----------------------------------------------------------------------
//     //  access the two vetices connected by an edge given the edge descriptor
//     //-----------------------------------------------------------------------
//     VertexDescriptor sd = source(ed12, graph);
//     VertexDescriptor td = target(ed12, graph);
//     cout << "source vertex of the edge " << ed12 << " is " << sd << endl;
//     cout << "target vertex of the edge " << ed12 << " is " << td << endl;

//     //-----------------------------------------------------------------------
//     //  query the edge for a vertex pair.
//     //  If the second element of the returned value is true, the edge exists and the first
//     //  element carries the edge descriptor.
//     //  If the second element of the returned value is false, the edge does not exists.
//     //-----------------------------------------------------------------------
//     pair<EdgeDescriptor, bool> test = boost::edge(vd0, vd2, graph);
//     if (test.second == true) {
//         EdgeDescriptor ed = test.first;
//         cout << "vertex " << vd0 << " and " << vd2 << " are connected by the edge " << ed << endl;
//      }
//     else
//         cout << "vertex " << vd0 << " and " << vd2 << " are not connected by an edge." << endl;

//     test = boost::edge(vd1, vd2, graph);
//     if (test.second == true) {
//         EdgeDescriptor ed = test.first;
//         cout << "vertex " << vd1 << " and " << vd2 << " are connected by the edge " << ed << endl;
//      }
//     else
//         cout << "vertex " << vd1 << " and " << vd2 << " are not connected by an edge." << endl;

//     //-----------------------------------------------------------------------
//     //  access the neighboring vertices of a vertex
//     //-----------------------------------------------------------------------
//     cout << "neighboring vertices of vertex " << vd1 << ": ";
//     pair<AdjacentVertexIterator, AdjacentVertexIterator> adj_v_iter = boost::adjacent_vertices(vd1, graph);
//     for (AdjacentVertexIterator ait = adj_v_iter.first; ait != adj_v_iter.second; ++ait) {
//         VertexDescriptor vd = *ait;
//         cout << vd << " ";
//     }
//     cout << endl;

//     //-----------------------------------------------------------------------
//     //  access the incicent edges of a vertex
//     //-----------------------------------------------------------------------
//     cout << "incident edges of vertex " << vd1 << ": ";
//     pair<OutEdgeIterator, OutEdgeIterator> adj_e_iter = boost::out_edges(vd1, graph);
//     for (OutEdgeIterator ait = adj_e_iter.first; ait != adj_e_iter.second; ++ait) {
//         EdgeDescriptor ed = *ait;
//         cout << ed << " ";
//     }
//     cout << endl;

//     return EXIT_SUCCESS;
// }