// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "CollisionsVisualization.hpp"

#include <cstdio>

#include <yarp/os/LogStream.h>

using namespace sharon;

/************************************************************************/


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

/************************************************************************/

bool CollisionsVisualization::configure(yarp::os::ResourceFinder & rf)
{
    yDebug() << "CollisionsVisualization config:" << rf.toString();

    std::printf("--------------------------------------------------------------\n");

    if (rf.check("help"))
    {
        std::printf("CollisionsVisualization options:\n");
        std::printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        //std::printf("\t--file (default: \"%s\")\n", fileName.c_str());
    }
    rf.setDefaultConfigFile("collisionsVisualization.ini");

    m_robot = rf.find("robot").asString();
    m_deviceName = rf.find("deviceName").asString();
    m_frameId = rf.find("frameId").asString();
    if(!openDevices())
        return false;
    
    //  Getting the limits of each joint
    printf("---- Joint limits of %s\n",m_deviceName.c_str());
    m_qmin.resize(m_numJoints);
    m_qmax.resize(m_numJoints);

    for(unsigned int joint = 0; joint < m_numJoints; joint++){
        double min, max;
        m_iControlLimits->getLimits(joint, &min, &max);
        m_qmin(joint) = min;
        m_qmax(joint) = max;
    }


    yInfo() <<"Lets create the collision objects";
   
    m_boxSizes.push_back(std::array<float, 3>{0.3, 0.46, 0.5});
    CollisionGeometryPtr_t teoRootTrunk{new fcl::Boxf{m_boxSizes[0][0], m_boxSizes[0][1], m_boxSizes[0][2]}};
    fcl::Transform3f tfTest;
    fcl::CollisionObjectf collisionObject0{teoRootTrunk, tfTest};

    m_boxSizes.push_back(std::array<float, 3>{0.0, 0.0, 0.0});
    CollisionGeometryPtr_t teoAux1{new fcl::Boxf{m_boxSizes[2][0], m_boxSizes[2][1], m_boxSizes[2][2]}};
    fcl::CollisionObjectf collisionObject2{teoAux1, tfTest};

    
    m_boxSizes.push_back(std::array<float, 3>{0.4, 0.3, 0.435});
    CollisionGeometryPtr_t teoTrunk{new fcl::Boxf{m_boxSizes[1][0], m_boxSizes[1][1], m_boxSizes[1][2]}};
    fcl::CollisionObjectf collisionObject1{teoTrunk, tfTest};


    m_boxSizes.push_back(std::array<float, 3>{0.0, 0.0, 0.0});
    fcl::CollisionObjectf collisionObject3{teoAux1, tfTest};

    

    m_boxSizes.push_back(std::array<float, 3>{AXIAL_SHOULDER_LINK_RADIUS,AXIAL_SHOULDER_LINK_RADIUS,AXIAL_SHOULDER_LINK_LENGTH});
    CollisionGeometryPtr_t teoAxialShoulder{new fcl::Boxf{m_boxSizes[2][0], m_boxSizes[2][1], m_boxSizes[2][2]}};//{new fcl::Box{0.15, 0.15, 0.32901}};
    fcl::CollisionObjectf collisionObject4{teoAxialShoulder, tfTest};

    m_boxSizes.push_back(std::array<float, 3>{0.0, 0.0, 0.0});
    fcl::CollisionObjectf collisionObject5{teoAux1, tfTest};

    m_boxSizes.push_back(std::array<float, 3>{FRONTAL_ELBOW_LINK_RADIUS, FRONTAL_ELBOW_LINK_RADIUS, FRONTAL_ELBOW_LINK_LENGTH});
    CollisionGeometryPtr_t teoElbow{new fcl::Boxf{m_boxSizes[3][0], m_boxSizes[3][1], m_boxSizes[3][2]}};
    fcl::CollisionObjectf collisionObject6{teoElbow, tfTest};
    
    m_boxSizes.push_back(std::array<float, 3>{0.0, 0.0, 0.0});
    fcl::CollisionObjectf collisionObject8{teoAux1, tfTest};

    m_boxSizes.push_back(std::array<float, 3>{0.1, 0.07, 0.07});
    CollisionGeometryPtr_t teoWrist{new fcl::Boxf{m_boxSizes[4][0], m_boxSizes[4][1], m_boxSizes[4][2]}};
    fcl::CollisionObjectf collisionObject7{teoWrist, tfTest};


    m_boxSizes.push_back(std::array<float, 3>{FRONTAL_WRIST_LINK_WIDTH, FRONTAL_WRIST_LINK_HEIGHT, FRONTAL_WRIST_LINK_LENGTH});
    CollisionGeometryPtr_t endEffector{new fcl::Boxf{m_boxSizes[5][0], m_boxSizes[5][1], m_boxSizes[5][2]}};
    fcl::CollisionObjectf collisionObject9{endEffector, tfTest};


    CollisionGeometryPtr_t table{new fcl::Boxf{0.6,1.3,0.95}};
    fcl::CollisionObjectf collisionObjectTable{table, tfTest};
    fcl::Quaternionf rotation(1.0,0.0,0.0,0.0);
    // fcl::Vector3f translation(1.65, 0.0, -0.43);
    fcl::Vector3f translation(0.77, 0.0, -0.43);
    collisionObjectTable.setTransform(rotation, translation);
    m_tableCollision.clear();
    m_tableCollision.push_back(collisionObjectTable);

    int nOfCollisionObjects = 10;
    m_collisionObjects.clear();
    m_collisionObjects.reserve(nOfCollisionObjects);
    m_collisionObjects.emplace_back(collisionObject0);
    m_collisionObjects.emplace_back(collisionObject2);
    m_collisionObjects.emplace_back(collisionObject1);
    m_collisionObjects.emplace_back(collisionObject3);
    m_collisionObjects.emplace_back(collisionObject4);
    m_collisionObjects.emplace_back(collisionObject5);
    m_collisionObjects.emplace_back(collisionObject6);
    m_collisionObjects.emplace_back(collisionObject8);
    m_collisionObjects.emplace_back(collisionObject7);
    m_collisionObjects.emplace_back(collisionObject9);

    yInfo()<<"Collision objects created";
    m_offsetCollisionObjects.reserve(nOfCollisionObjects);
    std::array<float, 3> offsetObject = {0, 0, 0};
    for (int i = 0; i < nOfCollisionObjects; i++)
    {
        m_offsetCollisionObjects.emplace_back(offsetObject);
    }

    m_offsetCollisionObjects[0][2] = -0.2;
    m_offsetCollisionObjects[2][0] = -0.16;
    m_offsetCollisionObjects[2][1] = 0.0;
    m_offsetCollisionObjects[2][2] = 0.345;
    m_offsetCollisionObjects[8][1] = 0.0;
    m_offsetCollisionObjects[8][0] = 0.08;

    yInfo()<<"offset collisions created";
    m_chain = makeTeoTrunkAndRightArmKinematicsFromDH();
    

    m_checkSelfCollision = new CheckSelfCollision(m_chain, m_qmin, m_qmax, m_collisionObjects, m_offsetCollisionObjects, m_tableCollision);
    yInfo()<<"Check selfCollisionObject created";

    m_rosNode = new yarp::os::Node("/collisionsVisualization");


    m_collisionObjectsTopic = new yarp::os::Publisher<yarp::rosmsg::visualization_msgs::MarkerArray>;

    if (m_collisionObjectsTopic->topic("/collisionObjects")==false)
    {
        yError("Error opening collisionObjects topic.\n");
    }
    else{
        yInfo("Opening collisionObjects topic succesfully.\n");
    }
    std::printf("--------------------------------------------------------------\n");

    yarp::os::Time::delay(1);

    return true;
}

bool CollisionsVisualization::openDevices(){
    
    yInfo() <<"Lets open the devices";
    
    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/"+m_robot+"/"+m_deviceName);
    options.put("local", "/" +m_robot + "/"+m_deviceName);
    m_device.open(options);
    if (!m_device.isValid())
    {
        yError() << "Robot "<<m_deviceName<<" device not available";
        m_device.close();
        yarp::os::Network::fini();
        return false;
    }

    // connecting our device with "IEncoders" interface
    if (!m_device.view(m_iEncoders))
    {
        yError() << "Problems acquiring IEncoders interface in "<<m_deviceName;
        return false;
    }
    else
    {
        yInfo() << "Acquired IEncoders interface in "<<m_deviceName;
        if (!m_iEncoders->getAxes(&m_numJoints))
            yError() << "Problems acquiring numJoints";
        else
            yWarning() << "Number of joints:" << m_numJoints;
    }

    if (!m_device.view(m_iControlLimits))
    {
        yError() << "Could not view iControlLimits in "<<m_deviceName;
        return false;
    }

    yInfo()<<"Devices open";
    return true;
}

/************************************************************************/

double CollisionsVisualization::getPeriod()
{
    return 0.5;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/

bool CollisionsVisualization::updateModule()
{
    yInfo()<<"Module update";

    // get the current encoder positions
    std::vector<double> currentQ(m_numJoints);
    if(!m_iEncoders->getEncoders(currentQ.data())){
        yError() << " Failed getEncoders() of "<<m_deviceName;
        return false;
    }
    // get encoders positions in a KDL::JntArray
    KDL::JntArray jointpositions = KDL::JntArray(m_numJoints);
    for(int i=0; i<m_numJoints; i++){
        jointpositions(i) = currentQ[i];
        yInfo()<<"Joint "<<i<<" position: "<<jointpositions(i);
    }

    // update collisions objects transformations
    m_checkSelfCollision->updateCollisionObjectsTransform(jointpositions);
   
    
    m_markerArray.clear();

    for(unsigned int i=0; i<m_collisionObjects.size(); i++){
        std::vector<double> transformation;
        m_checkSelfCollision->getTransformation(m_checkSelfCollision->collisionObjects[i], transformation);
        addMarker(i, transformation, m_boxSizes[i]);
    }

    if (m_collisionObjectsTopic) {
        yInfo("Publish...\n");
        m_collisionObjectsTopic->write(m_markerArray);
    }

}

bool CollisionsVisualization::addMarker(const int numberLink, const std::vector<double>& transformation, const std::array<float,3> & boxSize){
    yarp::rosmsg::visualization_msgs::Marker marker;

    marker.header.stamp = yarp::os::Time::now();
    yInfo()<<"Frame id: "<<m_frameId;
    marker.header.frame_id = m_frameId;
    marker.header.seq = numberLink;
    marker.id = numberLink;
    marker.action = yarp::rosmsg::visualization_msgs::Marker::ADD;
    marker.type = yarp::rosmsg::visualization_msgs::Marker::CUBE;
    yarp::rosmsg::geometry_msgs::Point p;
    p.x = transformation[4];
    p.y = transformation[5];
    p.z = transformation[6];
    marker.pose.position = p;
    marker.pose.orientation.x = transformation[0];
    marker.pose.orientation.y = transformation[1];
    marker.pose.orientation.z = transformation[2];
    marker.pose.orientation.w = transformation[3];

    marker.scale.x = boxSize[0];
    marker.scale.y = boxSize[1];
    marker.scale.z = boxSize[2];

    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.header.stamp = yarp::os::Time::now();
    m_markerArray.markers.push_back(marker);

    return true;
}
/************************************************************************/

bool CollisionsVisualization::interruptModule()
{
    return true;
}

/************************************************************************/
