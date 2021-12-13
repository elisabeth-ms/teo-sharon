// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TrajectoryGeneration.hpp"

#include <algorithm>

#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include <yarp/os/LogStream.h>
#include <KdlVectorConverter.hpp>

using namespace roboticslab::KinRepresentation;
using namespace roboticslab::KdlVectorConverter;
using namespace sharon;

#define DEFAULT_MAX_DIFF_INV 0.0000001
#define DEFAULT_PREFIX "/trajectoryGeneration/"
#define DEFAULT_DEVICE_NAME "trunkAndRightArm"
#define DEFAULT_KINEMATICS_CONFIG "teo-trunk-rightArm-fetch.ini"
#define DEFAULT_RANGE_RRT 2.0
#define DEFAULT_PRUNE_THRESHOLD 0.6
#define DEFAULT_MAX_PLANNER_TIME 2.0


std::vector<std::string>availablePlanners = {"RRTConnect", "RRTStar"};


namespace errorsTrajectoryGeneration{

  const std::string goal_not_inv_kin ("invKin failed for goal pose");
  const std::string goal_collision("robot collides at the goal configuration");
  const std::string pose_6_elements("pose list must have 6 elements (x, y, z, rotx, roty, rotz)");
  const std::string joints_elements("size of joints list is different than the number of joints");
  const std::string start_collision("robot collides at the start configuration");
  const std::string path_not_found("path NOT found");
  const std::string joints_outside_bounds("joints outside bounds");
  const std::string not_get_current_Q("could NOT get encoders values");
};


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

static KDL::Chain makeTeoTrunkAndLeftArmKinematicsFromDH()
{
    const KDL::Joint rotZ(KDL::Joint::RotZ);
    const KDL::Joint fixed(KDL::Joint::None); //Rigid Connection
    KDL::Chain chain;
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.0, -KDL::PI / 2, 0.1932, 0.0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(0.305, 0.0, 0.34692, -KDL::PI / 2)));
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

static KDL::Chain makeTeoFixedTrunkAndRightArmKinematicsFromDH(){
    const KDL::Joint rotZ(KDL::Joint::RotZ);
    const KDL::Joint fixed(KDL::Joint::None); //Rigid Connection

    KDL::Chain chain;
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0.0, -KDL::PI/2, 0.1932,0.0)));
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0.305, 0.0, 0.34692, -KDL::PI/2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(    0, -KDL::PI / 2,        0,            0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(    0, -KDL::PI / 2,        0, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(    0, -KDL::PI / 2, -0.32901, -KDL::PI / 2)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(    0,  KDL::PI / 2,        0,            0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(    0, -KDL::PI / 2,   -0.215,            0)));
    chain.addSegment(KDL::Segment(rotZ, KDL::Frame::DH(-0.09,            0,        0, -KDL::PI / 2)));

    return chain;
}

/************************************************************************/

static KDL::Chain makeTeoFixedTrunkAndLeftArmKinematicsFromDH(){
    const KDL::Joint rotZ(KDL::Joint::RotZ);
    const KDL::Joint fixed(KDL::Joint::None); //Rigid Connection

    KDL::Chain chain;
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0.0, -KDL::PI / 2, 0.1932, 0.0)));
    chain.addSegment(KDL::Segment(fixed, KDL::Frame::DH(0.305, 0.0, 0.34692, -KDL::PI / 2)));
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

void TrajectoryGeneration::changeJointsLimitsFromConfigFile(KDL::JntArray & qlim, const yarp::os::Searchable& config, const std::string& mode){
    if(!config.check(mode))
    {
        yInfo()<<"q"<<mode<<"limits NOT defined in config file. Continue with the limits defined in the IControlLimits devices.";
    }else{
        yarp::os::Bottle * qConfig = config.find(mode).asList();
        for(int i=0; i<numArmJoints+numTrunkJoints; i++){
            if(qConfig->get(i).asString()!= "D" && !qConfig->get(i).isNull()){
                qlim(i) =std::stof(qConfig->get(i).asString());
            }
        }
    }

}


/************************************************************************/

bool TrajectoryGeneration::open(yarp::os::Searchable& config)
{
    robot = config.check("robot", yarp::os::Value(DEFAULT_ROBOT), "name of /robot to be used").asString();
    planningSpace = config.check("planningSpace", yarp::os::Value(DEFAULT_PLANNING_SPACE), "planning space").asString();
    deviceName = config.check("deviceName", yarp::os::Value(DEFAULT_DEVICE_NAME), "device name").asString();
    kinematicsConfig = config.check("kinematicsConfig", yarp::os::Value(DEFAULT_KINEMATICS_CONFIG), "kinematics config").asString();
    plannerType = config.check("planner", yarp::os::Value(DEFAULT_PLANNER), "planner type").asString();
    plannerRange = config.check("plannerRange", yarp::os::Value(DEFAULT_RANGE_RRT), "range the planner is supposed to use").asFloat64();
    pruneThreshold = config.check("pruneThreshold", yarp::os::Value(DEFAULT_PRUNE_THRESHOLD), "prune theshold used for RRTStar").asFloat64();
    maxPlannerTime = config.check("maxPlannerTime", yarp::os::Value(DEFAULT_MAX_PLANNER_TIME), "seconds the algorithm is allowed to spend planning").asFloat64();
    prefix = "/trajectoryGeneration/"+deviceName;
    printf("TrajectoryGeneration using robot: %s\n",robot.c_str());
    printf("TrajectoryGeneration using planningSpace: %s\n",planningSpace.c_str());
    printf("TrajectoryGeneration using deviceName: %s\n",deviceName.c_str());
    printf("TrajectoryGeneration using kinematicsConfig: %s\n",kinematicsConfig.c_str());
    printf("TrajectoryGeneration using plannerType: %s\n", plannerType.c_str());
    printf("TrajectoryGeneration using plannerRange: %f\n", plannerRange);
    printf("TrajectoryGeneration using pruneThreshold: %f\n", pruneThreshold);


    printf("--------------------------------------------------------------\n");
    if (config.check("help"))
    {
        printf("TrajectoryGeneration options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--robot: %s [%s]\n", robot.c_str(), DEFAULT_ROBOT);
        ::exit(0);
    }

    openDevices();


    //  Getting the limits of each joint
    printf("---- Joint limits of %s and %s----\n",trunkDeviceName.c_str(), rightArmDeviceName.c_str());
    qmin.resize(numTrunkJoints+numArmJoints);
    qmax.resize(numTrunkJoints+numArmJoints);

    for(unsigned int joint = 0; joint < numTrunkJoints; joint++){
        double min, max;
        trunkIControlLimits->getLimits(joint, &min, &max);
        qmin(joint) = min;
        qmax(joint) = max;
    }

    for (unsigned int joint = 0; joint < numArmJoints; joint++)
    {
        double min, max;
        armIControlLimits->getLimits(joint, &min, &max);
        qmin(joint+numTrunkJoints) = min;
        qmax(joint+numTrunkJoints) = max;
    }

    changeJointsLimitsFromConfigFile(qmin,config, "qmin");
    changeJointsLimitsFromConfigFile(qmax,config, "qmax");


    for(unsigned int joint =0; joint<numTrunkJoints+numArmJoints; joint++){
        yInfo("Joint %d limits: [%f,%f]", joint, qmin(joint), qmax(joint));
        qrMin.addFloat64(qmin(joint));
        qrMax.addFloat64(qmax(joint));
    }

    // ----- Configuring KDL Solver for device -----

    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("kinematics"); // context to find kinematic config files
    std::string kinPath = rf.findFileByName(kinematicsConfig);
    armSolverOptions.fromConfigFile(kinPath);
    armSolverOptions.put("device", "KdlSolver");
    armSolverOptions.put("mins", yarp::os::Value::makeList(qrMin.toString().c_str()));
    armSolverOptions.put("maxs", yarp::os::Value::makeList(qrMax.toString().c_str()));
    armSolverOptions.put("ik", "nrjl"); // to use screw theory IK
    armSolverOptions.put("maxIter", 1000);
    armSolverDevice.open(armSolverOptions);
    if (!armSolverDevice.isValid())
    {
        yError() << "KDLSolver solver device for "<<deviceName<<" is not valid";
        return false;
    }

    if (!armSolverDevice.view(armICartesianSolver))
    {
        yError() << "Could not view iCartesianSolver in KDLSolver";
        return false;
    }

    yInfo() << "Acquired armICartesianSolver interface";

    yInfo() << "Running";
    std::vector<double> position;

    if (!rpcServer.open(prefix + "/rpc:s"))
    {
        yError() << "Unable to open RPC server port" << rpcServer.getName();
        return false;
    }
    

    //Init collisions objects
    if(deviceName == "trunkAndRightArm")
        chain = makeTeoTrunkAndRightArmKinematicsFromDH();
    else if (deviceName == "rightArm")
    {
        chain = makeTeoFixedTrunkAndRightArmKinematicsFromDH();
    }
    else if(deviceName == "trunkAndLeftArm")
    {
        chain = makeTeoTrunkAndLeftArmKinematicsFromDH();
    }
    else if(deviceName == "leftArm"){
        chain = makeTeoFixedTrunkAndLeftArmKinematicsFromDH();
    }
    else{
        yError()<<"Invalid deviceName. Options: trunkAndRightArm, rightArm, trunkAndLeftArm, leftArm";
    }

    unsigned int nj = chain.getNrOfJoints();
    yInfo() << "Number of joints: " << nj;
    unsigned int nl = chain.getNrOfSegments();
    yInfo() << "Number of segments: " << nl;
   
    yInfo() <<"Lets create the collision objects";
    CollisionGeometryPtr_t teoRootTrunk{new fcl::Boxf{0.24, 0.24, 0.5}};
    fcl::Transform3f tfTest;
    fcl::CollisionObjectf collisionObject1{teoRootTrunk, tfTest};

    CollisionGeometryPtr_t teoTrunk{new fcl::Boxf{0.3, 0.3, 0.46}};
    fcl::CollisionObjectf collisionObject2{teoTrunk, tfTest};
   
    CollisionGeometryPtr_t teoAxialShoulder{new fcl::Boxf{AXIAL_SHOULDER_LINK_RADIUS,AXIAL_SHOULDER_LINK_RADIUS,AXIAL_SHOULDER_LINK_LENGTH}};//{new fcl::Box{0.15, 0.15, 0.32901}};
    fcl::CollisionObjectf collisionObject3{teoAxialShoulder, tfTest};

    CollisionGeometryPtr_t teoElbow{new fcl::Boxf{FRONTAL_ELBOW_LINK_RADIUS, FRONTAL_ELBOW_LINK_RADIUS, FRONTAL_ELBOW_LINK_LENGTH}};
    fcl::CollisionObjectf collisionObject4{teoElbow, tfTest};
    
    CollisionGeometryPtr_t teoWrist{new fcl::Boxf{0.19, 0.29, 0.19}};
    fcl::CollisionObjectf collisionObject5{teoWrist, tfTest};

    CollisionGeometryPtr_t endEffector{new fcl::Boxf{0.0,0.0,0.0}};
    fcl::CollisionObjectf collisionObject6{endEffector, tfTest};


    CollisionGeometryPtr_t table{new fcl::Boxf{0.8,0.8,1.0}};
    fcl::CollisionObjectf collisionObjectTable{table, tfTest};
    fcl::Quaternionf rotation(1.0,0.0,0.0,0.0);
    // fcl::Vector3f translation(1.65, 0.0, -0.43);
    fcl::Vector3f translation(0.65, 0.0, -0.43);
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

    yInfo()<<"Collision objects created";
    offsetCollisionObjects.reserve(nOfCollisionObjects);
    std::array<float, 3> offsetObject = {0, 0, 0};
    for (int i = 0; i < nOfCollisionObjects; i++)
    {
        offsetCollisionObjects.emplace_back(offsetObject);
    }

    offsetCollisionObjects[0][2] = -0.2;
    offsetCollisionObjects[1][1] = 0.0;
    offsetCollisionObjects[1][2] = +0.1734;
    offsetCollisionObjects[4][1] = 0.055;

    yInfo()<<"offset collisions created";

    checkSelfCollision = new CheckSelfCollision(chain, qmin, qmax, collisionObjects, offsetCollisionObjects, tableCollision);
    yInfo()<<"Check selfCollisionObject created";
    
    
    if(planningSpace == "cartesian"){ // cartesian space
        space = ob::StateSpacePtr(new ob::SE3StateSpace());
        ob::RealVectorBounds bounds{3};

        bounds.setLow(0, -0.0);
        bounds.setHigh(0, 0.6);

        bounds.setLow(1, -0.8);
        bounds.setHigh(1, 0.2);

        bounds.setLow(2, -0.1);
        bounds.setHigh(2, 0.6);

        space->as<ob::SE3StateSpace>()->setBounds(bounds);
    
        //space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

        si->setStateValidityChecker(std::bind(&TrajectoryGeneration::isValid, this, std::placeholders::_1));
        si->setup();

        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
    }
    else{// joint space
        space = ob::StateSpacePtr(new ob::RealVectorStateSpace(nj));
        ob::RealVectorBounds bounds{nj};
        for(unsigned int j=0; j<nj; j++){
            bounds.setLow(j, qmin(j)+MARGIN_BOUNDS);
            bounds.setHigh(j, qmax(j)-MARGIN_BOUNDS);
        }
        space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

        si->setStateValidityChecker(std::bind(&TrajectoryGeneration::isValid, this, std::placeholders::_1));
        si->setup();

        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
    }

    bool availablePlannerSelected = false;
    for (unsigned i=0; i<availablePlanners.size(); i++)
        if(plannerType == availablePlanners[i]){
            availablePlannerSelected = true;
            break;
        }
    if (!availablePlannerSelected)
    {
        yWarning()<<"Selected NOT available planner. Using default planner: "<<DEFAULT_PLANNER;
        plannerType = DEFAULT_PLANNER;
    }


    if (plannerType == "RRTConnect"){
        auto plannerRRT = (new og::RRTConnect(si));
        plannerRRT->setRange(plannerRange);
        planner = ob::PlannerPtr(plannerRRT);
    }else if (plannerType == "RRTStar")
    {
        auto plannerRRT = (new og::RRTstar(si));
        plannerRRT->setRange(plannerRange);
        plannerRRT->setPruneThreshold(1.0);
        plannerRRT->setTreePruning(true);
        plannerRRT->setInformedSampling(true);
        planner = ob::PlannerPtr(plannerRRT);
    }

    

   
    rpcServer.setReader(*this);

    return true;
}

bool TrajectoryGeneration::openDevices(){
    trunkDeviceName = "trunk";
    yarp::os::Property trunkOptions;
    trunkOptions.put("device", "remote_controlboard");
    trunkOptions.put("remote", "/"+robot+"/"+trunkDeviceName);
    trunkOptions.put("local", "/" +robot + "/"+trunkDeviceName);
    trunkDevice.open(trunkOptions);
    if (!trunkDevice.isValid())
    {
        yError() << "Robot "<<trunkDeviceName<<" device not available";
        trunkDevice.close();
        yarp::os::Network::fini();
        return false;
    }

    // connecting our device with "IEncoders" interface
    if (!trunkDevice.view(trunkIEncoders))
    {
        yError() << "Problems acquiring IEncoders interface in "<<trunkDeviceName;
        return false;
    }
    else
    {
        yInfo() << "Acquired IEncoders interface in "<<trunkDeviceName;
        if (!trunkIEncoders->getAxes(&numTrunkJoints))
            yError() << "Problems acquiring numTrunkJoints";
        else
            yWarning() << "Number of joints:" << numTrunkJoints;
    }

    if (!trunkDevice.view(trunkIControlLimits))
    {
        yError() << "Could not view iControlLimits in "<<trunkDeviceName;
        return false;
    }

 // connecting our device with "control mode" interface, initializing which control mode we want (position)
    if (!trunkDevice.view(trunkIControlMode))
    {
        yError() << "Problems acquiring trunkIControlMode interface";
        return false;
    }
    else
        yInfo() << "Acquired trunkIControlMode interface";

    // connecting our device with "PositionControl" interface
    if (!trunkDevice.view(trunkIPositionControl))
    {
        yError() << "Problems acquiring trunkIPositionControl interface";
        return false;
    }
    else
        yInfo() << "Acquired trunkIPositionControl interface";


    // ------  ARM DEVICE -------
    rightArmDeviceName = "rightArm";
    yarp::os::Property armOptions;
    armOptions.put("device", "remote_controlboard");
    armOptions.put("remote", "/"+robot+"/"+rightArmDeviceName);
    armOptions.put("local",  "/" +robot + "/"+rightArmDeviceName);
    armDevice.open(armOptions);
    if (!armDevice.isValid())
    {
        yError() << "Robot "<<rightArmDeviceName<<" device not available";
        armDevice.close();
        yarp::os::Network::fini();
        return false;
    }

    // connecting our device with "IEncoders" interface
    if (!armDevice.view(armIEncoders))
    {
        yError() << "Problems acquiring IEncoders interface in "<<rightArmDeviceName;
        return false;
    }
    else
    {
        yInfo() << "Acquired IEncoders interface in "<<rightArmDeviceName;
        if (!armIEncoders->getAxes(&numArmJoints))
            yError() << "Problems acquiring numArmJoints";
        else
            yWarning() << "Number of joints:" << numArmJoints;
    }

    if (!armDevice.view(armIControlLimits))
    {
        yError() << "Could not view iControlLimits in "<<rightArmDeviceName;
        return false;
    }
     // connecting our device with "control mode" interface, initializing which control mode we want (position)
    if (!armDevice.view(armIControlMode))
    {
        yError() << "Problems acquiring armIControlMode interface";
        return false;
    }
    else
        yInfo() << "Acquired armIControlMode interface";

    // connecting our device with "PositionControl" interface
    if (!armDevice.view(armIPositionControl))
    {
        yError() << "Problems acquiring armIPositionControl interface";
        return false;
    }
    else
        yInfo() << "Acquired armIPositionControl interface";
    
    yInfo()<<"Devices open";
    return true;
}


bool TrajectoryGeneration::getCurrentQ(std::vector<double> & currentQ){


        std::vector<double> currentQTrunk(numTrunkJoints);
        if (!trunkIEncoders->getEncoders(currentQTrunk.data()))
        {
            yError() << "Failed getEncoders() of "<<trunkDeviceName;
            return false;
        }


        std::vector<double> currentQArm(numArmJoints);
        if (!armIEncoders->getEncoders(currentQArm.data()))
        {
            yError() << "Failed getEncoders() of "<<rightArmDeviceName;
            return false;
        }

        for(unsigned int j = 0; j< numTrunkJoints; j++)
            currentQ[j] = currentQTrunk[j];
        for(unsigned int j = 0; j< numArmJoints; j++)
            currentQ[j+numTrunkJoints] = currentQArm[j];    
        return true;

}


/************************************************************************/
bool TrajectoryGeneration::isValid(const ob::State *state)
{

    if(planningSpace == "cartesian"){
        // cast the abstract state type to the type we expect
        const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);


        KDL::Frame frame;

        KDL::Rotation rotKdl = KDL::Rotation::Quaternion(rot->x, rot->y, rot->z, rot->w);
        KDL::Vector posKdl = KDL::Vector(pos->values[0], pos->values[1], pos->values[2]);
        double x, y, z, w;
        rotKdl.GetQuaternion(x, y, z, w);

        frame.M = rotKdl;
        frame.p = posKdl;
        std::vector<double> testAxisAngle = frameToVector(frame);

        std::vector<double> currentQ(numArmJoints+numTrunkJoints);


        // Check if we are in the starting state

        std::vector<double> xStart;

        if (!armICartesianSolver->fwdKin(currentQ, xStart))
        {
            yError() << "fwdKin failed";
            return false;
        }


        bool computeInvKin = true;

        double diffSumSquare = 0;
        for (int i = 0; i < 6; i++)
        {
            diffSumSquare += (xStart[i] - testAxisAngle[i]) * (xStart[i] - testAxisAngle[i]);
        }


        if (diffSumSquare < DEFAULT_MAX_DIFF_INV)
        {
            computeInvKin = false;
        }

        std::vector<double> desireQ(numArmJoints+numTrunkJoints);


        if (computeInvKin)
        {
            if (!armICartesianSolver->invKin(testAxisAngle, currentQ, desireQ))
            {
                yError() << "invKin() failed";
                return false;
            }
        
        
            KDL::JntArray jointpositions = KDL::JntArray(8);
            goalQ.clear();

            for (unsigned int i = 0; i < jointpositions.rows(); i++)
            {
                jointpositions(i) = desireQ[i];
                goalQ.push_back(desireQ[i]);
            }
            
            yInfo()<<"desireQ: "<<desireQ;

            checkSelfCollision->updateCollisionObjectsTransform(jointpositions);
            bool selfCollide = checkSelfCollision->selfCollision();
            return !selfCollide;
        }
        return true;
    }
    else{ // joint space
        
        const ob::RealVectorStateSpace::StateType *jointState = state->as<ob::RealVectorStateSpace::StateType>();
        KDL::JntArray jointpositions = KDL::JntArray(numArmJoints+numTrunkJoints);

        for (unsigned int i = 0; i < jointpositions.rows(); i++)
        {
            jointpositions(i) = jointState->values[i];
            // yInfo()<<"Joint("<<i<<")="<<jointpositions(i);
        }
        
        jointsInsideBounds = checkSelfCollision->updateCollisionObjectsTransform(jointpositions);
        
        if(jointsInsideBounds){
            // yInfo()<<"eo: selfCollision"<<checkSelfCollision->selfCollision();
            return !checkSelfCollision->selfCollision();
        }
        else{
            // yInfo()<<"eo: joints outside bounds";
            return false;
        }
    }
}

bool TrajectoryGeneration::computeDiscretePath(ob::ScopedState<ob::SE3StateSpace> start, ob::ScopedState<ob::SE3StateSpace> goal, std::vector<std::vector<double>> &jointsTrajectory, bool &validStartState, bool &validGoalState)
{

    // pdef->clearStartStates();
    // pdef->addStartState(start);

    auto plannerRRT = (new og::RRTstar(si));
    plannerRRT->setRange(0.05);
    plannerRRT->setPruneThreshold(0.1);
    plannerRRT->setTreePruning(true);
    plannerRRT->setInformedSampling(true);


    ob::State *startState = pdef->getStartState(0);

  
    // if (collide(startState))
    //     yInfo("Start state collides");
    // else
    // {
    //     yInfo("Start state doesn't collide");
    // }

    if (isValid(startState)){
        yInfo() << "Valid starting state";
        validStartState = true;
    }
    else
    {
        yInfo() << "Not valid starting state";
        validStartState = false;
        return false;
    }

    pdef->clearGoal();

    pdef->setGoalState(goal);

    ob::State *goalState = pdef->getGoal()->as<ob::GoalState>()->getState();


    // if (collide(goalState))
    //     yInfo("Goal state collides");
    // else
    // {
    //     yInfo("Goal state doesn't collide");
    // }

    if (isValid(goalState)){
        yInfo() << "Valid goal state";
        validGoalState = true;
    }
    else
    {
        yInfo() << "Not valid goal state";
        validGoalState = false;
        return false;
    }

    planner->clear();
    planner->setProblemDefinition(pdef);

    planner->setup();

    bool solutionFound = planner->solve(maxPlannerTime);

    if (solutionFound == true)
    {
        yInfo() << "Sol";
        ob::PathPtr path = pdef->getSolutionPath();
        yInfo() << "Found discrete path from start to goal";
        path->print(std::cout);
        pth = path->as<og::PathGeometric>();
        yInfo() << pth->getStateCount();
    }


    std::size_t iState = 0;

    while (iState < pth->getStateCount())
    {
        ob::State *state = pth->getState(iState);

        const ob::SE3StateSpace::StateType *se3state = state->as<ob::SE3StateSpace::StateType>();

        // extract the first component of the state and cast it to what we expect
        const ob::RealVectorStateSpace::StateType *pos = se3state->as<ob::RealVectorStateSpace::StateType>(0);

        // extract the second component of the state and cast it to what we expect
        const ob::SO3StateSpace::StateType *rot = se3state->as<ob::SO3StateSpace::StateType>(1);

        KDL::Frame frame;

        KDL::Rotation rotKdl = KDL::Rotation::Quaternion(rot->x, rot->y, rot->z, rot->w);
        KDL::Vector posKdl = KDL::Vector(pos->values[0], pos->values[1], pos->values[2]);
        //frame.M.Quaternion(rot->x, rot->y, rot->z, rot->w);
        double x, y, z, w;
        rotKdl.GetQuaternion(x, y, z, w);
        frame.M = rotKdl;
        frame.p = posKdl;
        std::vector<double> pose = frameToVector(frame);
        
        std::vector<double> currentQ(numArmJoints+numTrunkJoints);

        if(!getCurrentQ(currentQ)){
            return false;
        }


        bool computeInvKin = true;
        std::vector<double>xStart;
        double diffSumSquare = 0;
        for (int i = 0; i < 6; i++)
        {
            diffSumSquare += (xStart[i] - pose[i]) * (xStart[i] - pose[i]);
        }

        yInfo() << "diffSumSquare: " << diffSumSquare;

        if (diffSumSquare < DEFAULT_MAX_DIFF_INV)
        {
            yInfo() << "Start state-> we do not calculate the inverse kinematics";
            computeInvKin = false;
        }

        std::vector<double> desireQ(numArmJoints+numTrunkJoints);

        if (computeInvKin)
        {
            if (!armICartesianSolver->invKin(pose, currentQ, desireQ))
            {
                yError() << "invKin() failed";
            }
            else{
                Bottle bPose;
                for(int i = 0; i < 6; i++){
                    bPose.addFloat64(pose[i]);
                }
                std::vector<double> jointsPosition;
                jointsPosition.reserve(numArmJoints+numTrunkJoints);
                for(int i = 0; i<numArmJoints+numTrunkJoints; i++){
                    jointsPosition.emplace_back(desireQ[i]);
                }
                jointsTrajectory.push_back(jointsPosition);
            }
        }
        
        iState++;
    }

//#endif
    // followDiscretePath();

    return solutionFound;
}


bool TrajectoryGeneration::computeDiscretePath(ob::ScopedState<ob::RealVectorStateSpace> start, ob::ScopedState<ob::RealVectorStateSpace> goal, std::vector<std::vector<double>> &jointsTrajectory,
                                                std::string & errorMessage)
{

    pdef->clearStartStates();
    pdef->addStartState(start);

    start.print();

    ob::RealVectorBounds bounds = space->as<ob::RealVectorStateSpace>()->getBounds();
    for(unsigned int j=0; j<numArmJoints+numTrunkJoints; j++){
        yInfo()<<"Low: "<<bounds.low[j]<<" high: "<<bounds.high[j];
    }



    ob::State *startState = pdef->getStartState(0);

  
   yInfo()<<"Is startState valid?";
    if (!isValid(startState)){
        yInfo()<<"Not valid start state";
        if (si->satisfiesBounds(startState))
            errorMessage = errorsTrajectoryGeneration::start_collision;
        else{
            errorMessage = errorsTrajectoryGeneration::joints_outside_bounds;
        }
        return false;
    }

    pdef->clearGoal();

    pdef->setGoalState(goal);

    ob::State *goalState = pdef->getGoal()->as<ob::GoalState>()->getState();

   yInfo()<<"Is goalState valid?";

    if (!isValid(goalState)){
        yInfo()<<"Not valid goal state";
        if (si->satisfiesBounds(goalState))
            errorMessage = errorsTrajectoryGeneration::goal_collision;
        else{
            errorMessage = errorsTrajectoryGeneration::joints_outside_bounds;
        }
        return false;
    }
    yInfo()<<"goalState VALID";


    
    pdef->print();
    planner->clear();
    planner->printProperties(std::cout);
    planner->printSettings(std::cout);
    planner->setProblemDefinition(pdef);
    yInfo()<<"setProblemDefinition done";

    planner->setup();
    yInfo()<<"setup done";
    pdef->clearSolutionPaths();
    yInfo()<<"clearSolutionPaths done";


    yInfo()<<"solve";
    bool solutionFound = planner->solve(maxPlannerTime);
    printf("Solution Found: %d", solutionFound);

    if (solutionFound)
    {
        yInfo() << "Sol";
        ob::PathPtr path = pdef->getSolutionPath();
        yInfo() << "Found discrete path from start to goal";
        path->print(std::cout);
        pth = path->as<og::PathGeometric>();
        yInfo() << pth->getStateCount();

        std::size_t iState = 0;

        while (iState < pth->getStateCount())
        {
            ob::State *state = pth->getState(iState);

            const ob::RealVectorStateSpace::StateType *jointState = state->as<ob::RealVectorStateSpace::StateType>();
            
            std::vector<double> poseQ;
            poseQ.reserve(numArmJoints+numTrunkJoints);
            for(unsigned int j=0; j<numArmJoints+numTrunkJoints; j++)
                poseQ.emplace_back(jointState->values[j]);

            jointsTrajectory.push_back(poseQ);
            yInfo()<<poseQ;
            
            iState++;
        }
        return true;
    }else{
        errorMessage == errorsTrajectoryGeneration::path_not_found;
        return false;
    }


}

bool TrajectoryGeneration::checkGoalPose(yarp::os::Bottle * bGoal, std::vector<double> & desireQ, std::string & errorMessage){
    yInfo()<<"Check goal pose";
    std::vector<double> xGoal(6);
    if (bGoal->size() != 6){
        errorMessage = errorsTrajectoryGeneration::pose_6_elements;
        return false;
    }
    for (int i = 0; i < 6; i++)
        xGoal[i] = bGoal->get(i).asFloat64();
    yInfo() <<"Goal: "<< xGoal[0] << " " << xGoal[1] << " " << xGoal[2] << " " << xGoal[3] << " " << xGoal[4] << " " << xGoal[5];

    std::vector<double> currentQ(numArmJoints+numTrunkJoints);

    if(!getCurrentQ(currentQ)){
        errorMessage = errorsTrajectoryGeneration::not_get_current_Q;
        return false;
    }
    else{
        // Lests add the start state, just for checking
        ob::ScopedState<ob::RealVectorStateSpace> start(space);
        for(unsigned int j=0; j<numArmJoints+numTrunkJoints; j++){
            start[j] = currentQ[j];
        }

        pdef->clearStartStates();
        pdef->addStartState(start);

        start.print();


        ob::State *startState = pdef->getStartState(0);

  
        if (!isValid(startState)){
            if (si->satisfiesBounds(startState))
                errorMessage = errorsTrajectoryGeneration::start_collision;
            else{
                errorMessage = errorsTrajectoryGeneration::joints_outside_bounds;
            }
            return false;
        }


        if (!armICartesianSolver->invKin(xGoal, currentQ, desireQ))
        {
            errorMessage = errorsTrajectoryGeneration::goal_not_inv_kin;
            return false;
        }
        else{   
            ob::ScopedState<ob::RealVectorStateSpace> goal(space);
            for(unsigned int j=0; j<numArmJoints+numTrunkJoints; j++){
                goal[j] = desireQ[j];
            }
            pdef->clearGoal();

            pdef->setGoalState(goal);

            ob::State *goalState = pdef->getGoal()->as<ob::GoalState>()->getState();
            if(isValid(goalState)){
                return true;
            }
            else{
                if (si->satisfiesBounds(goalState))
                    errorMessage = errorsTrajectoryGeneration::goal_collision;
                else{
                    errorMessage = errorsTrajectoryGeneration::joints_outside_bounds;
                }
                return false;
            }
        }    
    }
}

bool TrajectoryGeneration::checkGoalJoints(yarp::os::Bottle * bGoal, std::string & errorMessage){
    if (bGoal->size() != numArmJoints+ numTrunkJoints){
        yWarning()<<errorsTrajectoryGeneration::joints_elements;
        errorMessage = errorsTrajectoryGeneration::joints_elements;
        return false;
    }
    else{
        ob::ScopedState<ob::RealVectorStateSpace> goal(space);
        for(unsigned int j=0; j<numArmJoints+numTrunkJoints; j++){
            goal[j] = bGoal->get(j).asFloat64();
        }
        pdef->clearGoal();

        pdef->setGoalState(goal);

        ob::State *goalState = pdef->getGoal()->as<ob::GoalState>()->getState();
        if(isValid(goalState)){
            return true;
        }
        else{
            if (si->satisfiesBounds(goalState))
                errorMessage = errorsTrajectoryGeneration::goal_collision;
            else{
                errorMessage = errorsTrajectoryGeneration::joints_outside_bounds;
            }
            return false;
        }
    }
}


bool TrajectoryGeneration::read(yarp::os::ConnectionReader &connection)
{
    yarp::os::Bottle reply;
    auto *writer = connection.getWriter();


    // if (!yarp::os::NetworkBase::exists("/"+robot+"/"+trunkDeviceName+"/rpc:i") || !yarp::os::NetworkBase::exists("/"+robot+"/"+rightArmDeviceName+"/rpc:i"))
    // {
    //     yError()<<"Check if the device "<<trunkDeviceName<<" is open!";
    //     // configure(resourceFinder);
    //     reply.addString("Device not open");
    //     return reply.write(*writer);
    // }
    // else{
    //     if(!yarp::os::NetworkBase::isConnected("/"+robot+"/"+trunkDeviceName+"/rpc:o", "/"+robot+"/"+trunkDeviceName+"/rpc:i") ||
    //         !yarp::os::NetworkBase::isConnected("/"+robot+"/"+rightArmDeviceName+"/rpc:o", "/"+robot+"/"+rightArmDeviceName+"/rpc:i") )
    //     {
    //         yWarning()<<"Device not connected";
    //         armDevice.close();
    //         trunkDevice.close();
    //         openDevices();
    //         yInfo()<<"Wait a 2 seconds...";
    //         yarp::os::Time::delay(4.0);
            
    //     }
    // }

    // if (!yarp::os::NetworkBase::exists("/"+robot+"/"+rightArmDeviceName+"/rpc:i"))
    // {
    //     yError()<<"Check if the device "<<rightArmDeviceName<<" is open!";
    //     // configure(resourceFinder);
    //     reply.addString("Device not open");
    //     return reply.write(*writer);
    // }
    // else{
    //     if(!yarp::os::NetworkBase::isConnected("/"+robot+"/"+rightArmDeviceName+"/rpc:o", "/"+robot+"/"+rightArmDeviceName+"/rpc:i"))
    //     {
    //         yWarning()<<"Device not connected";
    //         rightArmDeviceName = "rightArm";
    //         openDevices();

    //         reply.addString("Open device");
    //         return reply.write(*writer);
    //     }
    // }


    yarp::os::Bottle command;
    if (!command.read(connection) || writer == nullptr)
    {
        return false;
    }
    yInfo() << "command:" << command.toString();



    std::vector<double> currentQ(numArmJoints+numTrunkJoints);

     
    if (planningSpace=="cartesian"){ //TODO: Cartesian
        std::vector<double> xStart;
        if(!getCurrentQ(currentQ)){
            return false;
        }

        if (!armICartesianSolver->fwdKin(currentQ, xStart))
        {
            yError() << "fwdKin failed";
            return false;
        }
        
        yInfo() << "Start:" << xStart[0] << " " << xStart[1] << " " << xStart[2] << " " << xStart[3] << " " << xStart[4] << " " << xStart[5];
        armSolverOptions.unput("mins");
        armSolverOptions.unput("maxs");
        yarp::os::Bottle qMin, qMax;
        qMin.addFloat64(currentQ[0]-2.5);
        qMin.addFloat64(currentQ[1]-2.5);
        qMax.addFloat64(currentQ[0]+0.5);
        qMax.addFloat64(currentQ[1]+0.5);
        for(int i=2; i<currentQ.size(); i++){
            qMin.addFloat64(qrMin.get(i).asFloat64());
            qMax.addFloat64(qrMax.get(i).asFloat64());
        }
        armSolverOptions.put("mins", yarp::os::Value::makeList(qMin.toString().c_str()));
        armSolverOptions.put("maxs", yarp::os::Value::makeList(qMax.toString().c_str()));
        armSolverDevice.close();

        armSolverDevice.open(armSolverOptions);
        if (!armSolverDevice.isValid())
        {
            yError() << "KDLSolver solver device for "<<deviceName<<" is not valid";
            return false;
        }


        if (!armSolverDevice.view(armICartesianSolver))
        {
            yError() << "Could not view iCartesianSolver in KDLSolver";
            return false;
        }

        if (!armICartesianSolver->fwdKin(currentQ, xStart))
        {
            yError() << "fwdKin failed";

            reply.addString("fwdKin failed for the start state");
            return reply.write(*writer);
        }
        yInfo() << "Start:" << xStart[0] << " " << xStart[1] << " " << xStart[2] << " " << xStart[3] << " " << xStart[4] << " " << xStart[5];

        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

        KDL::Frame frame = vectorToFrame(xStart);
        double qx, qy, qz, qw;
        frame.M.GetQuaternion(qx, qy, qz, qw);
        ob::ScopedState<ob::SE3StateSpace> start(space);

        start->setXYZ(xStart[0], xStart[1], xStart[2]);
        start->rotation().x = qx;
        start->rotation().y = qy;
        start->rotation().z = qz;
        start->rotation().w = qw;
        pdef->clearStartStates();
        pdef->addStartState(start);

        // if (command.get(0).toString() == "Check goal"){
        //     yInfo()<<"Check goal";
        //     Bottle * bGoal = command.get(1).asList();
        //     std::vector<double> xGoal(6);
        //     for (int i = 0; i < 6; i++)
        //         xGoal[i] = bGoal->get(i).asDouble();
        //     yInfo() <<"Goal: "<< xGoal[0] << " " << xGoal[1] << " " << xGoal[2] << " " << xGoal[3] << " " << xGoal[4] << " " << xGoal[5];

        //     frame = vectorToFrame(xGoal);
        //     frame.M.GetQuaternion(qx, qy, qz, qw);
        //     ob::ScopedState<ob::SE3StateSpace> goal(space);

        //     goal->setXYZ(xGoal[0], xGoal[1], xGoal[2]);
        //     goal->rotation().x = qx;
        //     goal->rotation().y = qy;
        //     goal->rotation().z = qz;
        //     goal->rotation().w = qw;

        //     pdef->clearGoal();

        //     pdef->setGoalState(goal);

        //     ob::State *goalState = pdef->getGoal()->as<ob::GoalState>()->getState();
        //     if(isValid(goalState)){
        //         yInfo() << "Valid";
        //         reply.addString("Valid");
        //         Bottle bJointsPosition;

        //         for(int j = 0; j<numArmJoints+numTrunkJoints; j++){
        //             yInfo()<<goalQ[j];
        //             bJointsPosition.addDouble(goalQ[j]);
        //         }
        //         reply.addList() = bJointsPosition;
        //     }
        //     else{
        //         yInfo() << "Not Valid";
        //         reply.addString("Not Valid");
        //     }
        // }

        // else if (command.get(0).toString() == "Compute trajectory"){
        //     Bottle * bGoal = command.get(1).asList();
        //     std::vector<double> xGoal(6);
        //     for (int i = 0; i < 6; i++)
        //         xGoal[i] = bGoal->get(i).asDouble();
        //     frame = vectorToFrame(xGoal);
        //     frame.M.GetQuaternion(qx, qy, qz, qw);
        //     ob::ScopedState<ob::SE3StateSpace> goal(space);

        //     goal->setXYZ(xGoal[0], xGoal[1], xGoal[2]);
        //     goal->rotation().x = qx;
        //     goal->rotation().y = qy;
        //     goal->rotation().z = qz;
        //     goal->rotation().w = qw;

        //     pdef->clearGoal();

        //     pdef->setGoalState(goal);
        //     std::vector<std::vector<double>>jointsTrajectory;
        //     bool validStartState, validGoalState;
        //     bool solutionFound = computeDiscretePath(start, goal, jointsTrajectory, validStartState, validGoalState);

        //     if (solutionFound)
        //     {
        //         // followDiscretePath();
        //         yInfo() << "Solution Found";
        //         reply.addString("Solution Found");
        //         Bottle bJointsTrajectory;
        //         for(int i=0; i<jointsTrajectory.size(); i++){
        //             Bottle bJointsPosition;
        //             for(int j = 0; j<numArmJoints+numTrunkJoints; j++){
        //                 bJointsPosition.addDouble(jointsTrajectory[i][j]);
        //             }
        //             bJointsTrajectory.addList() = bJointsPosition;
        //         }
        //         reply.addList() =bJointsTrajectory;
        //     }
        //     else{
        //         if(!validStartState)
        //         {
        //             yInfo() <<"Start state NOT valid";
        //             reply.addString("Start state NOT valid");
        //         }
        //         if(!validGoalState){
        //             yInfo() <<"Goal state NOT valid";
        //             reply.addString("Goal state NOT valid"); 
        //         }
        //         if(validStartState && validGoalState){
        //             yInfo() << "Solution NOT found";
        //             reply.addString("Solution NOT found");
        //         }
        //     }
        // }
    }
    else{ // joint space
        yInfo("Joint space");
        unsigned int nj = numTrunkJoints+numArmJoints;
        space = ob::StateSpacePtr(new ob::RealVectorStateSpace(nj));
        ob::RealVectorBounds bounds{nj};
        for(unsigned int j=0; j<nj; j++){
            bounds.setLow(j, qmin(j));
            bounds.setHigh(j, qmax(j));
        }
        space->as<ob::RealVectorStateSpace>()->setBounds(bounds);

        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

        si->setStateValidityChecker(std::bind(&TrajectoryGeneration::isValid, this, std::placeholders::_1));
        si->setup();


        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));

        ob::ScopedState<ob::RealVectorStateSpace> start(space);

        if(!getCurrentQ(currentQ)){
            return false;
        }
        for(unsigned int j=0; j<numArmJoints+numTrunkJoints; j++){
            start[j] = currentQ[j];
        }

        
        switch(command.get(0).asVocab32()){
            case VOCAB_HELP:{
                yInfo() <<"help";
                static auto usage = makeUsage();
                reply.append(usage);
            }break;
            case VOCAB_CHECK_GOAL_POSE:
                {yInfo() << "Check goal pose (x, y, z, rotx, roty, rotz)";
                std::vector<double> desireQ(numArmJoints+numTrunkJoints);
                std::string errorMessage;
                if(!checkGoalPose(command.get(1).asList(), desireQ, errorMessage)){
                    reply.addVocab32(VOCAB_FAILED);
                    reply.addString(errorMessage);
                }
                else{
                    reply.addVocab32(VOCAB_OK);
                    Bottle bJointsPosition;
                    for(int j = 0; j<numArmJoints+numTrunkJoints; j++){
                        bJointsPosition.addFloat64(desireQ[j]);
                    }
                    reply.addList() = bJointsPosition;
                }
                }break;
            case VOCAB_CHECK_GOAL_JOINTS:
                {
                yInfo() << "Check goal joints (j0, j1, ..., jn)";
                std::string errorMessage;
                if(!checkGoalJoints(command.get(1).asList(), errorMessage)){
                    reply.addVocab32(VOCAB_FAILED);
                    reply.addString(errorMessage);
                }
                else{
                    reply.addVocab32(VOCAB_OK);
                }
                }break;
            case VOCAB_COMPUTE_JOINTS_PATH_GOAL_POSE:
                {
                    yInfo()<<"Compute path in joint space to goal pose (x, y, z, rotx, roty, rotz";
                    std::vector<double> desireQ(numArmJoints+numTrunkJoints);
                    std::string errorMessage;
                    if(!checkGoalPose(command.get(1).asList(), desireQ, errorMessage)){
                        reply.addVocab32(VOCAB_FAILED);
                        reply.addString(errorMessage);
                    }
                    else{
                        ob::ScopedState<ob::RealVectorStateSpace> goal(space);
                        for(unsigned int j=0; j<numArmJoints+numTrunkJoints; j++){
                            goal[j] = desireQ[j];
                        }
                        pdef->clearGoal();
                        pdef->setGoalState(goal);

                        std::vector<std::vector<double>>jointsTrajectory;
                        std::string errorMessage;
                        yInfo()<<"Lets compute the path";
                        bool solutionFound = computeDiscretePath(start, goal, jointsTrajectory, errorMessage);

                        if (solutionFound){
                            reply.addVocab32(VOCAB_OK);
                            Bottle bJointsTrajectory;
                            for(int i=0; i<jointsTrajectory.size(); i++){
                                Bottle bJointsPosition;
                                for(int j = 0; j<numArmJoints+numTrunkJoints; j++){
                                    bJointsPosition.addFloat64(jointsTrajectory[i][j]);
                                }
                                bJointsTrajectory.addList() = bJointsPosition;
                            }
                            reply.addList() =bJointsTrajectory;
                        }
                        else{
                            reply.addVocab32(VOCAB_FAILED);
                            reply.addString(errorMessage);
                        }
                    }
                }break;
            case VOCAB_COMPUTE_JOINTS_PATH_GOAL_JOINTS:
                {yInfo()<<"Compute path in joint space to goal joints configuration (j0, j1, ..., jn)";
                Bottle * bGoal = command.get(1).asList();
                std::string errorMessage;
                if (bGoal->size() != numArmJoints+ numTrunkJoints){
                    reply.addVocab32(VOCAB_FAILED);
                    reply.addString(errorsTrajectoryGeneration::joints_elements);
                }
                else{
                    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
                    for(unsigned int j=0; j<numArmJoints+numTrunkJoints; j++){
                        goal[j] = bGoal->get(j).asFloat64();
                    }
                    pdef->clearGoal();
                    pdef->setGoalState(goal);

                    ob::State *goalState = pdef->getGoal()->as<ob::GoalState>()->getState();

                    if (!si->satisfiesBounds(goalState)){
                        errorMessage = errorsTrajectoryGeneration::joints_outside_bounds;
                        return false;
                    }
                    else{
                        std::vector<std::vector<double>>jointsTrajectory;
                        bool solutionFound = computeDiscretePath(start, goal, jointsTrajectory, errorMessage);
                        if (solutionFound){
                            reply.addVocab32(VOCAB_OK);
                            Bottle bJointsTrajectory;
                            for(int i=0; i<jointsTrajectory.size(); i++){
                                Bottle bJointsPosition;
                                for(int j = 0; j<numArmJoints+numTrunkJoints; j++){
                                    bJointsPosition.addFloat64(jointsTrajectory[i][j]);
                                }
                                bJointsTrajectory.addList() = bJointsPosition;
                            }
                            reply.addList() =bJointsTrajectory;
                        }
                        else{
                            reply.addVocab32(VOCAB_FAILED);
                            reply.addString(errorMessage);
                        }
                    }
                }
                }break;

            }      
    }
    return reply.write(*writer);
}