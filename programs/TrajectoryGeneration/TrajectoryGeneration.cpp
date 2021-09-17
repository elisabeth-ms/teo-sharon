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
#define DEFAULT_PREFIX "/trajectoryGeneration"
#define DEFAULT_DEVICE_NAME "trunkAndRightArm"
#define DEFAULT_KINEMATICS_CONFIG "teo-trunk-rightArm-fetch.ini"
#define DEFAULT_RANGE_RRT 0.1
#define DEFAULT_PRUNE_THRESHOLD 0.6



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

/************************************************************************/

bool TrajectoryGeneration::configure(yarp::os::ResourceFinder &rf)
{
    resourceFinder = rf;
    robot = rf.check("robot", yarp::os::Value(DEFAULT_ROBOT), "name of /robot to be used").asString();
    std::string prefix = rf.check("prefix", yarp::os::Value(DEFAULT_PREFIX), "port prefix").asString();
    planningSpace = rf.check("planningSpace", yarp::os::Value(DEFAULT_PLANNING_SPACE), "planning space").asString();
    deviceName = rf.check("deviceName", yarp::os::Value(DEFAULT_DEVICE_NAME), "device name").asString();
    kinematicsConfig = rf.check("kinematicsConfig", yarp::os::Value(DEFAULT_KINEMATICS_CONFIG), "kinematics config").asString();
    printf("TrajectoryGeneration using robot: %s\n",robot.c_str());
    printf("TrajectoryGeneration using prefix: %s\n",prefix.c_str());
    printf("TrajectoryGeneration using planningSpace: %s\n",planningSpace.c_str());
    printf("TrajectoryGeneration using deviceName: %s\n",deviceName.c_str());
    printf("TrajectoryGeneration using kinematicsConfig: %s\n",kinematicsConfig.c_str());


    printf("--------------------------------------------------------------\n");
    if (rf.check("help"))
    {
        printf("TrajectoryGeneration options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--robot: %s [%s]\n", robot.c_str(), DEFAULT_ROBOT);
        ::exit(0);
    }

    if (!outCartesianTrajectoryPort.open("/"+robot +"/"+deviceName+"/cartesianTrajectory:o"))
    {
        yError("Bad outCartesianTrajectoryPort.open\n");
        return false;
    }

    // ------  ARM DEVICE -------

    yarp::os::Property armOptions;
    armOptions.put("device", "remote_controlboard");
    armOptions.put("remote", "/"+robot+"/"+deviceName);
    armOptions.put("local", "/" + robot + "/"+deviceName);
    armDevice.open(armOptions);
    if (!armDevice.isValid())
    {
        yError() << "Robot "<<deviceName<<" device not available";
        armDevice.close();
        yarp::os::Network::fini();
        return false;
    }

    // connecting our device with "IEncoders" interface
    if (!armDevice.view(armIEncoders))
    {
        yError() << "Problems acquiring IEncoders interface in "<<deviceName;
        return false;
    }
    else
    {
        yInfo() << "Acquired IEncoders interface in "<<deviceName;
        if (!armIEncoders->getAxes(&numArmJoints))
            yError() << "Problems acquiring numArmJoints";
        else
            yWarning() << "Number of joints:" << numArmJoints;
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

    // ----- Configuring KDL Solver for device -----


    if (!armDevice.view(armIControlLimits))
    {
        yError() << "Could not view iControlLimits in "<<deviceName;
        return false;
    }

    //  Getting the limits of each joint
    printf("---- Joint limits of %s ----\n",deviceName.c_str());

    qmin.resize(numArmJoints);
    qmax.resize(numArmJoints);


    for (unsigned int joint = 0; joint < numArmJoints; joint++)
    {
        double min, max;
        armIControlLimits->getLimits(joint, &min, &max);
        if(numArmJoints == 8 && joint == 1){ // we don't want the frontal joint tilt so much
            min = - 10;
            max = 20.0;
        }
        qrMin.addDouble(min);
        qrMax.addDouble(max);
        yInfo("Joint %d limits: [%f,%f]", joint, min, max);
        qmin(joint) = min;
        qmax(joint) = max;
    }


    std::string kinPath = rf.findFileByName(kinematicsConfig);
    armSolverOptions.fromConfigFile(kinPath);
    armSolverOptions.put("device", "KdlSolver");
    armSolverOptions.put("mins", yarp::os::Value::makeList(qrMin.toString().c_str()));
    armSolverOptions.put("maxs", yarp::os::Value::makeList(qrMax.toString().c_str()));
    armSolverOptions.put("ik", "nrjl"); // to use screw theory IK
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
    CollisionGeometryPtr_t teoRootTrunk{new fcl::Boxf{0.25, 0.25, 0.6}};
    fcl::Transform3f tfTest;
    fcl::CollisionObjectf collisionObject1{teoRootTrunk, tfTest};

    CollisionGeometryPtr_t teoTrunk{new fcl::Boxf{0.3, 0.3, 0.46}};
    fcl::CollisionObjectf collisionObject2{teoTrunk, tfTest};
   
    CollisionGeometryPtr_t teoAxialShoulder{new fcl::Boxf{AXIAL_SHOULDER_LINK_RADIUS,AXIAL_SHOULDER_LINK_RADIUS,AXIAL_SHOULDER_LINK_LENGTH}};//{new fcl::Box{0.15, 0.15, 0.32901}};
    fcl::CollisionObjectf collisionObject3{teoAxialShoulder, tfTest};

    CollisionGeometryPtr_t teoElbow{new fcl::Boxf{FRONTAL_ELBOW_LINK_RADIUS, FRONTAL_ELBOW_LINK_RADIUS, FRONTAL_ELBOW_LINK_LENGTH}};
    fcl::CollisionObjectf collisionObject4{teoElbow, tfTest};
    
    CollisionGeometryPtr_t teoWrist{new fcl::Boxf{FRONTAL_WRIST_LINK_RADIUS, 0.24, 0.15}};
    fcl::CollisionObjectf collisionObject5{teoWrist, tfTest};

    CollisionGeometryPtr_t endEffector{new fcl::Boxf{0.1,0.1,0.1}};
    fcl::CollisionObjectf collisionObject6{endEffector, tfTest};


    CollisionGeometryPtr_t table{new fcl::Boxf{0.8,1.5,0.9}};
    fcl::CollisionObjectf collisionObjectTable{table, tfTest};
    fcl::Vector3f translation(0.6, -0.0, -0.45);
    collisionObjectTable.setTranslation(translation);
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
            bounds.setLow(j, qmin(j));
            bounds.setHigh(j, qmax(j));
        }
        space->as<ob::RealVectorStateSpace>()->setBounds(bounds);
        si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));

        si->setStateValidityChecker(std::bind(&TrajectoryGeneration::isValid, this, std::placeholders::_1));
        si->setup();

        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
    }


    rpcServer.setReader(*this);

    return true;
}

/************************************************************************/

bool TrajectoryGeneration::interruptModule()
{
    return true;
}

/************************************************************************/

double TrajectoryGeneration::getPeriod()
{
    return 15.0; // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/

bool TrajectoryGeneration::updateModule()
{

    return true;
}

/************************************************************************/

bool TrajectoryGeneration::threadInit()
{
    initTime = Time::now();
    return true;
}

/**************** Sending Thread *************************/

void TrajectoryGeneration::run()
{
    yInfo() << "running";
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

        // //yInfo() << pos->values[0] << " " << pos->values[1] << " " << pos->values[2];

        KDL::Frame frame;

        KDL::Rotation rotKdl = KDL::Rotation::Quaternion(rot->x, rot->y, rot->z, rot->w);
        KDL::Vector posKdl = KDL::Vector(pos->values[0], pos->values[1], pos->values[2]);
        //frame.M.Quaternion(rot->x, rot->y, rot->z, rot->w);
        double x, y, z, w;
        rotKdl.GetQuaternion(x, y, z, w);
        // //yInfo()<<"Pos: "<<pos->values[0]<<" "<< pos->values[1]<<" "<< pos->values[2];
        // //yInfo() << "Quaterion" << x << " " << y << " "
        // //        << " " << z << " " << w;
        frame.M = rotKdl;
        frame.p = posKdl;
        std::vector<double> testAxisAngle = frameToVector(frame);
        // //yInfo() << testAxisAngle;
        // //yInfo() << testAxisAngle;

        std::vector<double> currentQ(numArmJoints);
        if (!armIEncoders->getEncoders(currentQ.data()))
        {
            yError() << "Failed getEncoders() of "<<deviceName;
            return false;
        }

        // Check if we are in the starting state

        std::vector<double> xStart;

        if (!armICartesianSolver->fwdKin(currentQ, xStart))
        {
            yError() << "fwdKin failed";
            return false;
        }
        yInfo() << "Start:" << xStart[0] << " " << xStart[1] << " " << xStart[2] << " " << xStart[3] << " " << xStart[4] << " " << xStart[5];

        bool computeInvKin = true;

        double diffSumSquare = 0;
        for (int i = 0; i < 6; i++)
        {
            diffSumSquare += (xStart[i] - testAxisAngle[i]) * (xStart[i] - testAxisAngle[i]);
        }

        //yInfo() << "diffSumSquare: " << diffSumSquare;

        if (diffSumSquare < DEFAULT_MAX_DIFF_INV)
        {
            //yInfo() << "Start state-> we do not calculate the inverse kinematics";
            computeInvKin = false;
        }

        std::vector<double> desireQ(numArmJoints);


        if (computeInvKin)
        {
            if (!armICartesianSolver->invKin(testAxisAngle, currentQ, desireQ))
            {
                yError() << "invKin() failed";
                return false;
            }
        
            yInfo()<<"desireQ: "<<desireQ;
        
            KDL::JntArray jointpositions = KDL::JntArray(8);

            for (unsigned int i = 0; i < jointpositions.rows(); i++)
            {
                jointpositions(i) = desireQ[i];
            }
            checkSelfCollision->updateCollisionObjectsTransform(jointpositions);
            bool selfCollide = checkSelfCollision->selfCollision();
            if(selfCollide == true)
                yInfo()<<"Collide";
            yInfo()<<selfCollide;
            return !selfCollide;
        }
        return true;
    }
    else{ // joint space
        yInfo()<<"joint space isValid()";
        const ob::RealVectorStateSpace::StateType *jointState = state->as<ob::RealVectorStateSpace::StateType>();
        KDL::JntArray jointpositions = KDL::JntArray(numArmJoints);

        for (unsigned int i = 0; i < jointpositions.rows(); i++)
        {
            jointpositions(i) = jointState->values[i];
            yInfo()<<jointpositions(i);

        }
        checkSelfCollision->updateCollisionObjectsTransform(jointpositions);
        bool selfCollide = checkSelfCollision->selfCollision();
        if(selfCollide == true)
            yInfo()<<"Collide";
        return !selfCollide;

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

    ob::PlannerPtr planner = ob::PlannerPtr(plannerRRT);

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

    planner->setProblemDefinition(pdef);

    planner->setup();

    bool solutionFound = planner->solve(2.0);

    if (solutionFound == true)
    {
        yInfo() << "Sol";
        ob::PathPtr path = pdef->getSolutionPath();
        yInfo() << "Found discrete path from start to goal";
        path->print(std::cout);
        pth = path->as<og::PathGeometric>();
        yInfo() << pth->getStateCount();
    }

    // Send computed trajectory through the port for plotting

//#ifdef SEND_TRAJECTORY_DATA
    Bottle &out= outCartesianTrajectoryPort.prepare();
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
        
        std::vector<double> currentQ(numArmJoints);
        if (!armIEncoders->getEncoders(currentQ.data()))
        {
            yError() << "Failed getEncoders() of "<<deviceName;
            return false;
        }
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
            diffSumSquare += (xStart[i] - pose[i]) * (xStart[i] - pose[i]);
        }

        yInfo() << "diffSumSquare: " << diffSumSquare;

        if (diffSumSquare < DEFAULT_MAX_DIFF_INV)
        {
            yInfo() << "Start state-> we do not calculate the inverse kinematics";
            computeInvKin = false;
        }

        std::vector<double> desireQ(numArmJoints);

        if (computeInvKin)
        {
            if (!armICartesianSolver->invKin(pose, currentQ, desireQ))
            {
                yError() << "invKin() failed";
            }
            else{
                Bottle bPose;
                for(int i = 0; i < 6; i++){
                    bPose.addDouble(pose[i]);
                }
                std::vector<double> jointsPosition;
                jointsPosition.reserve(numArmJoints);
                for(int i = 0; i<numArmJoints; i++){
                    jointsPosition.emplace_back(desireQ[i]);
                }
                jointsTrajectory.push_back(jointsPosition);
                out.addList() = bPose;
            }
        }
        
        iState++;
    }
    outCartesianTrajectoryPort.write(true);

//#endif
    // followDiscretePath();

    return solutionFound;
}


bool TrajectoryGeneration::computeDiscretePath(ob::ScopedState<ob::RealVectorStateSpace> start, ob::ScopedState<ob::RealVectorStateSpace> goal, std::vector<std::vector<double>> &jointsTrajectory, bool &validStartState, bool &validGoalState)
{

    // pdef->clearStartStates();
    // pdef->addStartState(start);


    auto plannerRRT = (new og::RRTstar(si));
    plannerRRT->setRange(1.0);
    plannerRRT->setPruneThreshold(1.5);
    plannerRRT->setTreePruning(true);
    plannerRRT->setInformedSampling(true);

    ob::PlannerPtr planner = ob::PlannerPtr(plannerRRT);

    ob::State *startState = pdef->getStartState(0);

  
   
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

    ob::RealVectorBounds bounds = space->as<ob::RealVectorStateSpace>()->getBounds();
    for(unsigned int j=0; j<numArmJoints; j++){
        yInfo()<<"Low: "<<bounds.low[j]<<" high: "<<bounds.high[j];
    }

    planner->setProblemDefinition(pdef);

    planner->setup();
    pdef->clearSolutionPaths();


    bool solutionFound = planner->solve(15.0);


    if (solutionFound == true)
    {
        yInfo() << "Sol";
        ob::PathPtr path = pdef->getSolutionPath();
        yInfo() << "Found discrete path from start to goal";
        path->print(std::cout);
        pth = path->as<og::PathGeometric>();
        yInfo() << pth->getStateCount();
    }

    // Send computed trajectory through the port for plotting

//#ifdef SEND_TRAJECTORY_DATA
    Bottle &out= outCartesianTrajectoryPort.prepare();
    std::size_t iState = 0;

    while (iState < pth->getStateCount())
    {
        ob::State *state = pth->getState(iState);

        const ob::RealVectorStateSpace::StateType *jointState = state->as<ob::RealVectorStateSpace::StateType>();


        std::vector<double> poseQ;
        poseQ.reserve(numArmJoints);
        for(unsigned int j=0; j<numArmJoints; j++)
            poseQ.emplace_back(jointState->values[j]);

        std::vector<double> pose;

        if (!armICartesianSolver->fwdKin(poseQ, pose))
        {
            yError() << "fwdKin failed";
            return false;
        }

        else{
            Bottle bPose;
            for(int i = 0; i < 6; i++){
                bPose.addDouble(pose[i]);
            }
            jointsTrajectory.push_back(poseQ);
            yInfo()<<poseQ;
            out.addList() = bPose;
        }
        
        iState++;
    }
    outCartesianTrajectoryPort.write(true);

//#endif
    // followDiscretePath();

    return solutionFound;
}

bool TrajectoryGeneration::read(yarp::os::ConnectionReader &connection)
{
    yarp::os::Bottle reply;
    auto *writer = connection.getWriter();


    if (!yarp::os::NetworkBase::exists("/"+robot+"/"+deviceName+"/rpc:i"))
    {
        yError()<<"Check if the device "<<deviceName<<" is open!";
        // configure(resourceFinder);
        reply.addString("Device not open");
        return reply.write(*writer);
    }
    else{
        if(!yarp::os::NetworkBase::isConnected("/"+robot+"/"+deviceName+"/rpc:o", "/"+robot+"/"+deviceName+"/rpc:i"))
        {
            yWarning()<<"Device not connected";
            configure(resourceFinder);
            reply.addString("Open device");
            return reply.write(*writer);
        }
    }

    yarp::os::Bottle command;
    if (!command.read(connection) || writer == nullptr)
    {
        return false;
    }
    yInfo() << "command:" << command.toString();



    std::vector<double> currentQ(numArmJoints);
    if (!armIEncoders->getEncoders(currentQ.data()))
    {
        yError() << "Failed getEncoders() of "<<deviceName;
    }

    yInfo() << "currentQ: " << currentQ;
     
    if (planningSpace=="cartesian"){

        armSolverOptions.unput("mins");
        armSolverOptions.unput("maxs");
        yarp::os::Bottle qMin, qMax;
        qMin.addDouble(currentQ[0]-2.5);
        qMin.addDouble(currentQ[1]-2.5);
        qMax.addDouble(currentQ[0]+0.5);
        qMax.addDouble(currentQ[1]+0.5);
        for(int i=2; i<currentQ.size(); i++){
            qMin.addDouble(qrMin.get(i).asDouble());
            qMax.addDouble(qrMax.get(i).asDouble());
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
        std::vector<double> xStart;

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

        if (command.get(0).toString() == "Check goal"){
            yInfo()<<"Check goal";
            Bottle * bGoal = command.get(1).asList();
            std::vector<double> xGoal(6);
            for (int i = 0; i < 6; i++)
                xGoal[i] = bGoal->get(i).asDouble();
            yInfo() <<"Goal: "<< xGoal[0] << " " << xGoal[1] << " " << xGoal[2] << " " << xGoal[3] << " " << xGoal[4] << " " << xGoal[5];

            frame = vectorToFrame(xGoal);
            frame.M.GetQuaternion(qx, qy, qz, qw);
            ob::ScopedState<ob::SE3StateSpace> goal(space);

            goal->setXYZ(xGoal[0], xGoal[1], xGoal[2]);
            goal->rotation().x = qx;
            goal->rotation().y = qy;
            goal->rotation().z = qz;
            goal->rotation().w = qw;

            pdef->clearGoal();

            pdef->setGoalState(goal);

            ob::State *goalState = pdef->getGoal()->as<ob::GoalState>()->getState();
            if(isValid(goalState)){
                yInfo() << "Valid";
                reply.addString("Valid");
            }
            else{
                yInfo() << "Not Valid";
                reply.addString("Not Valid");
            }
        }

        else if (command.get(0).toString() == "Compute trajectory"){
            Bottle * bGoal = command.get(1).asList();
            std::vector<double> xGoal(6);
            for (int i = 0; i < 6; i++)
                xGoal[i] = bGoal->get(i).asDouble();
            frame = vectorToFrame(xGoal);
            frame.M.GetQuaternion(qx, qy, qz, qw);
            ob::ScopedState<ob::SE3StateSpace> goal(space);

            goal->setXYZ(xGoal[0], xGoal[1], xGoal[2]);
            goal->rotation().x = qx;
            goal->rotation().y = qy;
            goal->rotation().z = qz;
            goal->rotation().w = qw;

            pdef->clearGoal();

            pdef->setGoalState(goal);
            std::vector<std::vector<double>>jointsTrajectory;
            bool validStartState, validGoalState;
            bool solutionFound = computeDiscretePath(start, goal, jointsTrajectory, validStartState, validGoalState);

            if (solutionFound)
            {
                // followDiscretePath();
                yInfo() << "Solution Found";
                reply.addString("Solution Found");
                Bottle bJointsTrajectory;
                for(int i=0; i<jointsTrajectory.size(); i++){
                    Bottle bJointsPosition;
                    for(int j = 0; j<numArmJoints; j++){
                        bJointsPosition.addDouble(jointsTrajectory[i][j]);
                    }
                    bJointsTrajectory.addList() = bJointsPosition;
                }
                reply.addList() =bJointsTrajectory;
            }
            else{
                if(!validStartState)
                {
                    yInfo() <<"Start state NOT valid";
                    reply.addString("Start state NOT valid");
                }
                if(!validGoalState){
                    yInfo() <<"Goal state NOT valid";
                    reply.addString("Goal state NOT valid"); 
                }
                if(validStartState && validGoalState){
                    yInfo() << "Solution NOT found";
                    reply.addString("Solution NOT found");
                }
            }
        }
    }
    else{ // joint space
        yInfo("Joint space");
        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
        ob::ScopedState<ob::RealVectorStateSpace> start(space);
        for(unsigned int j=0; j<numArmJoints; j++){
            start[j] = currentQ[j];
        }
        pdef->clearStartStates();
        pdef->addStartState(start);
        
        ob::State *startState = pdef->getStartState(0);
        if(isValid(startState))
            yInfo()<<"Valid start state";
        else{
            yInfo()<<"Not valid start state";
        }
                

        if (command.get(0).toString() == "Check goal"){
            yInfo()<<"Check goal";
            Bottle * bGoal = command.get(1).asList();
            std::vector<double> xGoal(6);
            for (int i = 0; i < 6; i++)
                xGoal[i] = bGoal->get(i).asDouble();
            yInfo() <<"Goal: "<< xGoal[0] << " " << xGoal[1] << " " << xGoal[2] << " " << xGoal[3] << " " << xGoal[4] << " " << xGoal[5];

            std::vector<double> desireQ(numArmJoints);

            std::vector<double> currentQ(numArmJoints);
            if (!armIEncoders->getEncoders(currentQ.data()))
            {
                yError() << "Failed getEncoders() of "<<deviceName;
                reply.addString("Not Valid");
            }
            else{
                if (!armICartesianSolver->invKin(xGoal, currentQ, desireQ))
                {
                    yError() << "invKin() failed";
                    reply.addString("Not Valid");
                }
                else{   
                    yInfo()<<"desireQ: "<<desireQ;
                    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
                    for(unsigned int j=0; j<numArmJoints; j++){
                        goal[j] = desireQ[j];
                    }
                    pdef->clearGoal();

                    pdef->setGoalState(goal);

                    ob::State *goalState = pdef->getGoal()->as<ob::GoalState>()->getState();
                    if(isValid(goalState)){
                        yInfo() << "Valid";
                        reply.addString("Valid");
                    }
                    else{
                        yInfo() << "Not Valid";
                        reply.addString("Not Valid");
                    }    
                }
            }
        }
        else if (command.get(0).toString() == "Compute trajectory"){
            yInfo()<<"Compute trajectory";
            Bottle * bGoal = command.get(1).asList();
            std::vector<double> xGoal(6);
            for (int i = 0; i < 6; i++)
                xGoal[i] = bGoal->get(i).asDouble();

            std::vector<double> desireQ(numArmJoints);
            std::vector<double> currentQ(numArmJoints);
            yInfo()<<"getting encoders";
            if (!armIEncoders->getEncoders(currentQ.data()))
            {
                yError() << "Failed getEncoders() of "<<deviceName;
                reply.addString("Not Valid");
            }
            
            else{
                yInfo()<<"getEncoders() of "<<deviceName<<" ok";
                if (!armICartesianSolver->invKin(xGoal, currentQ, desireQ))
                {
                    yError() << "invKin() failed";
                    reply.addString("Not Valid");
                }
                else{   
                    yInfo()<<"desireQ: "<<desireQ;
                    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
                    for(unsigned int j=0; j<numArmJoints; j++){
                        goal[j] = desireQ[j];
                    }
                    pdef->clearGoal();
                    pdef->setGoalState(goal);

                    std::vector<std::vector<double>>jointsTrajectory;
                    bool validStartState, validGoalState;
                    bool solutionFound = computeDiscretePath(start, goal, jointsTrajectory, validStartState, validGoalState);

                    if (solutionFound)
                    {
                        // followDiscretePath();
                        yInfo() << "Solution Found";
                        reply.addString("Solution Found");
                        Bottle bJointsTrajectory;
                        for(int i=0; i<jointsTrajectory.size(); i++){
                            Bottle bJointsPosition;
                            for(int j = 0; j<numArmJoints; j++){
                                bJointsPosition.addDouble(jointsTrajectory[i][j]);
                            }
                            bJointsTrajectory.addList() = bJointsPosition;
                        }
                        reply.addList() =bJointsTrajectory;
                    }
                    else{
                        if(!validStartState)
                        {
                            yInfo() <<"Start state NOT valid";
                            reply.addString("Start state NOT valid");
                        }
                        if(!validGoalState){
                            yInfo() <<"Goal state NOT valid";
                            reply.addString("Goal state NOT valid"); 
                        }
                        if(validStartState && validGoalState){
                            yInfo() << "Solution NOT found";
                            reply.addString("Solution NOT found");
                        }
                    }

                }

            }
        }
        else{
            yWarning()<<"command not available";
        }
    }
    return reply.write(*writer);
}