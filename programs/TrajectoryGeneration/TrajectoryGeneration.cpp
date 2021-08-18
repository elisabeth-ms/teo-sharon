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
#define DEFAULT_RANGE_RRT 0.1
#define DEFAULT_PRUNE_THRESHOLD 0.6

static KDL::Chain makeTeoFixedTrunkAndRightArmKinematicsFromDH()
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

bool TrajectoryGeneration::configure(yarp::os::ResourceFinder &rf)
{
    robot = rf.check("robot", yarp::os::Value(DEFAULT_ROBOT), "name of /robot to be used").asString();
    std::string prefix = rf.check("prefix", yarp::os::Value(DEFAULT_PREFIX), "port prefix").asString();
    planningSpace = rf.check("planningSpace", yarp::os::Value(DEFAULT_PLANNING_SPACE), "planning space").asString();
    printf("--------------------------------------------------------------\n");
    if (rf.check("help"))
    {
        printf("TrajectoryGeneration options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--robot: %s [%s]\n", robot.c_str(), DEFAULT_ROBOT);
        ::exit(0);
    }

    if (!outCartesianTrajectoryPort.open("/"+robot + "/cartesianTrajectory:o"))
    {
        yError("Bad outCartesianTrajectoryPort.open\n");
        return false;
    }

    // ------ RIGHT ARM -------

    yarp::os::Property rightArmOptions;
    rightArmOptions.put("device", "remote_controlboard");
    rightArmOptions.put("remote", "/teoSim/trunkAndRightArm");
    rightArmOptions.put("local", "/" + robot + "/trunkAndrightArm");
    rightArmDevice.open(rightArmOptions);
    if (!rightArmDevice.isValid())
    {
        yError() << "Robot trunkAndRightArm device not available";
        rightArmDevice.close();
        yarp::os::Network::fini();
        return false;
    }

    // connecting our device with "IEncoders" interface
    if (!rightArmDevice.view(rightArmIEncoders))
    {
        yError() << "Problems acquiring rightArmIEncoders interface";
        return false;
    }
    else
    {
        yInfo() << "Acquired rightArmIEncoders interface";
        if (!rightArmIEncoders->getAxes(&numRightArmJoints))
            yError() << "Problems acquiring numRightArmJoints";
        else
            yWarning() << "Number of joints:" << numRightArmJoints;
    }

    // connecting our device with "control mode" interface, initializing which control mode we want (position)
    if (!rightArmDevice.view(rightArmIControlMode))
    {
        yError() << "Problems acquiring rightArmIControlMode interface";
        return false;
    }
    else
        yInfo() << "Acquired rightArmIControlMode interface";

    // connecting our device with "PositionControl" interface
    if (!rightArmDevice.view(rightArmIPositionControl))
    {
        yError() << "Problems acquiring rightArmIPositionControl interface";
        return false;
    }
    else
        yInfo() << "Acquired rightArmIPositionControl interface";

    // connecting our device with "PositionDirect" interface
    // if (!rightArmDevice.view(rightArmIPositionDirect))
    // {
    //     yError() << "Problems acquiring rightArmIPositionDirect interface";
    //     return false;
    // }
    // else
    //     yInfo() << "Acquired rightArmIPositionDirect interface";

    // std::vector<int> modes(numRightArmJoints, VOCAB_CM_POSITION_DIRECT);

    // if (!rightArmIControlMode->setControlModes(modes.data()))
    // {
    //     yError() << "Unable to change mode";
    //     return 1;
    // }
    // ----- Configuring KDL Solver for trunk and right arm -----


    if (!rightArmDevice.view(rightArmIControlLimits))
    {
        yError() << "Could not view iControlLimits in rightArmDevice";
        return false;
    }

    //  Getting the limits of each joint
    printf("---- Joint limits of right-arm ----\n");

    qmin.resize(numRightArmJoints);
    qmax.resize(numRightArmJoints);

    for (unsigned int joint = 0; joint < numRightArmJoints; joint++)
    {
        double min, max;
        rightArmIControlLimits->getLimits(joint, &min, &max);
        qrMin.addDouble(min);
        qrMax.addDouble(max);
        yInfo("Joint %d limits: [%f,%f]", joint, min, max);
        qmin(joint) = min;
        qmax(joint) = max;
    }


    std::string rightKinPath = rf.findFileByName("teo-trunk-rightArm-fetch.ini");
    rightArmSolverOptions.fromConfigFile(rightKinPath);
    rightArmSolverOptions.put("device", "KdlSolver");
    rightArmSolverOptions.put("mins", yarp::os::Value::makeList(qrMin.toString().c_str()));
    rightArmSolverOptions.put("maxs", yarp::os::Value::makeList(qrMax.toString().c_str()));
    rightArmSolverOptions.put("ik", "nrjl"); // to use screw theory IK
    rightArmSolverDevice.open(rightArmSolverOptions);
    if (!rightArmSolverDevice.isValid())
    {
        yError() << "KDLSolver solver device for trunkAndRightArm is not valid";
        return false;
    }

    if (!rightArmSolverDevice.view(rightArmICartesianSolver))
    {
        yError() << "Could not view iCartesianSolver in KDLSolver";
        return false;
    }

    yInfo() << "Acquired rightArmICartesianSolver interface";

    yInfo() << "Running";
    std::vector<double> position;

    if (!rpcServer.open(prefix + "/rpc:s"))
    {
        yError() << "Unable to open RPC server port" << rpcServer.getName();
        return false;
    }
    

    //Init collisions objects
    trunkAndRightArmChain = makeTeoFixedTrunkAndRightArmKinematicsFromDH();

    unsigned int nj = trunkAndRightArmChain.getNrOfJoints();
    yInfo() << "Number of joints: " << nj;
    unsigned int nl = trunkAndRightArmChain.getNrOfSegments();
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


    int nOfCollisionObjects = 6;
    rightArmCollisionObjects.clear();
    rightArmCollisionObjects.reserve(nOfCollisionObjects);
    rightArmCollisionObjects.emplace_back(collisionObject1);
    rightArmCollisionObjects.emplace_back(collisionObject2);
    rightArmCollisionObjects.emplace_back(collisionObject3);
    rightArmCollisionObjects.emplace_back(collisionObject4);
    rightArmCollisionObjects.emplace_back(collisionObject5);
    rightArmCollisionObjects.emplace_back(collisionObject6);

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

    checkSelfCollision = new CheckSelfCollision(trunkAndRightArmChain, qmin, qmax, rightArmCollisionObjects, offsetCollisionObjects);
    yInfo()<<"Check selfCollisionObject created";
    // space = ob::StateSpacePtr(new ob::RealVectorStateSpace(numRightArmJoints));
    if(planningSpace == "cartesian"){ // cartesian space
        space = ob::StateSpacePtr(new ob::SE3StateSpace());
        // ob::RealVectorBounds bounds{numRightArmJoints};
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



    // std::vector<double> currentQ(numRightArmJoints);
    // if (!rightArmIEncoders->getEncoders(currentQ.data()))
    // {
    //     yError() << "Failed getEncoders() of right-arm";
    //     return false;
    // }
    

    // KDL::JntArray qGoal = KDL::JntArray(numRightArmJoints);
    // qGoal(0) = 0;
    // qGoal(1) = -8;

    // checkSelfCollision->updateCollisionObjectsTransform(qGoal);
    // if (checkSelfCollision->selfCollision()){
    //     yInfo()<<"Goal self collide";
    // }
    // yInfo()<<"Goal doesn't collide";

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

        std::vector<double> currentQ(numRightArmJoints);
        if (!rightArmIEncoders->getEncoders(currentQ.data()))
        {
            yError() << "Failed getEncoders() of right-arm";
            return false;
        }

        // Check if we are in the starting state

        std::vector<double> xStart;

        if (!rightArmICartesianSolver->fwdKin(currentQ, xStart))
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

        std::vector<double> desireQ(numRightArmJoints);


        if (computeInvKin)
        {
            if (!rightArmICartesianSolver->invKin(testAxisAngle, currentQ, desireQ))
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
        KDL::JntArray jointpositions = KDL::JntArray(numRightArmJoints);

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
        
        std::vector<double> currentQ(numRightArmJoints);
        if (!rightArmIEncoders->getEncoders(currentQ.data()))
        {
            yError() << "Failed getEncoders() of right-arm";
            return false;
        }
        std::vector<double> xStart;

        if (!rightArmICartesianSolver->fwdKin(currentQ, xStart))
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

        std::vector<double> desireQ(numRightArmJoints);

        if (computeInvKin)
        {
            if (!rightArmICartesianSolver->invKin(pose, currentQ, desireQ))
            {
                yError() << "invKin() failed";
            }
            else{
                Bottle bPose;
                for(int i = 0; i < 6; i++){
                    bPose.addDouble(pose[i]);
                }
                std::vector<double> jointsPosition;
                jointsPosition.reserve(numRightArmJoints);
                for(int i = 0; i<numRightArmJoints; i++){
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
    plannerRRT->setRange(0.8);
    plannerRRT->setPruneThreshold(0.2);
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
    for(unsigned int j=0; j<numRightArmJoints; j++){
        yInfo()<<"Low: "<<bounds.low[j]<<" high: "<<bounds.high[j];
    }

    planner->setProblemDefinition(pdef);

    planner->setup();

    bool solutionFound = planner->solve(8.0);

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
        poseQ.reserve(numRightArmJoints);
        for(unsigned int j=0; j<numRightArmJoints; j++)
            poseQ.emplace_back(jointState->values[j]);

        std::vector<double> pose;

        if (!rightArmICartesianSolver->fwdKin(poseQ, pose))
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
    auto *writer = connection.getWriter();
    yarp::os::Bottle command;
    if (!command.read(connection) || writer == nullptr)
    {
        return false;
    }
    yarp::os::Bottle reply;

    yInfo() << "command:" << command.toString();


    std::vector<double> currentQ(numRightArmJoints);
    if (!rightArmIEncoders->getEncoders(currentQ.data()))
    {
        yError() << "Failed getEncoders() of right-arm";
    }

    yInfo() << "currentQ: " << currentQ;
     
    if (planningSpace=="cartesian"){

        rightArmSolverOptions.unput("mins");
        rightArmSolverOptions.unput("maxs");
        yarp::os::Bottle qMin, qMax;
        qMin.addDouble(currentQ[0]-2.5);
        qMin.addDouble(currentQ[1]-2.5);
        qMax.addDouble(currentQ[0]+0.5);
        qMax.addDouble(currentQ[1]+0.5);
        for(int i=2; i<currentQ.size(); i++){
            qMin.addDouble(qrMin.get(i).asDouble());
            qMax.addDouble(qrMax.get(i).asDouble());
        }
        rightArmSolverOptions.put("mins", yarp::os::Value::makeList(qMin.toString().c_str()));
        rightArmSolverOptions.put("maxs", yarp::os::Value::makeList(qMax.toString().c_str()));
        rightArmSolverDevice.close();

        rightArmSolverDevice.open(rightArmSolverOptions);
        if (!rightArmSolverDevice.isValid())
        {
            yError() << "KDLSolver solver device for trunkAndRightArm is not valid";
            return false;
        }


        if (!rightArmSolverDevice.view(rightArmICartesianSolver))
        {
            yError() << "Could not view iCartesianSolver in KDLSolver";
            return false;
        }
        std::vector<double> xStart;

        if (!rightArmICartesianSolver->fwdKin(currentQ, xStart))
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
                    for(int j = 0; j<numRightArmJoints; j++){
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
        pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
        ob::ScopedState<ob::RealVectorStateSpace> start(space);
        for(unsigned int j=0; j<numRightArmJoints; j++){
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
                
        // const ob::RealVectorStateSpace::StateType *jointState = startState->as<ob::RealVectorStateSpace::StateType>();
        // for(unsigned int j=0; j<numRightArmJoints; j++)
            // yInfo()<<"Start j"<<j<<": "<<jointState->values[j];
        
        if (command.get(0).toString() == "Check goal"){
            yInfo()<<"Check goal";
            Bottle * bGoal = command.get(1).asList();
            std::vector<double> xGoal(6);
            for (int i = 0; i < 6; i++)
                xGoal[i] = bGoal->get(i).asDouble();
            yInfo() <<"Goal: "<< xGoal[0] << " " << xGoal[1] << " " << xGoal[2] << " " << xGoal[3] << " " << xGoal[4] << " " << xGoal[5];

            std::vector<double> desireQ(numRightArmJoints);

            std::vector<double> currentQ(numRightArmJoints);
            if (!rightArmIEncoders->getEncoders(currentQ.data()))
            {
                yError() << "Failed getEncoders() of right-arm";
                reply.addString("Not Valid");
            }
            else{
                if (!rightArmICartesianSolver->invKin(xGoal, currentQ, desireQ))
                {
                    yError() << "invKin() failed";
                    reply.addString("Not Valid");
                }
                else{   
                    yInfo()<<"desireQ: "<<desireQ;
                    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
                    for(unsigned int j=0; j<numRightArmJoints; j++){
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

            std::vector<double> desireQ(numRightArmJoints);
            std::vector<double> currentQ(numRightArmJoints);
            yInfo()<<"getting encoders";
            if (!rightArmIEncoders->getEncoders(currentQ.data()))
            {
                yError() << "Failed getEncoders() of right-arm";
                reply.addString("Not Valid");
            }
            
            else{
                yInfo()<<"getEncoders() of right-arm ok";
                if (!rightArmICartesianSolver->invKin(xGoal, currentQ, desireQ))
                {
                    yError() << "invKin() failed";
                    reply.addString("Not Valid");
                }
                else{   
                    yInfo()<<"desireQ: "<<desireQ;
                    ob::ScopedState<ob::RealVectorStateSpace> goal(space);
                    for(unsigned int j=0; j<numRightArmJoints; j++){
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
                            for(int j = 0; j<numRightArmJoints; j++){
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
    }
    return reply.write(*writer);
}