// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TrajectoryGeneration.hpp"

#include <cstdio>
#include <algorithm>
#include <ColorDebug.h>
#include <KdlVectorConverter.hpp>
#include <KinematicRepresentation.hpp>
#include <kdl/path_line.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_rect.hpp>
#include <kdl/velocityprofile_trap.hpp>

#include <yarp/os/LogStream.h>
using namespace roboticslab::KdlVectorConverter;
using namespace roboticslab::KinRepresentation;
using namespace sharon;

/************************************************************************/

bool TrajectoryGeneration::configure(yarp::os::ResourceFinder & rf)
{
    watchdog = DEFAULT_WATCHDOG;  // double
    printf("TrajectoryGeneration options:\n");
    printf("\t--watchdog ([s] default: \"%f\")\n",watchdog);
    printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");

    yarp::os::Property options;
    options.put("device", "remote_controlboard");
    options.put("remote", "/teoSim/trunkAndRightArm");
    options.put("local", "/local/trunkAndRightArm");


    if(rf.check("watchdog")) watchdog = rf.find("watchdog").asFloat64();

    clientRightArm.open(options);

        if(!clientRightArm.isValid()){
        std::fprintf(stderr, "[fail] Unable to connect to /teoSim/trunkAndRightArm\n");
        return false;
    }
    // Open views
    bool ok=true;
    ok=ok && clientRightArm.view(lim); // Get the joint limits view
    ok=ok && clientRightArm.view(mod); // Get the control mode view
    ok=ok && clientRightArm.view(iEncoders); // Get the encoders view
    ok=ok && clientRightArm.view(pos); // Get the position control view

    if (!ok)
     {
         std::printf("ERROR: Problems acquiring robot interface\n");
         return false;
     }
     std::printf("SUCCESS: Acquired robot interface\n");

    pos->getAxes(&axes);
    std::printf("Axes: %d\n", axes);

    
    yarp::os::Bottle qrMin, qrMax;
    for(unsigned int joint=0;joint<axes;joint++)
    {
        double min, max;
        lim->getLimits(joint,&min,&max);
        qrMin.addDouble(min);
        qrMax.addDouble(max);
        yInfo("Joint %d limits: [%f,%f]",joint,min,max);
    }

    yarp::os::Property rightArmSolverOptions;
    std::string rightKinPath = rf.findFileByName("teo-fixedTrunk-rightArm-fetch.ini");;
    rightArmSolverOptions.fromConfigFile(rightKinPath);
    rightArmSolverOptions.put("device","KdlSolver");
    rightArmSolverOptions.put("mins", yarp::os::Value::makeList(qrMin.toString().c_str()));
    rightArmSolverOptions.put("maxs", yarp::os::Value::makeList(qrMax.toString().c_str()));
    rightArmSolverOptions.put("ik", "st"); // to use screw theory IK
    rightArmSolverDevice.open(rightArmSolverOptions);

    

    if( ! rightArmSolverDevice.isValid() )
    {
        yError() << "KDLSolver solver device for right-arm is not valid";
        return false;
    }

    if( ! rightArmSolverDevice.view(iCartesianSolver) )
    {
        yError() << "Could not view iCartesianSolver in KDLSolver";
        return false;
    }

    std::printf("Configuration done ok!");
    std::printf("--------------------------------------------------------------\n");



    inTrajectoryGenerationPort.open("/trajectoryArmGeneration/rpc:s");
    inTrajectoryGenerationPort.setReader(*this);

    // auto space(std::make_shared<ob::RealVectorStateSpace>(axes));
    // ob::RealVectorBounds bounds{axes};

    // double min,max;
    // for(int i=0; i<axes; i++){
    //     lim->getLimits(i,&min,&max);
    //     bounds.setLow(i, min);
    //     bounds.setHigh(i, max);
    //     printf("Joint: %d Min: %f Max: %f\n",i, min, max);
    // }
    // space->setBounds(bounds);

    // si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));
    // si->setStateValidityChecker(std::bind(&TrajectoryGeneration::isValid, this, std::placeholders::_1 ));
    // si->setup();

    // Get current joints values

    std::vector<double> q(axes); 

    yInfo()<<"Getting encoders values";
    while (!iEncoders->getEncoders(q.data()))
    {
        yarp::os::Time::delay(0.1);
    }

    yInfo()<<"Current encoders data: "<< q[0]<<" "<< q[1]<<" "<<q[2]<<" "<<q[3]<<" "<<q[4]<<" "<<q[5]<<" "<<q[6]<<" "<<q[7];
    std::vector<double> x;
    if ( ! iCartesianSolver->fwdKin(q,x)){
        yError()<<"fwdKin failed";
    }
    yInfo()<<"Start:"<< x[0]<<" "<<x[1]<<" "<<x[2]<<" "<<x[3]<<" "<<x[4]<<" "<<x[5];

    // ob::ScopedState<ob::RealVectorStateSpace> start(space);
    // // Set our starting state
    // for(int i=0; i<axes; i++)
    //     start->as<ob::RealVectorStateSpace::StateType>() ->values[i] = q[i];

    // ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
    // pdef->addStartState(start);

    // ob::State * startState = pdef->getStartState(0);

    // if(isValid(startState))
    //     yInfo()<<"Valid starting state";
    // else{
    //     yInfo()<<"Not valid starting state";
    // }

    std::vector<double> xGoal;
    std::vector<double> qGoal(axes);
    qGoal[0] = 5; qGoal[1] = 10; qGoal[2] = -50; qGoal[3] = 10; qGoal[4] = 40; qGoal[5] = -50; qGoal[6] = 70; qGoal[7] = -20; 

    if(!iCartesianSolver->fwdKin(qGoal, xGoal)){
        yError()<<"fwdKin failed";
    }
    yInfo()<<"Goal:"<< xGoal[0]<<" "<<xGoal[1]<<" "<<xGoal[2]<<" "<<xGoal[3]<<" "<<xGoal[4]<<" "<<xGoal[5];

    // std::vector<double> qGoalComputed(axes);
    // iCartesianSolver->invKin(xGoal,q, qGoalComputed);
    // yInfo()<<"Current encoders data: "<< qGoalComputed[0]<<" "<< qGoalComputed[1]<<" "<<qGoalComputed[2]<<" "<<qGoalComputed[3]<<" "<<qGoalComputed[4]<<" "<<qGoalComputed[5]<<" "<<qGoalComputed[6]<<" "<<qGoalComputed[7];




    

    // ob::ScopedState<ob::RealVectorStateSpace>goal(space);
    // goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 5;
    // goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 10;

    // goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = -50;
    // goal->as<ob::RealVectorStateSpace::StateType>()->values[3] = 10;
    // goal->as<ob::RealVectorStateSpace::StateType>()->values[4] = 40;
    // goal->as<ob::RealVectorStateSpace::StateType>()->values[5] = -50;
    // goal->as<ob::RealVectorStateSpace::StateType>()->values[6] = 70;
    // goal->as<ob::RealVectorStateSpace::StateType>()->values[7] = -20;

    // //goal.random();


    // pdef->setGoalState(goal);

    // auto plannerRRT(new og::RRTstar(si));
    // plannerRRT->setRange(1.5);
    // plannerRRT->setPruneThreshold(0.6);
    // plannerRRT->setTreePruning(true);
    // plannerRRT->setInformedSampling(true);

    // ob::PlannerPtr planner(plannerRRT);
    // planner->setProblemDefinition(pdef);
    // planner->setup();
    // bool solved = planner->solve(5.0);

    // if (solved){
    //     ob::PathPtr path = pdef->getSolutionPath();
    //     yInfo()<<"Found solution";
    //     path->print(std::cout);
    //     std::size_t j=0;
    //     og::PathGeometric * pth = path->as<og::PathGeometric>();

    //     while(j< pth->getStateCount()){
            
    //         ob::State *s=pth->getState(j);
    //         ob::RealVectorStateSpace::StateType *q = s->as<ob::RealVectorStateSpace::StateType>();
    //         yInfo()<<q->values[0]<<" "<<q->values[1]<<" "<<q->values[2]<<" "<<q->values[3]<<" "<<q->values[4]<<" "<<q->values[5];
    //         for(int joint=0; joint<axes; joint++){
    //             pos->positionMove(joint, q->values[joint]);
    //         }

    //         yInfo("Moving to next state in the path");

    //         yarp::os::Time::delay(0.05);
    //         j++;
    //     }
    // }


    return true;



    // options.fromString( rf.toString() );  //-- Should get noMirror, noRGBMirror, noDepthMirror, video modes...
    // options.put("device",strRGBDDevice);  //-- Important to override in case there is a "device" in the future
    // options.put("localImagePort",strRGBDLocal+"/rgbImage:i");
    // options.put("localDepthPort",strRGBDLocal+"/depthImage:i");
    // options.put("localRpcPort",strRGBDLocal+"/rpc:o");
    // options.put("remoteImagePort",strRGBDRemote+"/rgbImage:o");
    // options.put("remoteDepthPort",strRGBDRemote+"/depthImage:o");
    // options.put("remoteRpcPort",strRGBDRemote+"/rpc:i");
    // //if(rf.check("noMirror")) options.put("noMirror",1);  //-- Replaced by options.fromString( rf.toString() );

    // if(!dd.open(options))
    // {
    //     yError("Bad RGBDDevice \"%s\"...\n",strRGBDDevice.c_str());
    //     return false;
    // }
    // yInfo("RGBDDevice available.\n");

    // if (! dd.view(iRGBDSensor) )
    // {
    //     yError("RGBDDevice bad view.\n");
    //     return false;
    // }
    // yInfo("RGBDDevice ok view.\n");

    // //-----------------OPEN LOCAL PORTS------------//
    // std::string portPrefix("/rgbdDetection");
    // portPrefix += strRGBDRemote;
    // if(!outRgbImgPort.open(portPrefix + "/croppedImg:o"))
    // {
    //     yError("Bad outRgbImgPort.open\n");
    //     return false;
    // }
    // if(!outDepthImgPort.open(portPrefix + "/croppedDepthImg:o"))
    // {
    //     yError("Bad outDepthImgPort.open\n");
    //     return false;
    // }


    // if(!outPort.open(portPrefix + "/state:o"))
    // {
    //     yError("Bad outPort.open\n");
    //     return false;
    // }

    // if(!inFileNamePort.open(portPrefix + "/imageName:i"))
    // {
    //     yError("Bad fileNamePort.open\n");
    //     return false;
    // }

    // yarp::os::Network::connect("/server/imageName", portPrefix + "/imageName:i");

    // if(cropSelector != 0)
    // {
    //     outCropSelectorImg.open(strRGBDLocal + "/cropSelector/img:o");
    //     inCropSelectorPort.open(strRGBDLocal + "/cropSelector/state:i");
    // }

    // segmentorThread.setIRGBDSensor(iRGBDSensor);
    // segmentorThread.setOutRgbImgPort(&outRgbImgPort);
    // segmentorThread.setOutDepthImgPort(&outDepthImgPort);
    // segmentorThread.setOutPort(&outPort);
    // segmentorThread.setFileNamePort(&inFileNamePort);


    // segmentorThread.setCropSelector(cropSelector);
    // if(cropSelector != 0) {
    //     segmentorThread.setOutCropSelectorImg(&outCropSelectorImg);
    //     segmentorThread.setInCropSelectorPort(&inCropSelectorPort);
    // }


    // if(!segmentorThread.init(rf))
    // {
    //     yError("Bad segmentorThread.init\n");
    //     return false;
    // }
    // yInfo("--- end: configure\n");
}


/************************************************************************/

double TrajectoryGeneration::getPeriod()
{
    return watchdog;  // Fixed, in seconds, the slow thread that calls updateModule below
}

/************************************************************************/

bool TrajectoryGeneration::updateModule()
{
    yInfo()<<"TrajectoryGeneration alive...";
    return true;
}

/************************************************************************/

bool TrajectoryGeneration::interruptModule()
{
    yInfo()<<"TrajectoryGeneration closing...";
    // segmentorThread.stop();
    // outRgbImgPort.interrupt();
    // outDepthImgPort.interrupt();
    // outPort.interrupt();
    // inFileNamePort.interrupt();
    // if(cropSelector != 0) {
    //     outCropSelectorImg.interrupt();
    //     inCropSelectorPort.interrupt();
    // }
    // dd.close();
    // outRgbImgPort.close();
    // outDepthImgPort.close();
    // inFileNamePort.close();
    // outPort.close();
    // if(cropSelector != 0) {
    //     outCropSelectorImg.close();
    //     inCropSelectorPort.close();
    // }
    return true;
}

/************************************************************************/
bool TrajectoryGeneration::read(yarp::os::ConnectionReader & connection){
    auto * writer = connection.getWriter();
    yarp::os::Bottle command;

    
    if (!command.read(connection) || writer == nullptr)
    {
        return false;
    }

    if (command.size() != DEFAULT_POSESIZE)
    {
        printf("WARNING: Target pose must be x y z qx qy qz qw.\n");
        yarp::os::Bottle reply;
        reply.addString("Target pose not ok");
        return reply.write(*writer);
    }

    printf("command: %s\n", command.toString().c_str());

    for(int i=0; i<DEFAULT_POSESIZE; i++){
        printf("%f", command.get(i).asFloat32());
    }

    yarp::os::Bottle reply;
    reply.addString("Pose received");
    yarp::os::Time::delay(5.0);
    return reply.write(*writer);
}

/************************************************************************/
// bool TrajectoryGeneration::isValid(const ob::State *state){
//     //yInfo()<<"Checking if the state is valid";
//     const ob::RealVectorStateSpace::StateType *q = state->as<ob::RealVectorStateSpace::StateType>();
//     //yInfo()<<q->values[0];
//     if(si->satisfiesBounds(state)){
//         //yInfo("Bounds satisfied");
//         return true;
//     }
//     return false;
    
// }
