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
using namespace teo;

/************************************************************************/

bool TrajectoryGeneration::configure(yarp::os::ResourceFinder &rf)
{   
    robot = rf.check("robot",yarp::os::Value(DEFAULT_ROBOT),"name of /robot to be used").asString();

    printf("--------------------------------------------------------------\n");
    if (rf.check("help"))
    {
        printf("BalanceTray options:\n");        
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--robot: %s [%s]\n",robot.c_str(),DEFAULT_ROBOT);
        ::exit(0);
    }

    

    

    // ------ RIGHT ARM -------

    yarp::os::Property rightArmOptions;
    rightArmOptions.put("device","remote_controlboard");
    rightArmOptions.put("remote","/"+robot+"/rightArm");
    rightArmOptions.put("local","/"+robot+"/rightArm");
    rightArmDevice.open(rightArmOptions);
    if(!rightArmDevice.isValid()) {
        yError() << "Robot trunkAndRightArm device not available";
        rightArmDevice.close();
        yarp::os::Network::fini();
        return false;
    }    

    // connecting our device with "IEncoders" interface
    if (!rightArmDevice.view(rightArmIEncoders) ) {
        yError() << "Problems acquiring rightArmIEncoders interface";
        return false;
    }
    else
    {
        yInfo() << "Acquired rightArmIEncoders interface";
        if(!rightArmIEncoders->getAxes(&numRightArmJoints))
            yError() << "Problems acquiring numRightArmJoints";
        else yWarning() << "Number of joints:" << numRightArmJoints;
    }

    // connecting our device with "control mode" interface, initializing which control mode we want (position)
    if (!rightArmDevice.view(rightArmIControlMode) ) {
        yError() << "Problems acquiring rightArmIControlMode interface";
        return false;
    } else
        yInfo() << "Acquired rightArmIControlMode interface";

    // connecting our device with "PositionControl" interface
    if (!rightArmDevice.view(rightArmIPositionControl) ) {
        yError() << "Problems acquiring rightArmIPositionControl interface";
        return false;
    } else
        yInfo() << "Acquired rightArmIPositionControl interface";

    // connecting our device with "PositionDirect" interface
    if (!rightArmDevice.view(rightArmIPositionDirect) ) {
        yError() << "Problems acquiring rightArmIPositionDirect interface";
        return false;
    } else
        yInfo() << "Acquired rightArmIPositionDirect interface";


    
    // ----- Configuring KDL Solver for trunk and right arm -----

    if( ! rightArmDevice.view(rightArmIControlLimits) ) {
        yError() << "Could not view iControlLimits in rightArmDevice";
        return false;
    }

    //  Getting the limits of each joint
    printf("---- Joint limits of right-arm ----\n");
    yarp::os::Bottle qrMin, qrMax;
        for(unsigned int joint=0;joint<numRightArmJoints;joint++)
        {
            double min, max;
            rightArmIControlLimits->getLimits(joint,&min,&max);
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
        yError() << "KDLSolver solver device for trunkAndRightArm is not valid";
        return false;
    }

    if( ! rightArmSolverDevice.view(rightArmICartesianSolver) )
    {
        yError() << "Could not view iCartesianSolver in KDLSolver";
        return false;
    }

    yInfo() << "Acquired rightArmICartesianSolver interface";

    yInfo()<<"Running";
    std::vector<double> position;
    
    

    std::vector<double> currentQ(numRightArmJoints);
    if ( ! rightArmIEncoders->getEncoders( currentQ.data() ) ){
        yError() << "Failed getEncoders() of right-arm";

    }

    auto space(std::make_shared<ob::RealVectorStateSpace>(numRightArmJoints));
    ob::RealVectorBounds bounds{numRightArmJoints};

    double min,max;
    for(int i=0; i<numRightArmJoints; i++){
         rightArmIControlLimits->getLimits(i,&min,&max);
         bounds.setLow(i, min);
         bounds.setHigh(i, max);
         printf("Joint: %d Min: %f Max: %f\n",i, min, max);
    }
    space->setBounds(bounds);

    si = ob::SpaceInformationPtr(new ob::SpaceInformation(space));
    si->setStateValidityChecker(std::bind(&TrajectoryGeneration::isValid, this, std::placeholders::_1 ));
    si->setup();


    //for(int i=0; i <numRightArmJoints; i++){
    //    currentQ[i] = currentQ[i]*180.0/3.141516;     
    //}
    yInfo()<<"Current encoders data: "<< currentQ;

    // inverse kinematic

    std::vector<double> xGoal;
    std::vector<double> qGoal(numRightArmJoints);
    qGoal[0] = -50; qGoal[1] = 10; qGoal[2] = 40; qGoal[3] = -50; qGoal[4] = 70; qGoal[5] = -20; 

    if(!rightArmICartesianSolver->fwdKin(qGoal, xGoal)){
        yError()<<"fwdKin failed";
    }
    yInfo()<<"Goal:"<< xGoal[0]<<" "<<xGoal[1]<<" "<<xGoal[2]<<" "<<xGoal[3]<<" "<<xGoal[4]<<" "<<xGoal[5];

    std::vector<double> desireQ(numRightArmJoints);
    if ( ! rightArmICartesianSolver->invKin(xGoal, currentQ, desireQ) )    {
        yError() << "invKin() failed";
        return false;
    }
    yInfo()<<"desireQ:"<<desireQ;

    ob::ScopedState<ob::RealVectorStateSpace> start(space);
    // Set our starting state
    for(int i=0; i<numRightArmJoints; i++)
        start->as<ob::RealVectorStateSpace::StateType>() ->values[i] = currentQ[i];



    pdef = ob::ProblemDefinitionPtr(new ob::ProblemDefinition(si));
    pdef->addStartState(start);

    ob::State * startState = pdef->getStartState(0);

    // if(isValid(startState))
    //      yInfo()<<"Valid starting state";
    // else{
    //     yInfo()<<"Not valid starting state";
    // }

    ob::ScopedState<ob::RealVectorStateSpace>goal(space);

    goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = -50;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 10;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[2] = 40;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[3] = -50;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[4] = 70;
    goal->as<ob::RealVectorStateSpace::StateType>()->values[5] = -20;

    // // //goal.random();


    pdef->setGoalState(goal);

    auto plannerRRT = (new og::RRTstar(si));
    plannerRRT->setRange(1.5);
    plannerRRT->setPruneThreshold(0.6);
    plannerRRT->setTreePruning(true);
    plannerRRT->setInformedSampling(true);

    planner =  ob::PlannerPtr(plannerRRT);


    bool solutionFound = computeDiscretePath(start, goal);


    if (solutionFound){
        followDiscretePath(pth);
    }


    //std::vector<double> rdsx; // right current point, right destination point
 	KDL::Frame frame = vectorToFrame(xGoal);
    //decodePose(rdsx, rdsxaa, coordinate_system::CARTESIAN, orientation_system::AXIS_ANGLE, angular_units::DEGREES );


    return true;
}

/************************************************************************/

bool TrajectoryGeneration::interruptModule()
{
    this->stop(); // stop JR3/keyboard reading thread
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

bool TrajectoryGeneration::threadInit(){
    initTime = Time::now();
    return true;
}

/**************** Sending Thread *************************/

void TrajectoryGeneration::run()
{    
    yInfo()<<"running";
}
/************************************************************************/
bool TrajectoryGeneration::isValid(const ob::State *state){
    //yInfo()<<"Checking if the state is valid";
    const ob::RealVectorStateSpace::StateType *q = state->as<ob::RealVectorStateSpace::StateType>();
    //yInfo()<<q->values[0];
    if(si->satisfiesBounds(state)){
        //yInfo("Bounds satisfied");
        return true;
    }
    return false;   
}

bool TrajectoryGeneration::computeDiscretePath(ob::ScopedState<ob::RealVectorStateSpace>start, ob::ScopedState<ob::RealVectorStateSpace>goal){
    
    pdef->clearStartStates();
    pdef->addStartState(start);
    
    ob::State * startState = pdef->getStartState(0);

    if(isValid(startState))
        yInfo()<<"Valid starting state";
    else{
        yInfo()<<"Not valid starting state";
        return false;
    }

    pdef->clearGoal();

    pdef->setGoalState(goal);

    auto goalState = goal->as<ob::State>();

    if(isValid(goalState))
        yInfo()<<"Valid goal state";
    else{
        yInfo()<<"Not valid goal state";
        return false;
    }

    planner->setProblemDefinition(pdef);
    planner->setup();


    bool solutionFound = planner->solve(5.0);
    if (solutionFound ==true){
        ob::PathPtr path = pdef->getSolutionPath();
        yInfo()<<"Found discrete path from start to goal";
        path->print(std::cout);
        pth = path->as<og::PathGeometric>();
        yInfo()<<pth->getStateCount();
    }

    return solutionFound;
}

bool TrajectoryGeneration::followDiscretePath(og::PathGeometric * discretePath){
        std::size_t j=0;
        while(j< discretePath->getStateCount()){
            
            ob::State *s=pth->getState(j);
            ob::RealVectorStateSpace::StateType *q = s->as<ob::RealVectorStateSpace::StateType>();
            yInfo()<<q->values[0]<<" "<<q->values[1]<<" "<<q->values[2]<<" "<<q->values[3]<<" "<<q->values[4]<<" "<<q->values[5];
            std::vector<double> refQ(numRightArmJoints);
            //for(int joint=0; joint<numRightArmJoints; joint++){
            //    refQ[joint] = q->values[joint];
            //}
            for(int joint=0; joint<numRightArmJoints; joint++){
                rightArmIPositionControl->positionMove(joint,q->values[joint]);
            }
            
            yInfo("Moving to next state in the path");

            yarp::os::Time::delay(0.05);
            j++;
        }

}