// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/**
 * @ingroup teo-sharon_programs
 * \defgroup exampleProgram
 *
 * @brief Creates an instance of sharon::ColorsDetection
 *
 * @section exampleProgram_legal Legal
 *
 * Copyright: 2021 (C) Universidad Carlos III de Madrid
 *
 * Author: Elisabeth Menendez
 *
 * CopyPolicy: This program is free software; you can redistribute it and/or modify
 * it under the terms of the LGPLv2.1 or later
 *
 * <hr>
 *
 * This file can be edited at exampleProgram
 */

#include <cstdio>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "TrajectoryGeneration.hpp"
#include <yarp/dev/PolyDriver.h>

int main(int argc, char ** argv)
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    //rf.setDefaultContext("kinematics");
    rf.configure(argc, argv);

    yarp::dev::PolyDriver dTrajectoryGeneration("trajectoryGeneration");
    if (!dTrajectoryGeneration.isValid()) {
        printf("trajectoryGeneration not available\n");
        exit(1);
    }
    
    TrajectoryGeneration *trajectoryGeneration;
    dTrajectoryGeneration.view(trajectoryGeneration);
    if (trajectoryGeneration!=NULL) {
        printf("*** Device can work\n");
    }
    
    // sharon::TrajectoryGeneration mod;

    // if (rf.check("help"))
    // {
    //     return mod.runModule(rf);
    // }

    // std::printf("Run \"%s --help\" for options.\n", argv[0]);
    // std::printf("%s checking for yarp network... ", argv[0]);
    // std::fflush(stdout);

    // yarp::os::Network yarp;

    // if (!yarp.checkNetwork())
    // {
    //     std::fprintf(stderr, "[fail]\n%s found no yarp network (try running \"yarpserver &\"), bye!\n", argv[0]);
    //     return 1;
    // } else std::printf("[ok]\n");

    // return mod.runModule(rf);
}
