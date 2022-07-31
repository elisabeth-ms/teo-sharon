#include <cstdio>

#include <yarp/os/Network.h>
#include <yarp/os/ResourceFinder.h>

#include "CollisionsVisualization.hpp"

int main(int argc, char ** argv)
{
    yarp::os::ResourceFinder rf;
    rf.setVerbose(true);
    rf.setDefaultContext("collisionsVisualization");
    rf.setDefaultConfigFile("collisionsVisualization.ini");
    rf.configure(argc, argv);

    sharon::CollisionsVisualization mod;

    if (rf.check("help"))
    {
        return mod.runModule(rf);
    }

    std::printf("Run \"%s --help\" for options.\n", argv[0]);
    std::printf("%s checking for yarp network... ", argv[0]);
    std::fflush(stdout);

    yarp::os::Network yarp;

    if (!yarp.checkNetwork())
    {
        std::fprintf(stderr, "[fail]\n%s found no yarp network (try running \"yarpserver &\"), bye!\n", argv[0]);
        return 1;
    } else std::printf("[ok]\n");

    return mod.runModule(rf);
}