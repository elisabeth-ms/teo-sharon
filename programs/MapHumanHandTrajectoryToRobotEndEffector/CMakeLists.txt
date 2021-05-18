# Copyright: Universidad Carlos III de Madrid (C) 2021
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

if(ENABLE_mapHumanHandTrajectoryToRobotEndEffector)

set(KEYWORD "mapHumanHandTrajectoryToRobotEndEffector")

# Retrieve a list of targets defined elsewhere.
get_property(_exported_targets GLOBAL PROPERTY TEO_SHARON_TARGETS)

# Configure executable build target.
add_executable(${KEYWORD} MapHumanHandTrajectoryToRobotEndEffector.hpp
                          MapHumanHandTrajectoryToRobotEndEffector.cpp
                          )

# Link external libraries.

target_link_libraries(${KEYWORD} 
                                 YARP::YARP_init
                                 YARP::YARP_dev
                                 CheckSelfCollisionLibrary
                                 )

endif(ENABLE_mapHumanHandTrajectoryToRobotEndEffector)
