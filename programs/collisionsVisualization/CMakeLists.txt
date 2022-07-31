# Copyright: Universidad Carlos III de Madrid (C) 2022
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

if(ENABLE_collisionsVisualization)

set(KEYWORD "collisionsVisualization")

# Retrieve a list of targets defined elsewhere.
get_property(_exported_targets GLOBAL PROPERTY TEMPLATE_NAME_TARGETS)

# Configure executable build target.
add_executable(${KEYWORD} main.cpp
                          CollisionsVisualization.cpp
                          CollisionsVisualization.hpp)

# Link external libraries.
target_link_libraries(${KEYWORD} YARP::YARP_os
                                 YARP::YARP_init
                                 YARP::YARP_dev
                                 CheckSelfCollisionLibrary
                                 ${_exported_targets})

# Create installation rule.
install(TARGETS ${KEYWORD}
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        COMPONENT collisions_visualizer)

endif()