# Copyright: Universidad Carlos III de Madrid (C) 2021
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

if(ENABLE_mapHumanHandTrajectoryToRobotJointSpace)

find_package (Eigen3 REQUIRED)

set(KEYWORD "mapHumanHandTrajectoryToRobotJointSpace")

# Retrieve a list of targets defined elsewhere.
get_property(_exported_targets GLOBAL PROPERTY TEO_SHARON_TARGETS)

# Configure executable build target.
add_executable(${KEYWORD} MapHumanHandTrajectoryToRobotJointSpace.hpp
                            MapHumanHandTrajectoryToRobotJointSpace.cpp
                          )

# Link external libraries.

target_link_libraries(${KEYWORD} nlopt
                                 Eigen3::Eigen
                                 CheckSelfCollisionLibrary
                                 ${orocos_kdl_LIBRARIES}
                                 )

target_include_directories(${KEYWORD} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                             $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                                             ${orocos_kdl_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})


endif(ENABLE_mapHumanHandTrajectoryToRobotJointSpace)
