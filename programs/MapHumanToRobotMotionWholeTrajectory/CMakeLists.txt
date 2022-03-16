# Copyright: Universidad Carlos III de Madrid (C) 2021
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

if(ENABLE_mapHumanToRobotMotionWholeTrajectory)

find_package (Eigen3 REQUIRED)
find_package(Boost REQUIRED)

set(KEYWORD "mapHumanToRobotMotionWholeTrajectory")

# Retrieve a list of targets defined elsewhere.
get_property(_exported_targets GLOBAL PROPERTY TEO_SHARON_TARGETS)

# Configure executable build target.
add_executable(${KEYWORD} graph.h MapHumanToRobotMotion.hpp
                          MapHumanToRobotMotion.cpp)

# Link external libraries.

target_link_libraries(${KEYWORD} #nlopt
                                 Eigen3::Eigen
                                 CheckSelfCollisionLibrary
                                 ${orocos_kdl_LIBRARIES}
                                 )

target_include_directories(${KEYWORD} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                             $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                                             ${orocos_kdl_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

target_include_directories(${KEYWORD} PRIVATE ${Boost_INCLUDE_DIRS})


endif(ENABLE_mapHumanToRobotMotionWholeTrajectory)
