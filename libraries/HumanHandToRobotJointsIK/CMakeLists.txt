# Copyright: Universidad Carlos III de Madrid (C) 2021
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

if(ENABLE_humanHandToRobotJointsIK)

find_package (Eigen3 REQUIRED)
set (CMAKE_CXX_STANDARD 11)
set(KEYWORD "HumanHandToRobotJointsIK")

# Retrieve a list of targets defined elsewhere.
get_property(_exported_targets GLOBAL PROPERTY TEO_SHARON_TARGETS)

# Configure executable build target.
add_library(${KEYWORD} SHARED HumanHandToRobotJointsIK.hpp
                            HumanHandToRobotJointsIK.cpp
                          )

set_target_properties(${KEYWORD} PROPERTIES PUBLIC_HEADER "HumanHandToRobotJointsIK.hpp")

# Link external libraries.

target_link_libraries(${KEYWORD} nlopt
                                 Eigen3::Eigen
                                 CheckSelfCollisionLibrary
                                 ${orocos_kdl_LIBRARIES}
                                 )

target_include_directories(${KEYWORD} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                             $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                                             ${orocos_kdl_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})
install(TARGETS ${KEYWORD}
                                             EXPORT TEO_SHARON
                                             LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT human_hand_to_robot_joints_ik
                                             ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT human_hand_to_robot_joints_ik
                                             RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT human_hand_to_robot_joints_ik
                                             PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT human_hand_to_robot_joints_ik)
                                     

endif(ENABLE_humanHandToRobotJointsIK)
