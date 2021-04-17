# Copyright: Universidad Carlos III de Madrid (C) 2021
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

if(ENABLE_trajectoryGeneration)

set(KEYWORD "trajectoryGeneration")

find_package(YARP 3.3 REQUIRED COMPONENTS os dev)


find_package(orocos_kdl 1.4)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)


find_package(ompl REQUIRED)
find_package(fcl REQUIRED)

# Retrieve a list of targets defined elsewhere.
get_property(_exported_targets GLOBAL PROPERTY TEO_SHARON_TARGETS)

# Configure executable build target.
add_executable(${KEYWORD} main.cpp
                          TrajectoryGeneration.hpp
                          TrajectoryGeneration.cpp
                          )

add_executable(clientTrajectoryGenerationTargetPoseGazebo clientTrajectoryGenerationTargetPoseGazebo.cpp)

# Link external libraries.

target_link_libraries(${KEYWORD} 
                                 YARP::YARP_init
                                 YARP::YARP_dev
                                 ${orocos_kdl_LIBRARIES}
                                 ROBOTICSLAB::KdlVectorConverterLib
                                 ROBOTICSLAB::KinematicRepresentationLib
				 ${OMPL_LIBRARIES} 
                                 fcl
                                 )

target_link_libraries(clientTrajectoryGenerationTargetPoseGazebo
                                 YARP::YARP_init
                                 YARP::YARP_dev
                                 ${orocos_kdl_LIBRARIES}
                                 ROBOTICSLAB::KdlVectorConverterLib
                                 ROBOTICSLAB::KinematicRepresentationLib
                                 )

target_include_directories(${KEYWORD} PRIVATE ${orocos_kdl_INCLUDE_DIRS}  ${OMPL_INCLUDE_DIRS}     ${FCL_INCLUDE_DIRS})
target_include_directories(clientTrajectoryGenerationTargetPoseGazebo PRIVATE ${orocos_kdl_INCLUDE_DIRS}  ${OMPL_INCLUDE_DIRS}     ${FCL_INCLUDE_DIRS})

# Create installation rule.
install(TARGETS ${KEYWORD}
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        COMPONENT trajectory_generation)

endif(ENABLE_trajectoryGeneration)
