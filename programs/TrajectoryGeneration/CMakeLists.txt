# Copyright: Universidad Carlos III de Madrid (C) 2021
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

if(ENABLE_trajectoryGeneration)

set(KEYWORD "trajectoryGeneration")

find_package(YARP 3.3 REQUIRED COMPONENTS os dev)


find_package(orocos_kdl 1.4)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)


find_package(ompl REQUIRED)
find_package(fcl 0.6.0 REQUIRED)

# Retrieve a list of targets defined elsewhere.
# get_property(_exported_targets GLOBAL PROPERTY TEO_SHARON_TARGETS)


yarp_prepare_plugin(TrajectoryGeneration
                    CATEGORY device
                    TYPE TrajectoryGeneration
                    INCLUDE TrajectoryGeneration.hpp
		    DEFAULT ON)

yarp_add_plugin(TrajectoryGeneration TrajectoryGeneration.cpp TrajectoryGeneration.hpp)

# Configure executable build target.
# add_executable(${KEYWORD} main.cpp
#                           TrajectoryGeneration.hpp
#                           TrajectoryGeneration.cpp
#                           )

#add_executable(clientTrajectoryGenerationTargetPoseGazebo clientTrajectoryGenerationTargetPoseGazebo.cpp)

# Link external libraries.

target_link_libraries(TrajectoryGeneration
                                 YARP::YARP_init
                                 YARP::YARP_dev
                                 ${orocos_kdl_LIBRARIES}
                                 ROBOTICSLAB::KdlVectorConverterLib
                                 ROBOTICSLAB::KinematicRepresentationLib
				                 ${OMPL_LIBRARIES} 
                                 fcl
                                 CheckSelfCollisionLibrary
                                 )
target_include_directories(TrajectoryGeneration PRIVATE 
                                ${orocos_kdl_INCLUDE_DIRS}  
                                ${OMPL_INCLUDE_DIRS}     
                                ${FCL_INCLUDE_DIRS})

# list(APPEND YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS YARP_os
#                                                 YARP_sig
#                                                 YARP_dev
#                                                 ROBOTICSLAB::KdlVectorConverterLib
#                                                 ROBOTICSLAB::KinematicRepresentationLib
#                                                 ${OMPL_LIBRARIES} 
#                                                 fcl
#                                                 CheckSelfCollisionLibrary)

yarp_install(
                TARGETS TrajectoryGeneration
                LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
                ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
                YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}
                )




# add_executable(testTrajectoryGeneration main.cpp)
# target_include_directories(testTrajectoryGeneration PRIVATE {trajectoryGeneration_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS}  
#                                                             ${OMPL_INCLUDE_DIRS}     
#                                                             ${FCL_INCLUDE_DIRS})
# target_link_libraries(testTrajectoryGeneration trajectoryGeneration)

# set(YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS ${YARP_${YARP_PLUGIN_MASTER}_PRIVATE_DEPS} PARENT_SCOPE)
# yarp_install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/trajectoryGeneration.ini"  DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})                                 
# target_link_libraries(clientTrajectoryGenerationTargetPoseGazebo
#                                  YARP::YARP_init
#                                  YARP::YARP_dev
#                                  ${orocos_kdl_LIBRARIES}
#                                  ROBOTICSLAB::KdlVectorConverterLib
#                                  ROBOTICSLAB::KinematicRepresentationLib
#                                  CheckSelfCollisionLibrary
#                                  )

#target_include_directories(clientTrajectoryGenerationTargetPoseGazebo PRIVATE ${orocos_kdl_INCLUDE_DIRS}  ${OMPL_INCLUDE_DIRS}     ${FCL_INCLUDE_DIRS})

# Create installation rule.
# install(TARGETS ${KEYWORD}
#         DESTINATION ${CMAKE_INSTALL_BINDIR}
#         COMPONENT trajectory_generation)

endif(ENABLE_trajectoryGeneration)
