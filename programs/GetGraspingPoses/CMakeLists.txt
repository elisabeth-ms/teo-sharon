# Copyright: Universidad Carlos III de Madrid (C) 2021
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

if(ENABLE_getGraspingPoses)

set(KEYWORD "getGraspingPoses")

find_package(YARP 3.3 REQUIRED COMPONENTS os dev)
find_package(orocos_kdl 1.4)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)
find_package(SuperquadricLib 0.1.0.0 EXACT REQUIRED)

# Retrieve a list of targets defined elsewhere.
get_property(_exported_targets GLOBAL PROPERTY TEO_SHARON_TARGETS)
if(!PCL_FOUND)
        message(WARNING "pcl NOT found")
endif()


if(PCL_VERSION VERSION_LESS 1.9)
        message(WARNING  PCL_INCLUDE_DIRS)

        message(WARNING  "${PCL_INCLUDE_DIRS}")
	include_directories(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
	# workaround for PCL 1.8 (Ubuntu 18.04), exclude empty/blank strings as compile defs
	# list(FILTER PCL_DEFINITIONS EXCLUDE REGEX "^ +$")
	add_definitions(${PCL_DEFINITIONS})
endif()





# Configure executable build target.
add_executable(${KEYWORD} main.cpp
                          GetGraspingPoses.cpp
                          GetGraspingPoses.hpp
                          )

# Link external libraries.
target_link_libraries(${KEYWORD} YARP::YARP_OS
                                 YARP::YARP_init
                                 YARP::YARP_dev
         			 ${orocos_kdl_LIBRARIES}
                                 ROBOTICSLAB::KdlVectorConverterLib
                                 ROBOTICSLAB::KinematicRepresentationLib
                                 ${OpenCV_LIBRARIES}
                                 ${PCL_LIBRARIES}
                                 SuperquadricLib::SuperquadricLibModel
                                 ${_exported_targets})

target_include_directories(${KEYWORD} PRIVATE ${orocos_kdl_INCLUDE_DIRS})                                 
# Create installation rule.
install(TARGETS ${KEYWORD}
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        COMPONENT get_grasping_poses)

endif()
