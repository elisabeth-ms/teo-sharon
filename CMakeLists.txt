# Copyright: Universidad Carlos III de Madrid (C) 2021
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

# Start a project. Set the LANGUAGES option according to your
# needs, e.g. 'C', 'CXX', 'NONE'.
project(TEO_SHARON VERSION 0.1.0 LANGUAGES CXX)

# Set module path.
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake
                              ${CMAKE_SOURCE_DIR}/cmake/modules
                              ${CMAKE_SOURCE_DIR}/cmake/find-modules)

# Hard dependencies.
find_package(YARP 3.4 REQUIRED COMPONENTS os dev)
find_package(YCM 0.11 REQUIRED)
find_package(COLOR_DEBUG REQUIRED)
find_package(ROBOTICSLAB_VISION REQUIRED)
find_package(OpenCV REQUIRED)
find_package(orocos_kdl 1.4 QUIET)
find_package(ROBOTICSLAB_KINEMATICS_DYNAMICS REQUIRED)
find_package(PCL REQUIRED)
# Soft dependencies.
find_package(Doxygen QUIET)
find_package(GTestSources 1.6.0 QUIET)

# Add WITH_PLUGINS if this projects exports YARP devices.
yarp_configure_external_installation(teo-sharon)

install(DIRECTORY models worlds
        DESTINATION share/gazebo)


# Retrieve current TEO_SHARON version.
include(TEO_SHARONVersion)

# Define standard paths, check system configuration.
include(TEO_SHARONSystem)

# Setup build options.
include(TEO_SHARONOptions)



# Define and enter subdirectories.
add_subdirectory(cmake)
add_subdirectory(libraries)
add_subdirectory(programs)
add_subdirectory(tests)
add_subdirectory(share)
add_subdirectory(doc)

# Output configuration files.
include(TEO_SHARONDescribe)

# Invoke CPack from here.
include(TEO_SHARONPackage)

set(YARP_FORCE_DYNAMIC_PLUGINS TRUE CACHE INTERNAL "Force dynamic plugins")
yarp_configure_external_installation(teo_sharon WITH_PLUGINS)


# Generate a report of enabled/disabled features.
feature_summary(WHAT ALL)
