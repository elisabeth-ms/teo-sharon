# Copyright: Universidad Carlos III de Madrid (C) 2021
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

if(ENABLE_CheckSelfCollisionLibrary)

set(KEYWORD "CheckSelfCollisionLibrary")
set (CMAKE_CXX_STANDARD 11)
find_package(orocos_kdl 1.4 QUIET)
find_package(fcl 0.6.0 REQUIRED)
find_package(Eigen3 CONFIG REQUIRED)
# Configure library build target.
add_library(${KEYWORD} SHARED CheckSelfCollisionLibrary.cpp
                              CheckSelfCollisionLibrary.hpp)

# Set project properties.
set_target_properties(${KEYWORD} PROPERTIES PUBLIC_HEADER "CheckSelfCollisionLibrary.hpp")

# Link external libraries.
target_link_libraries(${KEYWORD} PUBLIC YARP::YARP_OS
                                 ${orocos_kdl_LIBRARIES}
                                 fcl
                                 PRIVATE ROBOTICSLAB::ColorDebug)

# Define include paths.
target_include_directories(${KEYWORD} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                             $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                                             ${orocos_kdl_INCLUDE_DIRS}
                                             ${FCL_INCLUDE_DIRS})

# Create installation rule.
install(TARGETS ${KEYWORD}
        EXPORT TEO_SHARON
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT check_self_collision_library
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT check_self_collision_library
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT check_self_collision_library
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT check_self_collision_library)

# Export current target.
set_property(GLOBAL APPEND PROPERTY TEO_SHARON_TARGETS ${KEYWORD})

endif()
