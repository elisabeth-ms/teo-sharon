# Copyright: Universidad Carlos III de Madrid (C) 2022
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

if(ENABLE_mapHumanToRobotMotionPerFrameIK)


find_package(PkgConfig REQUIRED)
find_package(orocos_kdl 1.4)


find_library(trac-ik NAMES trac_ik libtrac_ik)

include_directories(${PYTHON_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS} ${track_ik_INCLUDEDIR} ${track_ik_INCLUDEDIRS})

set(KEYWORD "mapHumanToRobotMotionPerFrameIK")
set(CMAKE_THREAD_LIBS_INIT "-lpthread")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(CMAKE_HAVE_THREADS_LIBRARY 1)
set(CMAKE_USE_WIN32_THREADS_INIT 0)
set(CMAKE_USE_PTHREADS_INIT 1)
set(THREADS_PREFER_PTHREAD_FLAG ON)
# Retrieve a list of targets defined elsewhere.
get_property(_exported_targets GLOBAL PROPERTY TEO_SHARON_TARGETS)



# Configure executable build target.
add_executable(${KEYWORD} MapHumanToRobotMotionPerFrameIK.hpp
                MapHumanToRobotMotionPerFrameIK.cpp)

# Link external libraries.

target_link_libraries(${KEYWORD} PRIVATE
                                 nlopt
                                 Eigen3::Eigen
                                 HumanMotionData
                                 ${orocos_kdl_LIBRARIES}
                                 trac_ik
                                 )



target_include_directories(${KEYWORD} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                             $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                                             ${orocos_kdl_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
                                             ${track_ik_INCLUDE_DIR} ${nlopt_include_DIR}
                                             )

# target_include_directories(${KEYWORD} PRIVATE ${Boost_INCLUDE_DIRS})


endif(ENABLE_mapHumanToRobotMotionPerFrameIK)
