# Copyright: Universidad Carlos III de Madrid (C) 2021
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

if(ENABLE_colorsDetection)

set(KEYWORD "colorsDetection")

# Retrieve a list of targets defined elsewhere.
get_property(_exported_targets GLOBAL PROPERTY TEO_SHARON_TARGETS)

# Configure executable build target.
add_executable(${KEYWORD} main.cpp
                          ColorsDetection.cpp
                          ColorsDetection.hpp
                          SegmentorThread.cpp
                          SegmentorThread.hpp)

# Link external libraries.
target_link_libraries(${KEYWORD} YARP::YARP_OS
                                 YARP::YARP_init
                                 YARP::YARP_dev
                                 ROBOTICSLAB::ColorDebug
                                 ROBOTICSLAB::VisionInterfaces
                                 ${OpenCV_LIBRARIES}
                                 ${_exported_targets})

# Create installation rule.
install(TARGETS ${KEYWORD}
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        COMPONENT colors_detection)

endif()
