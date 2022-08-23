# Copyright: Universidad Carlos III de Madrid (C) 2022
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

if(ENABLE_HumanMotionData)

set(KEYWORD "HumanMotionData")
set (CMAKE_CXX_STANDARD 11)

# Configure library build target.
add_library(${KEYWORD} SHARED HumanMotionData.cpp HumanMotionData.hpp)

# # Set project properties.
# set_target_properties(${KEYWORD} PROPERTIES PUBLIC_HEADER "GetHumanMotion.hpp")

# # Link external libraries.
# target_link_libraries(${KEYWORD} PUBLIC
#                                  ${orocos_kdl_LIBRARIES}
#                                  fcl
#                                  ${PCL_LIBRARIES}
#                                  )

# Define include paths.
target_include_directories(${KEYWORD} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                             $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
                                        #      ${orocos_kdl_INCLUDE_DIRS}
                                        #      ${FCL_INCLUDE_DIRS}
                                        )

# Create installation rule.
install(TARGETS ${KEYWORD}
        EXPORT TEO_SHARON
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT human_motion_data
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT human_motion_data
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT human_motion_data
        PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR} COMPONENT human_motion_data)

# Export current target.
set_property(GLOBAL APPEND PROPERTY TEO_SHARON_TARGETS ${KEYWORD})

endif()
