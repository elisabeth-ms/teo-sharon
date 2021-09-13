# Copyright: Universidad Carlos III de Madrid (C) 2021
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

### Go through single applications
#add_subdirectory(prog)
yarp_install(DIRECTORY conf DESTINATION ${TEO-SHARON_APPLICATIONS_INSTALL_DIR})
yarp_install(FILES applications/demoSharonSim.xml DESTINATION ${TEO-SHARON_APPLICATIONS_INSTALL_DIR})
yarp_install(FILES applications/demoSharon.xml DESTINATION ${TEO-SHARON_APPLICATIONS_INSTALL_DIR})
