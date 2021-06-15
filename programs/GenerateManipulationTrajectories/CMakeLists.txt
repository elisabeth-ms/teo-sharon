# Copyright: Universidad Carlos III de Madrid (C) 2021
# Authors: Elisabeth Menendez
# CopyPolicy: Released under the terms of the LGPLv2.1 or later

if(ENABLE_generateManipulationTrajectories)

set(KEYWORD "generateManipulationTrajectories")

find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(Armadillo 5.4 REQUIRED)
find_package(orocos_kdl 1.4)
find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)
find_package(PythonLibs 3.0 REQUIRED)

set(GLFWLIB_SEARCH_PATH /usr/lib/x86_64-linux-gnu/ /usr/local/lib/)
find_library(GLFW_LIB glfw PATHS ${GLFWLIB_SEARCH_PATH})


set(LIBRARIES
	${OPENGL_LIBRARIES}
	${GLEW_LIBRARIES}
	${GLFW_LIB}
	${ARMADILLO_LIBRARIES}
)

# Check compiler compatibility
include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


include_directories(
	include/
	include/arpack-arma/include
	${ARMADILLO_INCLUDE_DIR}
	${GLEW_INCLUDE_DIR}
)

include_directories(${PYTHON3_INCLUDE_DIRS} ${NumPy_INCLUDE_DIRS})

# Declare the support files needed for OpenGL2 demos
set(GL2_SUPPORT_SOURCES
	utils/imgui_impl_glfw_gl2.cpp
	utils/imgui.cpp
	utils/imgui_draw.cpp
	utils/gfx_ui.cpp
)


# Declare the support files needed for OpenGL2 demos (using gfx.h)
set(GFX_SUPPORT_SOURCES
	${GL2_SUPPORT_SOURCES}
	utils/gfx.cpp
)


# Declare the support files needed for OpenGL2 (using gfx2.h) demos
set(GFX2_SUPPORT_SOURCES
	${GL2_SUPPORT_SOURCES}
	utils/gfx2.cpp
)


# Declare the support files needed for OpenGL3 (using GLSL & gfx3.h) demos
set(GFX3_SUPPORT_SOURCES
	utils/imgui_impl_glfw_gl3.cpp
	utils/imgui.cpp
	utils/imgui_draw.cpp
	utils/gfx_ui.cpp
	utils/gfx3.cpp
)




# Retrieve a list of targets defined elsewhere.
get_property(_exported_targets GLOBAL PROPERTY TEO_SHARON_TARGETS)

# Configure executable build target.
add_executable(tpgmm tpgmm.cpp)

# Link external libraries.

target_link_libraries(tpgmm ${LIBRARIES} ${orocos_kdl_LIBRARIES}   ${PYTHON_LIBRARIES}
Python3::NumPy
)

target_include_directories(tpgmm PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
                                             $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
											 ${orocos_kdl_INCLUDE_DIRS}
                                            )


endif(ENABLE_generateManipulationTrajectories)
