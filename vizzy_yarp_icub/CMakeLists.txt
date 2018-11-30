# Copyright: (C) 2015 Vislab Laboratory - Instituto de Sistemas e Robotica - Instituto Superior Tecnico
# Authors: Plinio Moreno
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.9)

project(vizzyYarpIcub)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)

execute_process(COMMAND yarpidl_rosmsg TactSensorArray WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files" OUTPUT_QUIET)
execute_process(COMMAND yarpidl_rosmsg Int16 WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files" OUTPUT_QUIET)

include_directories(${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

include(YarpInstallationHelpers)
include(iCubHelpers)

set(CMAKE_BUILD_TYPE "Release")
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)

set(YARP_ROS_IDL_MSG_DIR ${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files)
file(MAKE_DIRECTORY ${YARP_ROS_IDL_MSG_DIR})
add_subdirectory(app)
add_subdirectory(src)

#message(${YARP_MODULES_INSTALL_DIR})
#message(${YARP_CONFIG_INSTALL_DIR})
#message(${YARP_CONTEXTS_INSTALL_DIR})


