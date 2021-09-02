# Copyright: (C) 2015 Vislab Laboratory - Instituto de Sistemas e Robotica - Instituto Superior Tecnico
# Authors: Plinio Moreno
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.9)

project(vizzyYarpIcub)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)

message("#debug ${PROJECT_BINARY_DIR}")
message("${CMAKE_CURRENT_SOURCE_DIR}")

include_directories(${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files)
include_directories(${PROJECT_BINARY_DIR}/_yarp_idl_)
list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

include(YarpInstallationHelpers)
include(iCubHelpers)

set(CMAKE_BUILD_TYPE "Debug")
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")

set(YARP_ROS_IDL_MSG_DIR ${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files)
file(MAKE_DIRECTORY ${YARP_ROS_IDL_MSG_DIR})

get_filename_component(DIR_ONE_ABOVE ../ ABSOLUTE)
message("${DIR_ONE_ABOVE}")
#execute_process(COMMAND ln -s ${DIR_ONE_ABOVE}/vizzy_msgs/srv/MotorsShutdown.srv ${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files/MotorsShutdown.srv)

execute_process(COMMAND yarpidl_rosmsg TactSensorArray WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files" OUTPUT_QUIET)# OUTPUT_FILE /home/vizzy/tmp)
execute_process(COMMAND yarpidl_rosmsg Int16 WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files" OUTPUT_QUIET)# OUTPUT_FILE /home/vizzy/tmp2)

yarp_idl_to_dir(vizzy_msgs/MotorsShutdown.srv ${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files SOURCES HEADERS INCLUDES)
yarp_idl_to_dir(vizzy_msgs/ArmDown.srv ${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files SOURCES HEADERS INCLUDES)
yarp_idl_to_dir(vizzy_msgs/ShutdownStartYarpRunProcess.srv ${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files SOURCES HEADERS INCLUDES)

add_subdirectory(app)
add_subdirectory(src)

#message(${YARP_MODULES_INSTALL_DIR})
#message(${YARP_CONFIG_INSTALL_DIR})
#message(${YARP_CONTEXTS_INSTALL_DIR})


