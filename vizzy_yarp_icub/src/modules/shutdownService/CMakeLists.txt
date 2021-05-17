# Copyright (C) 2021 Vislab
# Author: Plinio Moreno
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.8)
if(COMMAND cmake_policy)
cmake_policy(SET CMP0003 NEW)
endif(COMMAND cmake_policy)

set(PROJECTNAME shutdownService)
PROJECT(${PROJECTNAME})
find_package(YARP)
file(GLOB folder_source *.cpp)
file(GLOB folder_header *.h)
source_group("Source Files" FILES ${folder_source})
source_group("Header Files" FILES ${folder_header})
include_directories(${YARP_INCLUDE_DIRS} ${PROJECT_BINARY_DIR}/_yarp_idl_)
message("${INCLUDE_DIRECTORIES}")
message("${PROJECT_BINARY_DIR}")
#add_executable(${PROJECTNAME} ${folder_source} ${folder_header})
#target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})

add_executable(shutdownYarpIdlRemoteExample shutdownYarpIdlRemoteExample.cpp)
# link with YARP libraries
target_link_libraries(shutdownYarpIdlRemoteExample ${YARP_LIBRARIES})

add_executable(armDownService armDownService.cpp)
# link with YARP libraries
target_link_libraries(armDownService ${YARP_LIBRARIES})

add_executable(idleArmsShouldersService idleArmsShouldersService.cpp)
# link with YARP libraries
target_link_libraries(idleArmsShouldersService ${YARP_LIBRARIES})

add_executable(shutdownStartYarpRunProcess shutdownStartYarpRunProcess.cpp)
# link with YARP libraries
target_link_libraries(shutdownStartYarpRunProcess ${YARP_LIBRARIES})