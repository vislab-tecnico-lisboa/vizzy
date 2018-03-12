# Copyright: (C) 2011 Department of Robotics Brain and Cognitive Sciences - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini - Duarte Aragão - Plinio Moreno
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.6)
set(PROJECTNAME fingerLimbControl)
project(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
include(iCubOptions)
include(iCubHelpers)

# Thrift IDL
include(YarpIDL)
yarp_add_idl(IDL_GEN_FILES ${PROJECTNAME}.thrift)

set(folder_source main.cpp)
source_group("Source Files" FILES ${folder_source})

include_directories(${ICUB_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR})

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

add_executable(${PROJECTNAME} pid.cpp ForceReadingThread.cpp fingerLimbControl.cpp ${PROJECTNAME}.thrift ${IDL_GEN_FILES})
target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES})
#install(TARGETS ${PROJECTNAME} DESTINATION ${ICUB_APPLICATIONS_PREFIX})
