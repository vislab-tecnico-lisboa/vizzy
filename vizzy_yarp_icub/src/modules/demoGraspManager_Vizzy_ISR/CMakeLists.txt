# Copyright: (C) 2015 Vislab Laboratory - Instituto de Sistemas e Robotica - Instituto Superior Tecnico
# Authors: Plinio Moreno
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8)

SET(PROJECTNAME demoGraspManager_Vizzy_ISR)

PROJECT(${PROJECTNAME})

set(CMAKE_BUILD_TYPE Release)

set(DEBUG_DEFINES
    #RPC_DEBUG=1
    #TPM_DEBUG=1
)

set(RELEASE_DEFINES
    #DO_MCAST=1
    ) 

find_package(YARP)
find_package(ICUB)
find_package(OpenCV)

#set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${demoGraspManager_Vizzy_ISR_BINARY_DIR}/lib)
#set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${demoGraspManager_Vizzy_ISR_BINARY_DIR}/lib)
#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${demoGraspManager_Vizzy_ISR_BINARY_DIR}/bin)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH})

#set(ICUB_APPLICATIONS_PREFIX "${CMAKE_SOURCE_DIR}/bin"  CACHE PATH "Application path prefix")

FILE(GLOB folder_source *.cpp *.cc *.c)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

INCLUDE_DIRECTORIES(${fKin_INCLUDE_DIRS} ${iKin_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

# Set up our main executable.
#IF (folder_source)
ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})
#ELSE (folder_source)
#  MESSAGE(FATAL_ERROR "No source code files found. Please add something")
#ENDIF (folder_source)

#MESSAGE("If the directory ${demoGraspManager_Vizzy_ISR_SOURCE_DIR}/../../../kinematicInterface/vizzy_iKinGazeCtrl/iKinGazeCtrl/src exists everything is OK!")

TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} ${ICUB_LIBRARIES} ${OpenCV_LIBRARIES} fKinLib)
 
set_target_properties(
  ${PROJECTNAME}
  PROPERTIES
  COMPILE_DEFINITIONS_DEBUG
  "${DEBUG_DEFINES}"
  COMPILE_DEFINITIONS
  "${RELEASE_DEFINES}"
  COMPILE_FLAGS
  "-Wall -static"
  )

