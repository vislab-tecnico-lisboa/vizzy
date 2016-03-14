# Copyright: (C) 2015 Vislab Laboratory - Instituto de Sistemas e Robotica - Instituto Superior Tecnico
# Authors: Plinio Moreno
# CopyPolicy: Released under the terms of the GNU GPL v2.0.
cmake_minimum_required(VERSION 2.8.7)

SET(PROJECTNAME fKinLib)
PROJECT(${PROJECTNAME})
SET(folder_source vizzyFwd.cpp)
SET(folder_header vizzy/vizzyFwd.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})
INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${iKin_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS})

set(fKin_INCLUDE_DIRS ${PROJECT_SOURCE_DIR} CACHE STRING "fKin include directory")

ADD_LIBRARY(${PROJECTNAME} ${folder_source} ${folder_header})
TARGET_LINK_LIBRARIES(${PROJECTNAME} ${YARP_LIBRARIES} ${ICUB_LIBRARIES})
