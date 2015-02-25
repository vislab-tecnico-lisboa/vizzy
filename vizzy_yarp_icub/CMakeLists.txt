# Copyright: (C) 2015 Vislab Laboratory
# Authors: Plinio Moreno
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.9)

project(vizzyYarpIcub)

find_package(YARP REQUIRED)
find_package(ICUB REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUB_MODULE_PATH})

include(YarpInstallationHelpers)
include(iCubHelpers)

set(CMAKE_BUILD_TYPE "Release")
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/lib)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)


add_subdirectory(app)
add_subdirectory(src)

#message(${YARP_MODULES_INSTALL_DIR})
#message(${YARP_CONFIG_INSTALL_DIR})
#message(${YARP_CONTEXTS_INSTALL_DIR})


