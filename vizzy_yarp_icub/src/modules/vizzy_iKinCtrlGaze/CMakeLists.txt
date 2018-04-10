cmake_minimum_required(VERSION 2.8)

SET(PROJECTNAME vizzy_iKinGazeCtrl)

PROJECT(${PROJECTNAME})

set(DEBUG_DEFINES
    #RPC_DEBUG=1
    #TPM_DEBUG=1
)

set(RELEASE_DEFINES
    #DO_MCAST=1
    ) 

find_package(IPOPT)
find_package(OpenCV)

#SET(GCC_COVERAGE_COMPILE_FLAGS "-Wno-delete-non-virtual-dtor -Wno-reorder")
#list(APPEND CMAKE_CXX_FLAGS ${GCC_COVERAGE_COMPILE_FLAGS})
#SET( CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -std=c++11" )
#set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH} ${ICUB_MODULE_PATH})

#set(ICUB_APPLICATIONS_PREFIX "${CMAKE_SOURCE_DIR}/bin"  CACHE PATH "Application path prefix")


FILE(GLOB folder_source *.cpp *.cc *.c)
FILE(GLOB folder_header *.h)
SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

#SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${iKin_INCLUDE_DIRS} ${YARP_INCLUDE_DIRS} ${IPOPT_INCLUDE_DIRS} ${fKin_INCLUDE_DIRS})
#message("${fKin_INCLUDE_DIRS}")

ADD_DEFINITIONS(${IPOPT_DEFINITIONS})
# Set up our main executable.
IF (folder_source)
  ADD_EXECUTABLE(${PROJECTNAME} ${folder_source} ${folder_header})
ELSE (folder_source)
  MESSAGE(FATAL_ERROR "No source code files found. Please add something")
ENDIF (folder_source)

#MESSAGE("${OpenCV_LIBRARIES}")
#MESSAGE("${OpenCV_DIR}")

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
  LINK_FLAGS "${IPOPT_LINK_FLAGS}" 
  )

