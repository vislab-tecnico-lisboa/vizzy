cmake_minimum_required(VERSION 2.8.7)

# find YARP
find_package(YARP REQUIRED)

# add YARP include directories
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# set up our program
add_executable(readAndWriteTopicsTest ReadAndWriteTopicsTest.cpp)
add_executable(GazeActionBridge Gaze_Action_Bridge.cpp)
add_executable(fixationPointBridge FixationPointRos.cpp)
add_executable(ballPointBridge ParticleFilterBallPointRos.cpp)

# link with YARP libraries
target_link_libraries(readAndWriteTopicsTest ${YARP_LIBRARIES})
target_link_libraries(GazeActionBridge ${YARP_LIBRARIES})
target_link_libraries(fixationPointBridge ${YARP_LIBRARIES})
target_link_libraries(ballPointBridge ${YARP_LIBRARIES})
