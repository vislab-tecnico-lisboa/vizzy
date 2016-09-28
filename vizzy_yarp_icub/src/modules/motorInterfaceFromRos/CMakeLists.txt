cmake_minimum_required(VERSION 2.8.7)

# find YARP
find_package(YARP REQUIRED)

# add YARP include directories
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# set up our program
add_executable(headMotorsInterface head_motors_interface.cpp)
add_executable(leftArmMotorsInterface left_arm_motors_interface.cpp)
add_executable(leftArmMotorsInterfaceOneTopic left_arm_motors_interface_one_topic.cpp)
add_executable(leftArmFollowTrajBridge left_arm_trajectory_action_bridge.cpp)

# link with YARP libraries
target_link_libraries(headMotorsInterface ${YARP_LIBRARIES})
target_link_libraries(leftArmMotorsInterface ${YARP_LIBRARIES})
target_link_libraries(leftArmMotorsInterfaceOneTopic ${YARP_LIBRARIES})
target_link_libraries(leftArmFollowTrajBridge ${YARP_LIBRARIES})
