cmake_minimum_required(VERSION 2.8.7)

# find YARP
find_package(YARP REQUIRED)

# add YARP include directories
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

execute_process(COMMAND yarpidl_rosmsg vizzy_msgs/CartesianGoal WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files" OUTPUT_QUIET)
execute_process(COMMAND yarpidl_rosmsg vizzy_msgs/CartesianFeedback WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files" OUTPUT_QUIET)
execute_process(COMMAND yarpidl_rosmsg Float32 WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files" OUTPUT_QUIET)
execute_process(COMMAND yarpidl_rosmsg Float64 WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files" OUTPUT_QUIET)
execute_process(COMMAND yarpidl_rosmsg JointTrajectory WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files" OUTPUT_QUIET)
execute_process(COMMAND yarpidl_rosmsg Bool WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files" OUTPUT_QUIET)
execute_process(COMMAND yarpidl_rosmsg std_msgs/Int16 WORKING_DIRECTORY "${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files" OUTPUT_QUIET)
execute_process(COMMAND yarpidl_rosmsg motorsArray.msg WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}" OUTPUT_QUIET)

include_directories(${PROJECT_BINARY_DIR}/yarpidl_rosmsg_files)
# set up our program
add_executable(torsoMotorInterface torso_motor_interface.cpp)
add_executable(torsoMotorInterfaceSim torso_motor_interface_sim.cpp)
add_executable(headMotorsInterface head_motors_interface.cpp)
add_executable(leftArmMotorsInterface left_arm_motors_interface.cpp)
add_executable(leftArmMotorsInterfaceOneTopic left_arm_motors_interface_one_topic.cpp)
add_executable(leftArmFollowTrajBridge left_arm_trajectory_action_bridge.cpp)
add_executable(followTrajectoryBridge follow_trajectory_action_bridge.cpp)
add_executable(pos2_posDirTest position_direct_test_left_arm.cpp)
add_executable(cartesianRosBridge cartesian_controller_action_bridge.cpp)

# link with YARP libraries
target_link_libraries(headMotorsInterface ${YARP_LIBRARIES})
target_link_libraries(torsoMotorInterfaceSim ${YARP_LIBRARIES})
target_link_libraries(torsoMotorInterface ${YARP_LIBRARIES})
target_link_libraries(leftArmMotorsInterface ${YARP_LIBRARIES})
target_link_libraries(leftArmMotorsInterfaceOneTopic ${YARP_LIBRARIES})
target_link_libraries(leftArmFollowTrajBridge ${YARP_LIBRARIES})
target_link_libraries(followTrajectoryBridge ${YARP_LIBRARIES})
target_link_libraries(pos2_posDirTest ${YARP_LIBRARIES})
target_link_libraries(cartesianRosBridge ${YARP_LIBRARIES})
