cmake_minimum_required(VERSION 2.8.3)
project(vizzy_control)

find_package(catkin REQUIRED COMPONENTS 
  roscpp
  rospy
  std_msgs
  joint_state_controller
  effort_controllers
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vizzy_control
#  CATKIN_DEPENDS ros_control ros_controllers
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)