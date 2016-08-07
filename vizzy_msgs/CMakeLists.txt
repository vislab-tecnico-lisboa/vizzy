cmake_minimum_required(VERSION 2.8.3)
project(vizzy_msgs)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface

)

add_action_files(
  DIRECTORY action
  FILES Gaze.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES vizzy_msgs
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)
