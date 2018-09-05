cmake_minimum_required(VERSION 2.8.3)
project(vizzy_control)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  message_filters
  roscpp
  sensor_msgs
  std_msgs
  tf2
  image_transport
  pcl_ros
  tf2_ros
  message_generation
  nodelet
  vizzy_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface

)

add_message_files(
  FILES
  motorsArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES foveated_stereo_ros
#  CATKIN_DEPENDS cv_bridge geometry_msgs message_filters roscpp sensor_msgs std_msgs tf
#  DEPENDS system_lib
#   CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  #${CUDA_INCLUDE_DIRS}
)

add_definitions(-std=c++11)

add_executable(gaze_sim src/Gaze.cpp src/GazeSimulation.cpp)
## Specify libraries to link a library or executable target against
add_dependencies(gaze_sim vizzy_msgs_generate_messages_cpp)
target_link_libraries(gaze_sim
   ${catkin_LIBRARIES}
)

add_executable(gaze_real src/Gaze.cpp src/GazeReal.cpp)
add_dependencies(gaze_real vizzy_msgs_generate_messages_cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(gaze_real
   ${catkin_LIBRARIES}
)

add_executable(gaze_client src/GazeClient.cpp)
add_dependencies(gaze_client vizzy_msgs_generate_messages_cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(gaze_client
   ${catkin_LIBRARIES}
)

add_executable(gaze_client_wasd src/GazeClientWASD.cpp)
add_dependencies(gaze_client_wasd vizzy_msgs_generate_messages_cpp)
## Specify libraries to link a library or executable target against
target_link_libraries(gaze_client_wasd
   ${catkin_LIBRARIES}
)
## Declare a cpp library
add_library(fixation_point_lib
   src/FixationPoint.cpp
)

## Declare a cpp executable
add_executable(fixation_point_node src/FixationPointRos.cpp)



## Declare a cpp executable
add_executable(follow_trajectory_action_server src/VizzyTrajActionServerNode.cpp src/VizzyTrajectoryActionServer.cpp)
target_link_libraries(follow_trajectory_action_server
   ${catkin_LIBRARIES}
)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
#add_dependencies(fixation_point_node visualize_ball_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(fixation_point_node
   ${catkin_LIBRARIES}
    fixation_point_lib
)

