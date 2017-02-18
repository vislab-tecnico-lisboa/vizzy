cmake_minimum_required(VERSION 2.8.3)
project(vizzy_navigation)


find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    geometry_msgs
    nav2d_operator)


catkin_package(
  CATKIN_DEPENDS
    sensor_msgs
    geometry_msgs
    roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a cpp executable
add_executable(teleop_joy_node src/teleop_joy.cpp)
target_link_libraries(teleop_joy_node
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_executable(reroute_twist2nav2d_node src/twist2nav2d.cpp)
target_link_libraries(reroute_twist2nav2d_node
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES}
)

add_dependencies(reroute_twist2nav2d_node nav2d_operator_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

foreach(dir config launch rviz)
  install(DIRECTORY ${dir}/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/amcl.launch)
  roslaunch_add_file_check(launch/gmapping.launch)
  roslaunch_add_file_check(launch/move_base_eband_local.launch)
  roslaunch_add_file_check(launch/move_base_eband_remote.launch)
  roslaunch_add_file_check(launch/rviz.launch)
  ## launch/navigation.launch not tested until
  ## ros-drivers/openni_launch#10 is fixed.
endif ()
