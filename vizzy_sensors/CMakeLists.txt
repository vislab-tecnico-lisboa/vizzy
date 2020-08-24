# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(vizzy_sensors)

find_package(catkin REQUIRED
  COMPONENTS
    laser_filters
    message_generation
    nodelet
    pcl_ros
    pluginlib
    roscpp
    roslint
    sensor_msgs
    gazebo_ros
    gazebo_msgs
    image_transport
    camera_info_manager
    )


find_package(gazebo REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    hokuyo_node
    laser_filters
    message_runtime
    nodelet
    pcl_ros
    sensor_msgs
    tf
    gazebo_ros
    image_transport
    camera_info_manager
  INCLUDE_DIRS
    include
  LIBRARIES
    vizzy_sensors_filters
)

include_directories(include 
                    ${catkin_INCLUDE_DIRS}
                    SYSTEM
                    ${GAZEBO_INCLUDE_DIRS})

link_directories(${GAZEBO_LIBRARY_DIRS})


## Declare a library
add_library(
    realsense_gazebo_plugin
    src/RealSensePlugin.cpp
    src/gazebo_ros_realsense.cpp
    )

target_link_libraries(realsense_gazebo_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
add_dependencies(realsense_gazebo_plugin ${catkin_EXPORTED_TARGETS})


## Declare a cpp executable


add_library(vizzy_sensors_filters src/vizzy_sensors_filters.cpp)
target_link_libraries(vizzy_sensors_filters 
  ${Boost_LIBRARIES} 
  ${catkin_LIBRARIES}
)

# Install the plugin and the plugin description
install (DIRECTORY include/vizzy_sensors/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install (TARGETS vizzy_sensors_filters
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES vizzy_sensors_plugins.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

# Install runtime launch and configuration files
foreach(dir config launch)
  install(DIRECTORY ${dir}/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()

# unit tests are enabled selectively
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch)
  roslaunch_add_file_check(launch/hokuyo)
endif()
