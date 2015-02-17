# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 2.8.3)
project(vizzy_navigation)

find_package(catkin REQUIRED)
catkin_package()

foreach(dir config launch rviz)
  install(DIRECTORY ${dir}/ 
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach()


# unit tests are enabled selectively
if (CATKIN_ENABLE_TESTING)
  find_package(roslaunch REQUIRED)
  roslaunch_add_file_check(launch/amcl.launch)
  roslaunch_add_file_check(launch/gmapping.launch)
  roslaunch_add_file_check(launch/move_base_eband.launch)
  roslaunch_add_file_check(launch/rviz.launch)
  ## launch/navigation.launch not tested until
  ## ros-drivers/openni_launch#10 is fixed.
endif ()
