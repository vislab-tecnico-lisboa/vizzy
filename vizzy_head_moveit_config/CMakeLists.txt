cmake_minimum_required(VERSION 2.8.3)
project(vizzy_head_moveit_config)

find_package(catkin REQUIRED)

add_definitions(-std=c++11)

catkin_package()

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
