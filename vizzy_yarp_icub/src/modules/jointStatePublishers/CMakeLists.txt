cmake_minimum_required(VERSION 2.8.7)

# find YARP
find_package(YARP REQUIRED)

# add YARP include directories
include_directories(${YARP_INCLUDE_DIRS})
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

# set up our program
add_executable(vizzy_right_arm vizzy_right_arm.cpp)
add_executable(vizzy_left_arm vizzy_left_arm.cpp)
add_executable(vizzy_head vizzy_head.cpp)
add_executable(vizzy_waist vizzy_waist.cpp)
add_executable(vizzy_head_manual vizzy_head_manual.cpp)

# link with YARP libraries
target_link_libraries(vizzy_right_arm ${YARP_LIBRARIES})
target_link_libraries(vizzy_left_arm ${YARP_LIBRARIES})
target_link_libraries(vizzy_head ${YARP_LIBRARIES})
target_link_libraries(vizzy_waist ${YARP_LIBRARIES})
target_link_libraries(vizzy_head_manual ${YARP_LIBRARIES})
