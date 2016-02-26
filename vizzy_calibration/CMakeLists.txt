cmake_minimum_required(VERSION 2.8.3)
project(vizzy_calibration)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  ar_track_alvar

)

## System dependencies are found with CMake's conventions
find_package(PCL REQUIRED)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES opencv_calibration
#  CATKIN_DEPENDS opencv2 roscpp
#  DEPENDS system_lib
)


include_directories(${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(two_camera_extrinsic_calibration src/arm_camera_calibration.cpp)

## Specify libraries to link a library or executable target against
 target_link_libraries(two_camera_extrinsic_calibration
   ${catkin_LIBRARIES} ${PCL_LIBRARIES}
 )

add_executable(markersNoKinect src/IndividualMarkersNoKinect.cpp)
target_link_libraries(markersNoKinect ${catkin_LIBRARIES})


