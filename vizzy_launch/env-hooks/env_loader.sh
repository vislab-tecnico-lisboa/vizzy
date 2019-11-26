#!/bin/bash

export PYTHONPATH=$HOME/repositories/OpenFace/build/python${PYTHONPATH:+:${PYTHONPATH}}$
export OPENCV_LIB=$HOME/repositories/opencv/build/lib
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$OPENCV_LIB
source /opt/ros/kinetic/setup.bash
source /home/vizzy/catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://10.1.3.1:11311
export ROS_IP=10.1.3.1
export YARP_ROOT=/home/vizzy/yarp_repositories/yarp
export YARP_DIR=$YARP_ROOT/build
export YARP_ROBOT_NAME=vizzy
export ICUB_ROOT=/home/vizzy/yarp_repositories/icub-main
export ICUB_DIR=$ICUB_ROOT/build
export VIZZY_YARP_ICUB_ROOT=/home/vizzy/catkin_ws/src/vizzy/vizzy_yarp_icub/
export VIZZY_YARP_ICUB_DIR=$VIZZY_YARP_ICUB_ROOT/build
export icub_firmware_shared_DIR=/home/vizzy/yarp_repositories/icub-firmware-shared/build
export ICUBcontrib_DIR=/home/vizzy/yarp_repositories/icub-contrib-common/build
export YARP_DATA_DIRS=$YARP_DIR/share/yarp:$ICUB_DIR/share/iCub:$ICUBcontrib_DIR/share/ICUBcontrib:$VIZZY_YARP_ICUB_DIR/share/yarp
export YARP_COLORED_OUTPUT=1
export YARP_FORWARD_LOG_ENABLE=1
export PATH=$PATH:$YARP_DIR/bin:$ICUB_DIR/bin:$ICUBcontrib_DIR/bin:$VIZZY_YARP_ICUB_DIR/bin
export GAZEBO_PLUGIN_PATH=${GAZEBO_PLUGIN_PATH}:/home/vizzy/yarp_repositories/install/lib

exec "$@"
