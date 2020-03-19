#!/usr/bin/env bash

# Plinio Moreno @ Vislab

ROS_VERSION=melodic
printf "\n [Vizzy]: Did you execute: source ~/.bashrc in your terminal?\n"
read -p "[Vizzy]: What is your ROS VERSION? Default: $ROS_VERSION   " NEW_ROS_VERSION

if [ ! -z "$NEW_ROS_VERSION" ]; then
  ROS_VERSION=$NEW_ROS_VERSION
  echo "[Vizzy]: I'm going to install ROS packages of $ROS_VERSION"
fi

source $HOME/.bashrc
sudo apt-get install -y ros-$ROS_VERSION-smach
sudo apt-get install -y ros-$ROS_VERSION-pcl-ros
sudo apt-get install -y ros-$ROS_VERSION-nav2d
sudo apt-get install -y ros-$ROS_VERSION-tf2-geometry-msgs
sudo apt-get install -y ros-$ROS_VERSION-gazebo-*
sudo apt-get install -y ros-$ROS_VERSION-image-proc
sudo apt-get install -y ros-$ROS_VERSION-moveit-*
sudo apt-get install -y ros-$ROS_VERSION-opencv*
sudo apt-get install -y ros-$ROS_VERSION-image-proc
sudo apt-get install -y ros-$ROS_VERSION-gazebo-ros-control
sudo apt-get install -y ros-$ROS_VERSION-move-base
sudo apt-get install -y ros-$ROS_VERSION-amcl
sudo apt-get install -y ros-$ROS_VERSION-joy
sudo apt-get install -y ros-$ROS_VERSION-map-server
sudo apt-get install -y ros-$ROS_VERSION-joint-state-controller
sudo apt-get install -y ros-$ROS_VERSION-velocity-controllers
sudo apt-get install -y ros-$ROS_VERSION-position-controllers
sudo apt-get install -y ros-$ROS_VERSION-joint-trajectory-controller
sudo apt-get install -y ros-$ROS_VERSION-teb-local-planner
sudo apt-get install -y ros-$ROS_VERSION-laser-filters
sudo apt-get install -y libsuitesparse-dev
sudo apt-get install -y ros-$ROS_VERSION-fake-localization
sudo apt-get install -y ros-$ROS_VERSION-move-slow-and-clear
sudo apt-get install -y ros-$ROS_VERSION-move-base-flex
sudo apt-get install -y ros-$ROS_VERSION-global-planner
sudo apt-get install -y git

CATKIN_WS=$HOME/catkin_ws
read -p "[Vizzy]: Type your catkin workspace folder, Default: $CATKIN_WS   " NEW_CATKIN_WS
if [ ! -z "$NEW_CATKIN_WS" ]; then
  CATKIN_WS=$NEW_CATKIN_WS
  echo "[Vizzy]: I'm going to download my repository at $CATKIN_WS/src"
fi

cd "$CATKIN_WS/src"
git clone https://github.com/vislab-tecnico-lisboa/vizzy
cd vizzy
if [ $ROS_VERSION = "melodic" ]; then
  git checkout melodic-devel
fi
echo "export GAZEBO_MODEL_PATH=$CATKIN_WS/src/vizzy/vizzy_gazebo:\$GAZEBO_MODEL_PATH" >> $HOME/.bashrc
echo "source /$CATKIN_WS/devel/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc

cd $CATKIN_WS
catkin_make

# install Groot requirements
sudo apt-get install -y qtbase5-dev libqt5svg5-dev libzmq3-dev libdw-dev

# install behavior tree packages
cd "$CATKIN_WS/src"
git clone https://github.com/vislab-tecnico-lisboa/vizzy_speech
git clone https://github.com/vislab-tecnico-lisboa/vizzy_behavior_trees
git clone https://github.com/joao-avelino/vizzy_playground
git clone https://github.com/joao-avelino/Groot
cd Groot 
git submodule update --init --recursive
cd ..
git clone https://github.com/joao-avelino/BehaviorTree.CPP

cd $CATKIN_WS
catkin_make

