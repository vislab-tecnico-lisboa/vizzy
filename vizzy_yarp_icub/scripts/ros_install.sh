#!/usr/bin/env bash

# Plinio Moreno @ Vislab

ROS_VERSION=melodic
read -p "[Vizzy]: What is your ROS VERSION? Default: $ROS_VERSION   " NEW_ROS_VERSION

if [ ! -z "$NEW_ROS_VERSION" ]; then
  ROS_VERSION=$NEW_ROS_VERSION
  echo "[Vizzy]: I'm going to install ROS $ROS_VERSION"
fi

if [ $ROS_VERSION = "melodic" ]; then
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
elif [ $ROS_VERSION = "kinetic" ];then
  sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
  sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
fi

sudo apt update
sudo apt install ros-$ROS_VERSION-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/$ROS_VERSION/setup.bash" >> $HOME/.bashrc

printf "\n [Vizzy]: After this script finishes, verify that you execute: source ~/.bashrc in your terminal\n"

