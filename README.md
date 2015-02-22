# vizzy

This is Vizzy's "oh so amazing" repository!

This repository contains the necessary tools to interact with Vizzy - both on simulation and real robot usage.

The real robot uses two different middlewares for distinct body parts (YARP for the upperbody and ROS for the mobile base). In simulation there are two options in terms of middleware - one aligned with the real robot (using YARP and ROS) and one aiming simulation experiences (using exclusively ROS).

![vizzy with its arms opened](vizzy_images/vizzy_open_arms.jpg)

## Packages Description

* [vizzy_launch](vizzy_launch):
* [vizzy_description](vizzy_description):
* [vizzy_gazebo](vizzy_gazebo):
* [vizzy_navigation](vizzy_navigation):
* [vizzy_control](vizzy_control):
* [vizzy_sensors](vizzy_sensors):

## Environment

* Operating System (one of the two)
  * [Ubuntu 12.04](http://releases.ubuntu.com/precise/)
  * [Ubuntu 14.04](http://releases.ubuntu.com/trusty/)
* Middleware
  * [ROS](http://www.ros.org/)
  * [YARP](http://wiki.icub.org/yarpdoc/)
* Other Dependencies
  * [GAZEBO](http://gazebosim.org/)
  * [gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins)

## Download and Setup

You should have a catkin workspace on your file system to be able to compile the code. If you don't know how to do this please follow [these instructions](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

Having your catkin_workspace configured you are ready to open a terminal and run the following instructions:
	cd /path/to/your/catkin/workspace

## Compile

## Run