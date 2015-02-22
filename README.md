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

At any time you might need to install some more specific dependencies (like some missing ROS packages). Please open an issue in case you can't solve these or other dependencies.

## Download and Setup

You should have a catkin workspace on your file system to be able to compile the code. If you don't know how to do this please follow [these instructions](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

As soon as you have your catkin_workspace configured you are ready to open a terminal and run the following instructions:

    cd /path/to/your/catkin/workspace/src
    git clone https://github.com/vislab-tecnico-lisboa/vizzy

Be sure to add the source folder of your catkin workspace to the `ROS_PACKAGE_PATH` variable (skip this step if you already did it):

    gedit ~/.bashrc

Add the following line to the end of the file:

    export ROS_PACKAGE_PATH=/path/to/your/catkin/workspace/src:$ROS_PACKAGE_PATH

Save the file and run:

    source ~/.bashrc

You still need to add something to the `GAZEBO_MODEL_PATH`:

    gedit ~/.bashrc

(Yeah I know we could have done this on the previous step but I was afraid people would skip the last step without noticing this `export`...sorry)

    export GAZEBO_MODEL_PATH=/path/to/your/catkin/workspace/src/vizzy/vizzy_gazebo:$GAZEBO_MODEL_PATH

Save the file and run:

    source ~/.bashrc

You are now ready to compile the code!

## Compile

The compilation is quite straightforward. Just open a terminal:

    cd /path/to/your/catkin/workspace
    catkin_make

Hmm... does this seems to easy? I hope it does because you should now be ready to interact with Vizzy!

## Run

For now let's focus on simulation. Open a terminal:

    roslaunch vizzy_launch vizzy_simulation use_yarp:=true navigation:=true

Don't forget you'll need to have `yarpserver` running when you have the `use_yarp` argument set as true.

Feel free to play with the arguments as you want or to change the low-level launchers with more functionality.

## Issues

All kind of issues and contributions will be very welcome. Please get in touch on [our issues page](https://github.com/vislab-tecnico-lisboa/vizzy/issues) when help is needed!
