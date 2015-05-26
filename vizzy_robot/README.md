# vizzy-robot

Package to initialize interfaces for real robot usage

## Environment

All the functionality was tested on machines configured with:

+ Ubuntu 12.04 LTS
+ ROS Hydro (install it [here](http://wiki.ros.org/hydro/Installation/Ubuntu))
+ YARP

## Prerequisites

You need to download and setup two repositories to be able to correctly simulate Vizzy:

+ [vizzy-description](https://github.com/vislab-tecnico-lisboa/vizzy-description)
+ [vizzy-navigation](https://github.com/vislab-tecnico-lisboa/vizzy-navigation)

You will need some ROS packages that you might not have installed yet, run:

    sudo apt-get install ros-hydro-eband-local-planner
    sudo apt-get install ros-hydro-segway-rmp
    sudo apt-get install ros-hydro-hokuyo-node

## Download and Setup

Open a terminal and navigate to the folder where you want to download the package.

Now, run:

    git clone https://github.com/vislab-tecnico-lisboa/vizzy-robot.git

After completing the download, you should execute some instructions to setup your environment:

    sudo gedit ~/.bashrc

At the end of the file, add the following lines:

    export ROS_PACKAGE_PATH=[Path to the downloaded folder]:$ROS_PACKAGE_PATH

Save the file and run:

    source ~/.bashrc

## Running

If you have followed everything as stated in here, you should now be able to initiate the interfaces for Vizzy's mobile platform.

Open a terminal:

    roslaunch vizzy-robot real_robot.launch

Additionally you can use the RViz for visualization purposes by running:

    roslaunch vizzy-navigation rviz.launch