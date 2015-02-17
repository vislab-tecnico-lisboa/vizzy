# vizzy_simulator

ROS package to simulate Vizzy on Gazebo

## Environment

All the functionality was tested on machines configured with:

+ Ubuntu 12.04 LTS
+ ROS Hydro (install it [here](http://wiki.ros.org/hydro/Installation/Ubuntu))
+ YARP

## Prerequisites

You need to download and setup two repositories to be able to correctly simulate Vizzy:

+ [vizzy-description](https://github.com/vislab-tecnico-lisboa/vizzy-description)
+ [vizzy-navigation](https://github.com/vislab-tecnico-lisboa/vizzy-navigation)
+ [gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins)

For this last step (the gazebo-yarp-plugins one) you will need to do a little hack because in order for the GAZEBO to find the plugin, the setup of the variable $GAZEBO_PLUGIN_PATH is not enough.

    sudo gedit /usr/share/gazebo/setup.sh

Now add the path to your gazebo-yarp-plugins build folder to the line containing "export GAZEBO_PLUGIN_PATH". You should get something like this:

    export GAZEBO_PLUGIN_PATH=/usr/lib/gazebo-1.9/plugins:/home/miguel/github/gazebo-yarp-plugins/build

Now, add the same path to this other file:

    sudo gedit /usr/share/gazebo-1.9/setup.sh 

You will need a ROS package that you might not have installed yet, run:

    sudo apt-get install ros-hydro-eband-local-planner

## Download and Setup

Open a terminal and navigate to the folder where you want to download the package.

Now, run:

    git clone https://github.com/vislab-tecnico-lisboa/vizzy_simulator.git

After completing the download, you should execute some instructions to setup your environment:

    sudo gedit ~/.bashrc

At the end of the file, add the following lines:

    export ROS_PACKAGE_PATH=[Path to the downloaded folder]:$ROS_PACKAGE_PATH
    export GAZEBO_MODEL_PATH=[Path to the downloaded folder]:$GAZEBO_MODEL_PATH

Save the file and run:

    source ~/.bashrc

## Running

If you have followed everything as stated in here, you should now be able to run Gazebo and RViz to simulate Vizzy robot.

Open a terminal to run yarpserver for upperbody simulation:

    yarpserver

Open another terminal:

    roslaunch vizzy_simulator gazebo.launch

In case you want you to perform SLAM in simulation you can run instead:

    roslaunch vizzy_simulator gmapping_gazebo.launch
