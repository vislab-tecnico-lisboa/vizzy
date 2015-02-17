# vizzy_navigation

ROS package to support navigation on Vizzy simulator

## Environment

All the functionality was tested on machines configured with:

+ Ubuntu 12.04 LTS
+ ROS Hydro (install it [here](http://wiki.ros.org/hydro/Installation/Ubuntu))

## Prerequisites

This package is a complement to [vizzy-simulator repository](https://github.com/vislab-tecnico-lisboa/vizzy-simulator). Complete the Download and Setup section to configure the paths.

## Download and Setup

Open a terminal and navigate to the folder where you want to download the package.

Now, run:

    git clone https://github.com/vislab-tecnico-lisboa/vizzy_navigation.git

After completing the download, you should execute some instructions to setup your environment:

    sudo gedit ~/.bashrc

At the end of the file, add the following line:

    export ROS_PACKAGE_PATH=[Path to the downloaded folder]:$ROS_PACKAGE_PATH

Save the file and run:

    source ~/.bashrc
