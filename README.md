# vizzy

This is Vizzy's "oh so amazing" repository!

This repository contains the necessary tools to interact with Vizzy - both on simulation and real robot usage.

The real robot uses two different middlewares for distinct body parts (YARP for the upperbody and ROS for the mobile base). In simulation there are two options in terms of middleware - one aligned with the real robot (using YARP and ROS) and one aiming simulation experiences (using exclusively ROS).

![vizzy grasping red ball](vizzy_images/vizzy_grasping_ball.jpg)

## Packages Description

* [vizzy_launch](vizzy_launch): Contains the launch files to interact with Vizzy (both on simulation and real robot usage). For most users it is the only package they need to directly use.
* [vizzy_description](vizzy_description): Contains all of Vizzy's description files (regarding mechanical, kinematic, visual, etc). You can use its launcher called `display.launch` to check the model with the Rviz graphical tool.
* [vizzy_gazebo](vizzy_gazebo): Holds the launch files needed to simulate the robot on the GAZEBO simulator.
* [vizzy_navigation](vizzy_navigation): Contains the launch files for several packages of the ROS navigation stack.
* [vizzy_control](vizzy_control): Low-level controllers for Vizzy's simulation.
* [vizzy_sensors](vizzy_sensors): Package holding the Hokuyo filters for better scannings.
* [vizzy_yarp_icub](vizzy_yarp_icub): YARP based controllers, drivers, libraries and modules. Cartesian controllers, gaze controller, ball tracker grasping demo, yoga demo.

## External Packages
* [vizzy_speech](https://github.com/vislab-tecnico-lisboa/vizzy_speech): Cointains nodes, messages and tools for speech.
* [vizzy_behavior trees](https://github.com/vislab-tecnico-lisboa/vizzy_behavior_trees): Behavior trees to control Vizzy's actions
* [extra_maps](https://github.com/vislab-tecnico-lisboa/extra_maps): Extra maps used for demos
* [vizzy_serial_interfaces](https://github.com/vislab-tecnico-lisboa/vizzy_serial_interfaces): ROS Components that require serial communcation
* [vizzy_tactile_drivers](https://github.com/vislab-tecnico-lisboa/vizzy_tactile_drivers): ROS drivers (and calibrator) for tactile sensors
* [vizzy-expression-driver](https://github.com/vislab-tecnico-lisboa/vizzy-expression-driver): Facial expression drivers to be used in the future



## Environment

Note that for simulation purposes one can ignore all YARP dependencies as explained on the repository description.

* Operating System
  * [Ubuntu 16.04](http://releases.ubuntu.com/16.04/) - Currently supported version for both the real robot and the simulation, Ubuntu 16.04 and ROS Kinetic-Kame.
  * [Ubuntu 18.04](http://releases.ubuntu.com/18.04/) - Supported only for simulation, Ubuntu 18.04 and ROS Melodic-Morenia.
* Middleware
  * [ROS](http://www.ros.org/) - Currently supported version Kinetic for Ubuntu 16.04
  * [YARP](http://wiki.icub.org/yarpdoc/) - we try to keep everything working with the most recent version of YARP
* Other Dependencies
  * [GAZEBO](http://gazebosim.org/) - It needs GAZEBO greather or equal than 9 to run all the simulation plugins and packages.
  * [gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins) - we try to keep everything working with the most recent version of gazebo-yarp-plugins

At any time you might need to install some more specific dependencies (like some missing ROS packages). Please open an issue in case you can't solve these or other dependencies.

## Download and Setup

You should have a catkin workspace on your file system to be able to compile the code. If you don't know how to do this please follow [these instructions](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).

As soon as you have your catkin_workspace configured you are ready to open a terminal and run the following instructions:

    git clone https://github.com/vislab-tecnico-lisboa/vizzy_install

### Ubuntu 16.04

Run the scripts in the vizzy-install repostory in the following order. You will be asked about the ROS version, which in this case is `kinetic`, and the location of your catkin workspace (e.g. `/home/user/my_catkin_ws`)

    cd  vizzy_install
    ./ros_install.sh
    source ~/.bashrc
    ./ros_packages_install.sh
    source ~/.bashrc
    ./install_yarp.sh
    source ~/.bashrc

### Ubuntu 18.04

Run the scripts in the vizzy-install repostory in the following order. You will be asked about the ROS version, which in this case is `melodic`, and the location of your catkin workspace (e.g. `/home/user/my_catkin_ws`)

    cd  vizzy_install
    ./ros_install.sh
    source ~/.bashrc
    ./ros_packages_install.sh
    source ~/.bashrc
    ./install_yarp_1804.sh
    source ~/.bashrc


## Run

For now let's focus on simulation. Open a terminal:

    roslaunch vizzy_launch vizzy_simulation.launch

This configuration starts Vizzy in the ROS-mode, meaning that all the controllers are emulated using ROS. This configuration works with Gazebo 7.x and ROS kinetic (The default install procedure of ROS kinetic) and also with Gazebo 9 and ROS kinetic

The upper body controllers simulated by the [gazebo-yarp-plugins](https://github.com/robotology/gazebo-yarp-plugins) can be started by

    yarpserver
    roslaunch vizzy_launch vizzy_simulation.launch use_yarp:=true

Don't forget you'll need to have `yarpserver` running when you have the `use_yarp` argument set as true.

Feel free to play with the arguments as you want or to change the low-level launchers with more functionality.

## 3D-2D image projection - Using the correct parameters
When using ROS camera interface, be careful to use the correct matrix for point (back-)projection
* In case of using ROS topic `.../image_rect_color` the correct projection matrix is P (with zero distorsion parameters)
* In case of using ROS topic `.../image_raw` the correct projection matrix is K (with distortion parameters)

## AUDIO (ON REAL VIZZY)

# First time configuration

To access pulseaudio and all the sound options through the network you need to make it discoverable. For that use:

    paprefs
    
Add the vizzy user to the audio group

    sudo usermod -aG audio,pulse,pulse-access `whoami`

In order to access the audio configurations via SSH you need to define the the following environment variable on Vizzy:

    export PULSE_SERVER=127.0.0.1

Furthermore, without a X11 session the PulseAudio server will not launch automatically since it normally requires X11. To run PulseAudio in a headless machine you need to run it in daemon mode:

    pulseaudio -D

This should be launched automatically, but if the audio is not working you should check if the pulseaudio server is running. If not, execute the previous command and it should work.

To control the audio volume use

    alsamixer

Now you can access audio configurations via ssh -X. Useful commands:

    gnome-control-center
    pavucontrol
    pacmd

## Documentation
For more details see the following reference:

    @inproceedings{moreno2016vizzy,
      title={Vizzy: A humanoid on wheels for assistive robotics},
      author={Moreno, Plinio and Nunes, Ricardo and Figueiredo, Rui and Ferreira, Ricardo and Bernardino, Alexandre and Santos-Victor, Jos{\'e} and Beira, Ricardo and Vargas, Lu{\'\i}s and Arag{\~a}o, Duarte and Arag{\~a}o, Miguel},
      booktitle={Robot 2015: Second Iberian Robotics Conference},
      pages={17--28},
      year={2016},
      organization={Springer}
     }

## Issues

### Compile error:

    CMakeFiles/charging_action_server.dir/src/charging_action_server_node.cpp.o: In function pcl::PPFHashMapSearch::setInputFeatureCloud(boost::shared_ptr<pcl::PointCloud<pcl::PPFSignature> const>): charging_action_server_node.cpp:(.text+0xd0): multiple definition of pcl::PPFHashMapSearch::setInputFeatureCloud(boost::shared_ptr<pcl::PointCloud<pcl::PPFSignature> const>) CMakeFiles/charging_action_server.dir/src/charging_action_server.cpp.o:charging_action_server.cpp:(.text+0x170): first defined here...

The solution is to edit your "ppf_registration.hpp" and inline the functions: setInputFeatureCloud and nearestNeighborSearch:

    sudo vim /usr/include/pcl-1.7/pcl/registration/impl/ppf_registration.hpp

```cpp
inline void
pcl::PPFHashMapSearch::setInputFeatureCloud (PointCloud<PPFSignature>::ConstPtr feature_cloud)
{
```

```cpp
inline void
pcl::PPFHashMapSearch::nearestNeighborSearch (float &f1, float &f2, float &f3, float &f4,
                                              std::vector<std::pair<size_t, size_t> > &indices)
{
```

All kind of issues and contributions will be very welcome. Please get in touch on [our issues page](https://github.com/vislab-tecnico-lisboa/vizzy/issues) when help is needed!
