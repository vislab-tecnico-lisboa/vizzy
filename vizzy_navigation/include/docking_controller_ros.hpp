#ifndef DOCKING_CONTROLLER_ROS_HPP_
#define DOCKING_CONTROLLER_ROS_HPP_

#include <docking_controller.hpp>
#include <vizzy_navigation/DockingConfig.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>


class DockingControllerROS
{
private:
    ros::NodeHandle nh_;
    dynamic_reconfigure::Server<vizzy_navigation::DockingConfig> server;
    dynamic_reconfigure::Server<vizzy_navigation::DockingConfig>::CallbackType f;
    DockingController controller;

public:
    void dynamic_rec_callback(vizzy_navigation::DockingConfig &config, uint32_t level);
    void run();
    DockingControllerROS(ros::NodeHandle nh);
    ~DockingControllerROS();
};



#endif