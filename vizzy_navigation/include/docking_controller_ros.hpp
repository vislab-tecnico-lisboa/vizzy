/*Copyright 2019, Joao Avelino, All rights reserved.*/

#ifndef DOCKING_CONTROLLER_ROS_HPP_
#define DOCKING_CONTROLLER_ROS_HPP_

#include <docking_controller.hpp>
#include <vizzy_navigation/DockingConfig.h>
#include <dynamic_reconfigure/server.h>
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <exception>

class DockingControllerROS
{
private:
    ros::NodeHandle nh_;
    dynamic_reconfigure::Server<vizzy_navigation::DockingConfig> server_;
    dynamic_reconfigure::Server<vizzy_navigation::DockingConfig>::CallbackType f_;
    docking_ctrl::DockingController controller_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    std::string common_frame_;
    ros::Subscriber sub_;
    ros::Publisher pub_;
    ros::Time last_update_;
    
public:
    void dynamic_rec_callback(vizzy_navigation::DockingConfig &config, uint32_t level);
    void run();
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    docking_ctrl::Pose2D makeTransform(geometry_msgs::PoseStamped pose, std::string frame_id, ros::Time fromTime=ros::Time(0));
    DockingControllerROS(ros::NodeHandle nh);
    ~DockingControllerROS();
};



#endif