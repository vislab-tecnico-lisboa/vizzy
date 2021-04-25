#ifndef MAPLESS_NAVIGATOR_HPP_
#define MAPLESS_NAVIGATOR_HPP_

//ROS includes
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>

#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <mapless_nav/controller.hpp>

#include <tf2_ros/transform_listener.h>

#include <tf2_ros/message_filter.h>
#include <message_filters/subscriber.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#endif


class MaplessNavigator
{
private:
    
    enum State
    {
        STOPPED,
        LOCALCONTROLLER
    };

    State state_;

    ros::Publisher cmdPub_;
    std::string target_frame_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    ros::NodeHandle nh_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> poseSub_;
    tf2_ros::MessageFilter<geometry_msgs::PoseStamped> tf2Filter_;

    geometry_msgs::PoseStamped robot_pose_;

    //Current goal in the robot's frame
    geometry_msgs::PoseStamped current_goal_;
    double dist_tolerance_ = 0.1;
    double orient_tolerance_ = 0.1;

    void callback(const geometry_msgs::PoseStamped::ConstPtr& msg);


public:
    MaplessNavigator(ros::NodeHandle &nh);

    void doControlBase();

    ~MaplessNavigator();

};