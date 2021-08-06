/*Copyright 2021, Joao Avelino, All rights reserved.*/

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

#include <mapless_nav/controller.hpp>
#include <mapless_nav/obstacle_avoidance.hpp>

#include <vizzy_navigation/MaplessConfig.h>
#include <dynamic_reconfigure/server.h>




class MaplessNavigator
{
private:
    
    ros::NodeHandle nh_;
    ros::NodeHandle nPriv_;
    ros::Publisher cmdPub_;
    std::string target_frame_;
    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener tfListener_;
    message_filters::Subscriber<geometry_msgs::PoseStamped> poseSub_;
    tf2_ros::MessageFilter<geometry_msgs::PoseStamped> tf2Filter_;

    dynamic_reconfigure::Server<vizzy_navigation::MaplessConfig> server_;
    dynamic_reconfigure::Server<vizzy_navigation::MaplessConfig>::CallbackType f_;

    std::string common_frame_;
    geometry_msgs::PoseStamped robot_pose_;
    geometry_msgs::PoseStamped current_goal_;
    ros::Time last_update_;

    double linvel_min_ = 0.1;
    double angvel_min_ = 0.1;

    //Controller
    mapless_controller::SegwayController controller_;
    RobotOperator obs_avoider_;

    void dynamic_rec_callback(vizzy_navigation::MaplessConfig &config, uint32_t level);

    double getDistanceError();
    double getOrientationError();

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void updateGoal(geometry_msgs::PoseStamped &goal);
    mapless_controller::Pose2D makeTransform(geometry_msgs::PoseStamped pose, std::string frame_id, ros::Time fromTime=ros::Time(0));


public:
    MaplessNavigator(ros::NodeHandle &nh);

    void doControlBase();

    void disableControl();
    void enableControl();

    ~MaplessNavigator();

};

#endif