/**
 *  File: VizzyCartesianActionServer.h
 *  Desc: Class for interfacing with YARP cartesian controller.
 *  Auth: Plinio Moreno
 *        João Avelino
 *
 *
 */


#ifndef VIZZY_CARTESIAN_ACTION_H
#define VIZZY_CARTESIAN_ACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <vizzy_msgs/CartesianAction.h>
#include <vizzy_msgs/CartesianFeedback.h>
#include <vizzy_msgs/CartesianGoal.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//void actionBridgeCallback(const std_msgs::Int16::ConstPtr& msg);
class VizzyCartesianActionServer
{
 public:
    VizzyCartesianActionServer(const std::string & name, const ros::NodeHandle &n);
    ~VizzyCartesianActionServer();

    void goalCallback();
    actionlib::SimpleActionServer<vizzy_msgs::CartesianAction> action_server_;
    void actionBridgeCallback(const std_msgs::Int16::ConstPtr& msg);
    void feedbackCallback(const vizzy_msgs::CartesianFeedbackConstPtr &msg);
    void preemptCB();
 private:
    bool action_active;
    ros::NodeHandle node_handle_;
    ros::Time last_nonstall_time_;
    ros::Subscriber info_from_bridge;
    ros::Publisher trajectory_from_move_it;
    ros::Publisher stop_execution;
    ros::Publisher goal_from_user;
    ros::Subscriber feedback_from_bridge;
    vizzy_msgs::CartesianGoalConstPtr goal_msg;
    ros::NodeHandle private_node_handle;
    std::string robot_part;
    geometry_msgs::PoseStamped current_pose;
    vizzy_msgs::CartesianFeedback feedback_;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
};

#endif  // VIZZY_CARTESIAN_ACTION_H

