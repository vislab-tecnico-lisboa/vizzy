/**
 *  File: vizzy_right_arm_trajectory_action.h
 *  Desc: Class for interfacing with moveIt and the jaco arm.
 *  Auth: Plinio Moreno
 *
 *
 */


#ifndef VIZZY_RIGHT_ARM_TRAJECTORY_ACTION_H
#define VIZZY_RIGHT_ARM_TRAJECTORY_ACTION_H

#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#define NUM_JACO_JOINTS 6
//void actionBridgeCallback(const std_msgs::Int16::ConstPtr& msg);
class VizzyRightArmTrajectoryActionServer
{
 public:
    VizzyRightArmTrajectoryActionServer(const std::string & name, const ros::NodeHandle &n);
    ~VizzyRightArmTrajectoryActionServer();

    void goalCallback();
    actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> action_server_;
    void actionBridgeCallback(const std_msgs::Int16::ConstPtr& msg);
    void preemptCB();
 private:
    bool action_active;
    ros::NodeHandle node_handle_;
    ros::Time last_nonstall_time_;
    ros::Subscriber info_from_bridge;
    ros::Publisher trajectory_from_move_it;
    ros::Publisher stop_execution;
    control_msgs::FollowJointTrajectoryGoalConstPtr goal_msg;
};

#endif  // VIZZY_RIGHT_ARM_TRAJECTORY_ACTION_H

