/**
 *  File: VizzyFollowTrajectoryActionServer.cpp
 *  Desc: Class for interfacing moveIt with Vizzy's left arm.
 *  Auth: Plinio Moreno
 *
 *
 */



#include "VizzyTrajectoryActionServer.h"

using namespace std;

VizzyFollowTrajectoryActionServer::VizzyFollowTrajectoryActionServer(const std::string & name, const ros::NodeHandle &nh)
    : action_server_(nh,name, false),private_node_handle("~")
{
    private_node_handle.getParam("robot_part", robot_part);
    stop_execution = node_handle_.advertise<std_msgs::Bool>("/" + robot_part + "_trajectory_cancel", 1);
    trajectory_from_move_it = node_handle_.advertise<trajectory_msgs::JointTrajectory>("/" + robot_part + "_trajectory_from_moveit", 1);
    action_server_.registerGoalCallback(boost::bind(&VizzyFollowTrajectoryActionServer::goalCallback, this));
    action_server_.registerPreemptCallback(boost::bind(&VizzyFollowTrajectoryActionServer::preemptCB, this));
    info_from_bridge = node_handle_.subscribe("/" + robot_part + "_trajectory_feedback", 1, &VizzyFollowTrajectoryActionServer::actionBridgeCallback,this);
    action_server_.start();
    action_active=false;
}


VizzyFollowTrajectoryActionServer::~VizzyFollowTrajectoryActionServer()
{
}

void VizzyFollowTrajectoryActionServer::actionBridgeCallback(const std_msgs::Int16::ConstPtr& msg)
{
    if (!action_active)
        return;
    if (msg->data == 0){
      control_msgs::FollowJointTrajectoryResult result;
      ROS_INFO("Could not reach the goal.");
      result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      action_server_.setSucceeded(result);
    }
    else if (msg->data==1){
      control_msgs::FollowJointTrajectoryResult result;
      ROS_INFO("Goal reached!!");
      result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      action_server_.setSucceeded(result);
    }
    return;
}

void VizzyFollowTrajectoryActionServer::goalCallback()
{
    action_active=true;
    goal_msg = action_server_.acceptNewGoal();
    trajectory_from_move_it.publish(goal_msg->trajectory);
    return;
}

void VizzyFollowTrajectoryActionServer::preemptCB()
{
   ROS_INFO("joint_velocity_controller: Preempted");
   // set the action state to preempted
   action_active=false;
   action_server_.setPreempted();
   return;
}
