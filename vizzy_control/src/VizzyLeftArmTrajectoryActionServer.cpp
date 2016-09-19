/**
 *  File: VizzyLeftArmTrajectoryActionServer.cpp
 *  Desc: Class for interfacing moveIt with Vizzy's left arm.
 *  Auth: Plinio Moreno
 *
 *
 */



#include "VizzyLeftArmTrajectoryActionServer.h"

using namespace std;

VizzyLeftArmTrajectoryActionServer::VizzyLeftArmTrajectoryActionServer(const ros::NodeHandle &nh)
    : action_server_(nh, "joint_velocity_controller",
                     boost::bind(&VizzyLeftArmTrajectoryActionServer::actionCallback, this, _1), false)
{
    ROS_INFO("Step 0.2");
    info_from_bridge = node_handle_.subscribe("left_arm_trajectory_feedback", 1000, &VizzyLeftArmTrajectoryActionServer::actionBridgeCallback,this);
    stop_execution = node_handle_.advertise<std_msgs::Bool>("left_arm_trajectory_cancel", 1000);
    trajectory_from_move_it = node_handle_.advertise<trajectory_msgs::JointTrajectory>("left_arm_trajectory_from_moveit", 1000);
    action_server_.start();
}


VizzyLeftArmTrajectoryActionServer::~VizzyLeftArmTrajectoryActionServer()
{
}

void VizzyLeftArmTrajectoryActionServer::actionBridgeCallback(const std_msgs::Int16::ConstPtr& msg)
{
    if (msg->data == 0){
      control_msgs::FollowJointTrajectoryResult result;
      ROS_INFO("Could not complete joint angle action in the expected time.");
      result.error_code = control_msgs::FollowJointTrajectoryResult::PATH_TOLERANCE_VIOLATED;
      action_server_.setSucceeded(result);
    }
    else if (msg->data==1){
      control_msgs::FollowJointTrajectoryResult result;
      result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
      action_server_.setSucceeded(result);
    }
}

void VizzyLeftArmTrajectoryActionServer::actionCallback(const control_msgs::FollowJointTrajectoryGoalConstPtr &goal)
{
    ROS_INFO("Entering in callback");
    trajectory_from_move_it.publish(goal->trajectory);
    ROS_INFO("Trajectory sent");
    if (action_server_.isPreemptRequested())
    {
      std_msgs::Bool my_bool;
      my_bool.data = true;
      stop_execution.publish(my_bool);
      //preempt action server
      action_server_.setPreempted();
      ROS_INFO("Joint trajectory server preempted by client");
    }
    control_msgs::FollowJointTrajectoryResult result;
    result.error_code = control_msgs::FollowJointTrajectoryResult::SUCCESSFUL;
    action_server_.setSucceeded(result);

}


