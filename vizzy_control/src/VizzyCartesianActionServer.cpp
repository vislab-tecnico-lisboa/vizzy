/**
 *  File: VizzyCartesianActionServer.cpp
 *  Desc: Class for interfacing YARP cartesian controller with ROS.
 *  Auth: Plinio Moreno
 *        João Avelino
 *
 *
 */

#include "VizzyCartesianActionServer.h"

using namespace std;

VizzyCartesianActionServer::VizzyCartesianActionServer(const std::string &name, const ros::NodeHandle &nh)
    : action_server_(nh, name, false), private_node_handle("~")
{
    private_node_handle.getParam("robot_part", robot_part);
    stop_execution = node_handle_.advertise<std_msgs::Bool>("/" + robot_part + "cartesian_pose_cancel", 1);
    pose_from_user = node_handle_.advertise<geometry_msgs::Pose>("/" + robot_part + "_cartesian_pose_from_ros", 1);
    action_server_.registerGoalCallback(boost::bind(&VizzyCartesianActionServer::goalCallback, this));
    action_server_.registerPreemptCallback(boost::bind(&VizzyCartesianActionServer::preemptCB, this));
    info_from_bridge = node_handle_.subscribe("/" + robot_part + "_cartesian_pose_result", 1, &VizzyCartesianActionServer::actionBridgeCallback, this);
    feedback_from_bridge = node_handle_.subscribe("/" + robot_part + "_cartesian_pose_feedback", 1, &VizzyCartesianActionServer::feedbackCallback, this);
    action_server_.start();
    action_active = false;
}

VizzyCartesianActionServer::~VizzyCartesianActionServer()
{
}

void VizzyCartesianActionServer::feedbackCallback(const geometry_msgs::Pose::ConstPtr &msg)
{
    //current_pose = *msg;
    feedback_.current_e_eff_pose = *msg;
    current_pose = *msg;
    action_server_.publishFeedback(feedback_);
}

void VizzyCartesianActionServer::actionBridgeCallback(const std_msgs::Int16::ConstPtr &msg)
{
    if (!action_active)
        return;
    if (msg->data == 0)
    {
        vizzy_msgs::CartesianResult result;
        //control_msgs::FollowJointTrajectoryResult result;
        ROS_INFO("Could not reach the cartesian end effector goal.");
        result.state_reached = false;
        result.end_effector_pose = current_pose;
        action_server_.setSucceeded(result);
    }
    else if (msg->data == 1)
    {
        vizzy_msgs::CartesianResult result;
        ROS_INFO("Cartesian goal reached!!");
        result.state_reached = true;
        result.end_effector_pose = current_pose;
        action_server_.setSucceeded(result);
    }
    return;
}

void VizzyCartesianActionServer::goalCallback()
{
    action_active = true;
    goal_msg = action_server_.acceptNewGoal();
    if (goal_msg->type == vizzy_msgs::CartesianGoal::CARTESIAN)
        pose_from_user.publish(goal_msg->end_effector_pose);
    else if (goal_msg->type == vizzy_msgs::CartesianGoal::HOME){
        geometry_msgs::Pose my_nan_pose;
        my_nan_pose.position.x = std::nan("");
        my_nan_pose.position.y = std::nan("");
        my_nan_pose.position.z = std::nan("");
        pose_from_user.publish(my_nan_pose);
    }
    return;
}

void VizzyCartesianActionServer::preemptCB()
{
    ROS_INFO("joint_velocity_controller: Preempted");
    // set the action state to preempted
    action_active = false;
    geometry_msgs::Pose my_nan_pose;
    my_nan_pose.position.x = std::nan("");
    my_nan_pose.position.y = std::nan("");
    my_nan_pose.position.z = std::nan("");
    pose_from_user.publish(my_nan_pose);
    action_server_.setPreempted();
    return;
}
