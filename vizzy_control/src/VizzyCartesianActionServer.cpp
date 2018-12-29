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
    : action_server_(nh, name, false), private_node_handle("~"), tfBuffer(), tfListener(tfBuffer)
{
    private_node_handle.getParam("robot_part", robot_part);
    stop_execution = node_handle_.advertise<std_msgs::Bool>("/" + robot_part + "_cartesian_pose_cancel", 1);
    goal_from_user = node_handle_.advertise<vizzy_msgs::CartesianGoal>("/" + robot_part + "_cartesian_pose_from_ros", 1);
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

void VizzyCartesianActionServer::feedbackCallback(const vizzy_msgs::CartesianFeedbackConstPtr &msg)
{
    //ROS_ERROR("FEEDBACK CALLBACK");
    //current_pose = *msg;
    feedback_.current_e_eff_pose = msg->current_e_eff_pose;

    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped onBase;
    //std::cout << "frame: " << msg->current_e_eff_pose.header.frame_id << std::endl;    
    if(msg->current_e_eff_pose.header.frame_id == "base_link"  || goal_msg->type==vizzy_msgs::CartesianGoal::HOME || goal_msg->type==vizzy_msgs::CartesianGoal::GRAB || goal_msg->type==vizzy_msgs::CartesianGoal::RELEASE)
    {
        onBase = msg->current_e_eff_pose;
    }else{
        try{
        transformStamped = tfBuffer.lookupTransform("base_link", msg->current_e_eff_pose.header.frame_id,
                                    ros::Time(0));
        tf2::doTransform(msg->current_e_eff_pose, onBase, transformStamped);

        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
        }

    }


    current_pose = onBase;
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
        action_server_.setAborted(result);
    }
    else if (msg->data == 1)
    {
        vizzy_msgs::CartesianResult result;
        ROS_INFO("Cartesian goal reached!!");
	if (action_server_.isPreemptRequested() || action_server_.isActive()){
        result.state_reached = true;
        result.end_effector_pose = current_pose;
        action_server_.setSucceeded(result);
	}
    }
    return;
}

void VizzyCartesianActionServer::goalCallback()
{
    action_active = true;
    goal_msg = action_server_.acceptNewGoal();
    action_server_.isPreemptRequested();
    vizzy_msgs::CartesianGoal new_goal_msg;
    

    geometry_msgs::TransformStamped transformStamped;
    geometry_msgs::PoseStamped onBase;

    if(goal_msg->end_effector_pose.header.frame_id == "base_link" || goal_msg->type==vizzy_msgs::CartesianGoal::HOME || goal_msg->type==vizzy_msgs::CartesianGoal::GRAB || goal_msg->type==vizzy_msgs::CartesianGoal::RELEASE)
    {
        onBase = goal_msg->end_effector_pose;
    }else{
        try{
        transformStamped = tfBuffer.lookupTransform("base_link", goal_msg->end_effector_pose.header.frame_id,
                                    ros::Time(0));
        tf2::doTransform(goal_msg->end_effector_pose, onBase, transformStamped);

        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            return;
        }

    }
    new_goal_msg = *goal_msg;
    new_goal_msg.end_effector_pose = onBase;

    goal_from_user.publish(new_goal_msg);
    /*if (goal_msg->type == vizzy_msgs::CartesianGoal::CARTESIAN)
        goal_from_user.publish(goal_msg);
    else if (goal_msg->type == vizzy_msgs::CartesianGoal::HOME){
        geometry_msgs::Pose my_nan_pose;
        my_nan_pose.position.x = std::nan("");
        my_nan_pose.position.y = std::nan("");
        my_nan_pose.position.z = std::nan("");
        pose_from_user.publish(my_nan_pose);
    }
    else if (goal_msg->type == vizzy_msgs::CartesianGoal::VELOCITY){
        geometry_msgs::Pose my_nan_pose;
        my_nan_pose = goal_msg->end_effector_pose;
        my_nan_pose.orientation.y = std::nan("");
        my_nan_pose.orientation.z = std::nan("");
        my_nan_pose.orientation.w = std::nan("");
        pose_from_user.publish(my_nan_pose);
    }*/
    return;
}

void VizzyCartesianActionServer::preemptCB()
{
    ROS_INFO("joint_velocity_controller: Preempted");
    // set the action state to preempted
    action_active = false;
    // In this case preempted is equivalent to go to home position
    vizzy_msgs::CartesianGoal goal_msg_preempt;
    goal_msg_preempt.type = vizzy_msgs::CartesianGoal::PREEMPT;
    /*geometry_msgs::Pose my_nan_pose;
    my_nan_pose.position.x = std::nan("");
    my_nan_pose.position.y = std::nan("");
    my_nan_pose.position.z = std::nan("");*/
    goal_from_user.publish(goal_msg_preempt);
    ROS_INFO("Before setting preempted");
    action_server_.setPreempted();
    return;
}
