// By: Rui P. de Figueiredo : ruifigueiredo@isr.ist.utl.pt

#include "Gaze.h"
#include <cv_bridge/cv_bridge.h>
Gaze::Gaze(const std::string & name, const ros::NodeHandle & nh) :
    nh_(nh),
    as_(nh_, name, false),
    private_node_handle("~"),
    action_name_(name),
    tfBuffer(ros::Duration(3.0)),
    last_fixation_point(Eigen::Vector3d::Constant(std::numeric_limits<double>::max())),
    oculocephalic_group(new moveit::planning_interface::MoveGroupInterface("oculocephalic"))
{
    tf_listener=boost::shared_ptr<tf2_ros::TransformListener> (new tf2_ros::TransformListener(tfBuffer));
    oculocephalic_joint_names=oculocephalic_group->getActiveJoints();
    oculocephalic_joint_values.resize(oculocephalic_joint_names.size());
    std::fill(oculocephalic_joint_values.begin(), oculocephalic_joint_values.end(), 0);

    private_node_handle.param<std::string>("base_frame", base_frame_id, "base_frame");

    int queue_size_ = 5; //queue size
    fixation_point_goal_viz_pub = nh_.advertise<geometry_msgs::PointStamped>("fixation_point_goal_viz", queue_size_);

}

void Gaze::publishFixationPointGoal()
{
    // Convert to neck frame for convenience
    geometry_msgs::PointStamped goal_point_world_viz;
    //while(nh_.ok())
    {
        try
        {
            ros::Time current_time = ros::Time::now();
            tfBuffer.lookupTransform(world_frame, current_time, goal_msg->fixation_point.header.frame_id, goal_msg->fixation_point.header.stamp, world_frame, ros::Duration(0.05) );
            tfBuffer.transform(goal_msg->fixation_point, goal_point_world_viz, world_frame);
        }
        catch (tf2::TransformException &ex)
        {
            //ROS_WARN("%s",ex.what());
            return;
        }
        
    }

    fixation_point_goal_viz_pub.publish(goal_point_world_viz);
}


void Gaze::preemptCB()
{
    //ROS_INFO("%s: Preempted", action_name_.c_str());
    // set the action state to preempted
    as_.setPreempted();
}

void Gaze::goalCB()
{

    goal_msg = as_.acceptNewGoal();

    as_.isPreemptRequested();



    if(goal_msg->type==vizzy_msgs::GazeGoal::HOME)
    {
        moveHome();

    }
    else
    {

        publishFixationPointGoal();

        if(!moveCartesian())
        {

            result_.state_reached=false;

            //ROS_INFO("%s: Aborted", action_name_.c_str());
            as_.setAborted(result_);

        }
    }
    // set the action state to succeeded

    return;
}


