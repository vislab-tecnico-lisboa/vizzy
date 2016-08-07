#ifndef FIXATIONPOINTROS_H
#define FIXATIONPOINTROS_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PointStamped.h>
#include <boost/shared_ptr.hpp>
#include <tf/transform_listener.h>
#include "FixationPoint.h"
class FixationPointRos
{
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;
    boost::shared_ptr<tf::TransformListener> tf_listener;

    ros::Subscriber joint_state_sub;
    ros::Publisher fixation_point_pub;

    boost::shared_ptr<FixationPoint> fixation_point;

    std::string left_eye_frame;
    std::string right_eye_frame;

    double base_line;
public:
    FixationPointRos(const ros::NodeHandle & nh_, const ros::NodeHandle & nh_priv_);
    void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_states_msg);

};

#endif // FIXATIONPOINTROS_H
