#ifndef BALLFOLLOWERROS_H
#define BALLFOLLOWERROS_H

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class BallFollowerRos
{
    std::string navigation_frame;
    ros::Subscriber ball_position_sub;
    ros::NodeHandle nh;
    boost::shared_ptr<tf::TransformListener> tf_listener;

    void ballPositionCallback(const geometry_msgs::PointStampedConstPtr & ball_position_msg);
public:
    boost::shared_ptr<MoveBaseClient> move_base_client;
    BallFollowerRos(const ros::NodeHandle & nh_,
                    const std::string & move_base_client_name_,
                    const std::string & navigation_frame_);
};

#endif // BALLFOLLOWERROS_H
