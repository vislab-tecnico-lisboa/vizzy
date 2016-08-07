#include "BallFollowerRos.h"

BallFollowerRos::BallFollowerRos(const ros::NodeHandle & nh_,
                                 const std::string & move_base_server_name_,
                                 const std::string & navigation_frame_)
    : nh(nh_),
      move_base_client(new MoveBaseClient(move_base_server_name_, true)),
      navigation_frame(navigation_frame_),
      tf_listener(new tf::TransformListener(ros::Duration(10.0)))

{
    ball_position_sub=nh.subscribe("ball_point", 1, &BallFollowerRos::ballPositionCallback, this);
}

void BallFollowerRos::ballPositionCallback(const geometry_msgs::PointStampedConstPtr & ball_position_msg)
{
    ROS_INFO_STREAM("RECEIVED BALL POSITION: "<<
                    ball_position_msg->point.x << " "<<
                    ball_position_msg->point.y << " "<<
                    ball_position_msg->point.z);

    geometry_msgs::PointStamped goal_point;
    while(nh.ok())
    {
        try
        {
            tf_listener->waitForTransform(navigation_frame, ball_position_msg->header.frame_id, ros::Time(0), ros::Duration(10.0) );
            tf_listener->transformPoint(navigation_frame, *ball_position_msg, goal_point);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            continue;
        }
        break;
    }

    ROS_INFO_STREAM("RECEIVED BALL POSITION (after tf): " <<
                    goal_point.point.x << " "<<
                    goal_point.point.y << " "<<
                    goal_point.point.z);

    move_base_msgs::MoveBaseGoal goal_msg;

    goal_msg.target_pose.header.frame_id = navigation_frame;
    goal_msg.target_pose.header.stamp = ros::Time::now();

    // goal_msg.target_pose.pose.orientation.x = q.getX();
    //goal_msg.target_pose.pose.orientation.y = q.getY();
    //goal_msg.target_pose.pose.orientation.z = q.getZ();
    //goal_msg.target_pose.pose.orientation.w = q.getW();

    goal_msg.target_pose.pose.position.x = goal_point.point.x;
    goal_msg.target_pose.pose.position.y = goal_point.point.y;
    goal_msg.target_pose.pose.position.z = goal_point.point.z;

    //move_base_client->sendGoal(goal_msg);


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "vision_node");

    ros::NodeHandle nh;

    // Initialize node parameters from launch file or command line.
    // Use a private node handle so that multiple instances of the node can be run simultaneously
    // while using different parameters.

    ros::NodeHandle private_node_handle_("~");
    int rate;
    std::string navigation_frame;
    std::string move_base_server_name;

    private_node_handle_.param("rate", rate, 100);
    private_node_handle_.param<std::string>("navigation_frame", navigation_frame, "base_footprint");
    private_node_handle_.param<std::string>("move_base_server_name", move_base_server_name, "move_base");

    ROS_INFO_STREAM("rate: "<<rate);
    ROS_INFO_STREAM("navigation_frame: "<<navigation_frame);

    // Tell ROS how fast to run this node.
    ros::Rate r(rate);
    BallFollowerRos ball_follower(nh,
                                  move_base_server_name,
                                  navigation_frame);
    // Main loop.
    while (nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
} // end



