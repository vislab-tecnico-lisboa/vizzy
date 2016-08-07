#include "FixationPointRos.h"

FixationPointRos::FixationPointRos(const ros::NodeHandle & nh_,const ros::NodeHandle & nh_priv_) :
    nh(nh_),
    nh_priv(nh_priv_),
    tf_listener(new tf::TransformListener(ros::Duration(5.0)))
{

    nh_priv.param<std::string>("left_eye_frame", left_eye_frame, "left_eye_frame");
    nh_priv.param<std::string>("right_eye_frame", right_eye_frame, "right_eye_frame");

    ROS_INFO_STREAM("left_eye_frame: "<<left_eye_frame);
    ROS_INFO_STREAM("right_eye_frame: "<<right_eye_frame);

    tf::StampedTransform transform;

    while(nh.ok())
    {
        try
        {
            tf_listener->waitForTransform(right_eye_frame, left_eye_frame, ros::Time(0), ros::Duration(10.0) );
            tf_listener->lookupTransform(right_eye_frame, left_eye_frame, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            continue;
        }
        break;
    }
    tf::Vector3 origin=transform.getOrigin();

    fixation_point=boost::shared_ptr<FixationPoint>(new FixationPoint((double)origin.length()));
    joint_state_sub = nh.subscribe("/vizzy/joint_states", 10, &FixationPointRos::jointStateCallback,this);
    fixation_point_pub = nh.advertise<geometry_msgs::PointStamped>("fixation_point", 10);
}

void FixationPointRos::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_states_msg)
{
    double left_eye_angle;
    double right_eye_angle;
    double eyes_tilt_angle;

    for(int i=0; i<joint_states_msg->name.size(); ++i)
    {
        if(joint_states_msg->name[i]==std::string("l_eye_joint"))
        {
            left_eye_angle=(double)joint_states_msg->position[i];
        }
        else if(joint_states_msg->name[i]==std::string("r_eye_joint"))
        {
            right_eye_angle=(double)joint_states_msg->position[i];
        }
        else if(joint_states_msg->name[i]==std::string("eyes_tilt_joint"))
        {
            eyes_tilt_angle=(double)joint_states_msg->position[i];
        }
    }

    tf::StampedTransform transform;

    while(nh.ok())
    {
        try
        {
            tf_listener->waitForTransform(right_eye_frame, left_eye_frame, ros::Time(0), ros::Duration(10.0) );
            tf_listener->lookupTransform(right_eye_frame, left_eye_frame, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            continue;
        }
        break;
    }
    tf::Vector3 origin=transform.getOrigin();
    base_line=(double)origin.length(); // meters



    Eigen::Vector3d fixation_point_eigen=fixation_point->getFixationPoint(left_eye_angle,
                                                                          right_eye_angle,
                                                                          eyes_tilt_angle,
                                                                          base_line);
    geometry_msgs::PointStamped fixation_point_msg;
    fixation_point_msg.header.frame_id="eyes_center_vision_link";
    fixation_point_msg.header.stamp=ros::Time::now();

    fixation_point_msg.point.x=fixation_point_eigen(0);
    fixation_point_msg.point.y=fixation_point_eigen(1);
    fixation_point_msg.point.z=fixation_point_eigen(2);

    fixation_point_pub.publish(fixation_point_msg);

    //std::cout << fixation_point_msg << std::endl;
}

int main(int argc, char **argv)
{
    /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line.
   * For programmatic remappings you can use a different version of init() which takes
   * remappings directly, but for most command-line programs, passing argc and argv is
   * the easiest way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
    ros::init(argc, argv, "ball_vis");

    ros::NodeHandle nh;
    ros::NodeHandle private_node_handle("~");
    FixationPointRos fixation_point_ros(nh,private_node_handle);

    int rate;
    private_node_handle.param("rate", rate, 100);
    ROS_INFO_STREAM("rate: "<<rate);

    // Tell ROS how fast to run this node.
    ros::Rate r(rate);

    // Main loop.
    while (nh.ok())
    {
        ros::spinOnce();
        r.sleep();
    }

    return 0;
}
