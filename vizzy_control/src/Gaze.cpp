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
    oculocephalic_group(new moveit::planning_interface::MoveGroupInterface("oculocephalic")),
    active(false),
    it(nh),
    first_suppresion(true)
{
    tf_listener=boost::shared_ptr<tf2_ros::TransformListener> (new tf2_ros::TransformListener(tfBuffer));
    oculocephalic_joint_names=oculocephalic_group->getActiveJoints();
    oculocephalic_joint_values.resize(oculocephalic_joint_names.size());
    std::fill(oculocephalic_joint_values.begin(), oculocephalic_joint_values.end(), 0);

    private_node_handle.param<std::string>("base_frame", base_frame_id, "base_frame");
    private_node_handle.param("vel_threshold", vel_threshold, 0.1);

    int queue_size_ = 5; //queue size
    fixation_point_goal_viz_pub = nh_.advertise<geometry_msgs::PointStamped>("fixation_point_goal_viz", queue_size_);
    left_image_suppression_pub=nh_.advertise<sensor_msgs::Image>("left_camera_out", queue_size_);
    right_image_suppression_pub=nh_.advertise<sensor_msgs::Image>("right_camera_out", queue_size_);

    // Saccadic
    left_image_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > (new message_filters::Subscriber<sensor_msgs::Image>(nh_, "left_camera_in", queue_size_));
    right_image_sub=boost::shared_ptr<message_filters::Subscriber<sensor_msgs::Image> > (new message_filters::Subscriber<sensor_msgs::Image>(nh_, "right_camera_in", queue_size_));

    //TF's synchronized with the image
    left_image_filter=boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::Image> > (new tf2_ros::MessageFilter<sensor_msgs::Image>(*left_image_sub, tfBuffer, base_frame_id, queue_size_,nh_));
    right_image_filter=boost::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::Image> > (new tf2_ros::MessageFilter<sensor_msgs::Image>(*right_image_sub, tfBuffer, base_frame_id, queue_size_,nh_));

    gaze_sync=boost::shared_ptr<message_filters::Synchronizer<MySuppressionSyncPolicy> > (new message_filters::Synchronizer<MySuppressionSyncPolicy>(MySuppressionSyncPolicy(queue_size_),
                                                                                                                               *left_image_filter,
                                                                                                                               *right_image_filter
                                                                                                                               ));

    gaze_sync->registerCallback(boost::bind(&Gaze::suppresion, this, _1, _2));
}

void Gaze::suppresion(const sensor_msgs::Image::ConstPtr & left_image_msg,
                      const sensor_msgs::Image::ConstPtr & right_image_msg)
{


    static int count=0;
    if(active)
    {
        ROS_ERROR("ESTOU A SUPPRIMIR");
        //write left image
        //cv::imwrite("/home/vizzy/images_moutinho"+);
        // write right image
        return;
    }
    ++count;
    std::ostringstream ss_left, ss_right;
    ss_left << count << "_left.jpg";
    ss_right << count << "_right.jpg";

    cv::Mat left_image_mat =cv_bridge::toCvCopy(left_image_msg, "bgr8")->image;
    cv::Mat right_image_mat =cv_bridge::toCvCopy(right_image_msg, "bgr8")->image;

    //ROS_ERROR_STREAM(ss_left.str()<< " "<< ss_right.str());
    //cv::imwrite("/home/vizzy/images_moutinho/"+ss_left.str(),left_image_mat);
    //cv::imwrite("/home/vizzy/images_moutinho/"+ss_right.str(),right_image_mat);

    left_image_suppression_pub.publish(left_image_msg);
    right_image_suppression_pub.publish(right_image_msg);
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
    active=false;
    as_.setPreempted();
}

void Gaze::goalCB()
{

    goal_msg = as_.acceptNewGoal();

    as_.isPreemptRequested();


    start_time = ros::WallTime::now();
    active=true;

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


