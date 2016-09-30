#include "GazeReal.h"

GazeReal::GazeReal(const std::string & name, const ros::NodeHandle & nh) : Gaze(name,nh)
{
    private_node_handle.param<std::string>("left_eye_frame", left_eye_frame, "left_eye_frame");
    private_node_handle.param<std::string>("right_eye_frame", right_eye_frame, "right_eye_frame");
    private_node_handle.param<std::string>("neck_frame", neck_frame, "neck_frame");
    private_node_handle.param<std::string>("head_origin_frame", head_origin_frame, "head_origin_frame");
    private_node_handle.param<std::string>("eyes_center_frame", eyes_center_frame, "eyes_center_frame");
    private_node_handle.param<std::string>("world_frame", world_frame, "world_frame");
    private_node_handle.param<std::string>("fixation_point_frame", fixation_point_frame, "fixation_point_frame");

    //Publishers
    gazePublisher = nh_.advertise<geometry_msgs::Point>("fixation_point_out", 1);
    fix_point_sub = nh_.subscribe("fixation_point_in", 1, &GazeReal::analysisCB, this);

    as_.registerGoalCallback(boost::bind(&Gaze::goalCB, this));
    as_.registerPreemptCallback(boost::bind(&Gaze::preemptCB, this));

    as_.start();

    ROS_INFO("Going to move head and eyes to home position.");
    moveHome();
}


// TODO
bool GazeReal::moveHome()
{

    //We don't have control of the joints yet. So we need to set a decent initial fixation point by hand.
    //Let's set the point in the base_link frame x=-20, y=0, z=0.959
    //-20 because base_link has the x axis reversed

    //Modo martelanço activado... lol
    geometry_msgs::Point goalToRobot;
    goalToRobot.x = -20;
    goalToRobot.y = 0;
    goalToRobot.z = 0.959;
    //Modo martelanço desactivado
    home_position_fixation_point.point=goalToRobot;
    home_position_fixation_point.header.frame_id=world_frame;
    home_position_fixation_point.header.stamp=ros::Time::now();

    //while(nh_.ok()&&gazePublisher.getNumSubscribers()<1);

    gazePublisher.publish(goalToRobot);

    return true;
}

bool GazeReal::moveCartesian()
{
    //One day we will probably control each joint value here. But now we will just leave the fixation point for the ros-yarp bridge
    //to process

    geometry_msgs::PointStamped goal_point;

    //Ros-yarp bridge receives the point in the waist frame, right? Let's get it in that frame doing another copy/paste from Rui's code
    try
    {
        ros::Time current_time = ros::Time::now();
        tf_listener->waitForTransform(fixation_point_frame, current_time, goal_msg->fixation_point.header.frame_id, goal_msg->fixation_point.header.stamp, world_frame, ros::Duration(0.1) );
        tf_listener->transformPoint(fixation_point_frame, current_time, goal_msg->fixation_point, world_frame, goal_point);
    }
    catch (tf::TransformException &ex)
    {
        ROS_WARN("%s",ex.what());
        return false;
    }

    // TODO CHECK LIMITS USING MOVEIT!!!!

    //Convert it to point, since ros-yarp wont receive a pointStamped
    geometry_msgs::Point goalToRobot;
    goalToRobot.x = goal_point.point.x;
    goalToRobot.y = goal_point.point.y;
    goalToRobot.z = goal_point.point.z;

    gazePublisher.publish(goalToRobot);

    return true;
}

void GazeReal::analysisCB(const geometry_msgs::PointStamped::ConstPtr& fixation_point_msg)
{
    if(!active)
        return;

    //I need to know how the real robot gives us feedback regarding the current fixation point

    // Convert points to world frame
    geometry_msgs::PointStamped fixation_point_;
    geometry_msgs::PointStamped goal_point_;

    // Move home check joint state
    if(goal_msg->type==vizzy_msgs::GazeGoal::HOME)
    {

        try
        {
            ros::Time current_time = ros::Time::now();
            tf_listener->waitForTransform(world_frame, current_time, fixation_point_msg->header.frame_id, fixation_point_msg->header.stamp, world_frame, ros::Duration(0.1) );
            tf_listener->transformPoint(world_frame, current_time, *fixation_point_msg, world_frame, fixation_point_);
            tf_listener->waitForTransform(world_frame, current_time, home_position_fixation_point.header.frame_id, home_position_fixation_point.header.stamp, world_frame, ros::Duration(0.1) );
            tf_listener->transformPoint(world_frame, current_time, home_position_fixation_point, world_frame, goal_point_);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            return;
        }

        double error_x=fixation_point_.point.x-goal_point_.point.x;
        double error_y=fixation_point_.point.y-goal_point_.point.y;
        double error_z=fixation_point_.point.z-goal_point_.point.z;
        double error=sqrt(error_x*error_x+error_y*error_y+error_z*error_z);
        feedback_.state_reached=false;
        feedback_.fixation_point=fixation_point_;
        feedback_.fixation_point_error=error;

        if(error < goal_msg->fixation_point_error_tolerance)
        {
            feedback_.state_reached=true;

            as_.setSucceeded(result_);

            ros::WallTime total_time = ros::WallTime::now();
            ROS_INFO_STREAM(action_name_.c_str()<<": Succeeded. Total time: " <<  (total_time - start_time).toSec());
            active=false;
        }
    }
    else
    {
        try
        {
            ros::Time current_time = ros::Time::now();
            tf_listener->waitForTransform(world_frame, current_time, fixation_point_msg->header.frame_id, fixation_point_msg->header.stamp, world_frame, ros::Duration(0.1) );
            tf_listener->transformPoint(world_frame, current_time, *fixation_point_msg, world_frame, fixation_point_);
            tf_listener->waitForTransform(world_frame, current_time, goal_msg->fixation_point.header.frame_id, goal_msg->fixation_point.header.stamp, world_frame, ros::Duration(0.1) );
            tf_listener->transformPoint(world_frame, current_time, goal_msg->fixation_point, world_frame, goal_point_);
        }
        catch (tf::TransformException &ex)
        {
            ROS_WARN("%s",ex.what());
            return;
        }


        double error_x=fixation_point_.point.x-goal_point_.point.x;
        double error_y=fixation_point_.point.y-goal_point_.point.y;
        double error_z=fixation_point_.point.z-goal_point_.point.z;
        double error=sqrt(error_x*error_x+error_y*error_y+error_z*error_z);
        feedback_.state_reached=false;
        feedback_.fixation_point=fixation_point_;
        feedback_.fixation_point_error=error;

        if(error<goal_msg->fixation_point_error_tolerance)
        {
            result_.state_reached=true;
            feedback_.state_reached=true;

            as_.setSucceeded(result_);

            ros::WallTime total_time = ros::WallTime::now();
            ROS_INFO_STREAM(action_name_.c_str()<<": Succeeded. Total time: " <<  (total_time - start_time).toSec());
            active=false;
        }
    }

    as_.publishFeedback(feedback_);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "gaze");
    ros::NodeHandle nh_;
    ros::AsyncSpinner spinner(4);
    spinner.start();
    GazeReal gaze(ros::this_node::getName(),nh_);
    ros::waitForShutdown();
    spinner.stop();
    return 0;
}

