// By: Rui P. de Figueiredo : ruifigueiredo@isr.ist.utl.pt


#ifndef GAZE_H
#define GAZE_H
// MoveIt!
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/MoveGroupActionFeedback.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <std_msgs/Float64.h>
#include <geometry_msgs/PointStamped.h>


#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <vizzy_msgs/GazeAction.h>
#include <tf/transform_listener.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/message_filter.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <control_msgs/JointControllerState.h>

class Gaze
{

    std::string base_frame_id;
    std::map<std::string, int> joints_to_indices;

protected:

    double y_offset;
    double z_offset;
    double distance_ego_eyes;

    std::string left_eye_frame;
    std::string right_eye_frame;
    std::string head_origin_frame;
    std::string eyes_center_frame;
    std::string neck_frame;
    std::string world_frame;

    Eigen::Vector3d last_fixation_point;

    vizzy_msgs::GazeGoalConstPtr goal_msg;
    ros::WallTime start_time;


    planning_scene_monitor::CurrentStateMonitorPtr state_monitor;
    boost::shared_ptr<tf2_ros::TransformListener> tf_listener;
    tf2_ros::Buffer tfBuffer;
    ros::NodeHandle nh_;
    ros::NodeHandle private_node_handle;

    // NodeHandle instance must be created before this line. Otherwise strange error may occur.
    actionlib::SimpleActionServer<vizzy_msgs::GazeAction> as_;
    std::string action_name_;
    // create messages that are used to published feedback/result
    vizzy_msgs::GazeFeedback feedback_;
    vizzy_msgs::GazeResult result_;

    boost::shared_ptr<message_filters::Subscriber<geometry_msgs::PointStamped> > fixation_point_sub;

    // Publishers
    ros::Publisher neck_pan_pub;
    ros::Publisher neck_tilt_pub;
    ros::Publisher eyes_tilt_pub;
    ros::Publisher l_eye_pub;
    ros::Publisher r_eye_pub;
    ros::Publisher fixation_point_goal_viz_pub;

    void publishFixationPointGoal();

    virtual bool moveHome() = 0;
    virtual bool moveCartesian() = 0;

    std::vector<double> oculocephalic_joint_values;
    std::vector<std::string> oculocephalic_joint_names;
    moveit::planning_interface::MoveGroupInterface* oculocephalic_group;

public:
    double half_base_line;
    Gaze(const std::string & name, const ros::NodeHandle & nh);
    void preemptCB();
    /*virtual void analysisCB(const control_msgs::JointControllerState::ConstPtr & neck_pan_msg,
                    const control_msgs::JointControllerState::ConstPtr & neck_tilt_msg,
                    const control_msgs::JointControllerState::ConstPtr & eyes_tilt_msg,
                    const control_msgs::JointControllerState::ConstPtr & version_msg,
                    const control_msgs::JointControllerState::ConstPtr & vergence_msg,
                    const geometry_msgs::PointStamped::ConstPtr& fixation_point_msg);
                    */
    void goalCB();
};

#endif // GAZE_H
