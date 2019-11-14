#ifndef GAZESIM_H
#define GAZESIM_H

#include "Gaze.h"
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
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/MoveGroupAction.h>
#include <moveit_msgs/MoveGroupActionFeedback.h>

class GazeSimulation : public Gaze
{
public:
    typedef message_filters::sync_policies::ApproximateTime<control_msgs::JointControllerState, control_msgs::JointControllerState,control_msgs::JointControllerState, geometry_msgs::PointStamped> MySyncPolicy;
    boost::shared_ptr<message_filters::Subscriber<control_msgs::JointControllerState> > neck_pan_sub;
    boost::shared_ptr<message_filters::Subscriber<control_msgs::JointControllerState> > neck_tilt_sub;
    boost::shared_ptr<message_filters::Subscriber<control_msgs::JointControllerState> > eyes_tilt_sub;

    boost::shared_ptr<message_filters::Synchronizer<MySyncPolicy> >sync;

    GazeSimulation(const std::string & name, const ros::NodeHandle & nh);
    bool moveHome();
    bool moveCartesian();
    void analysisCB(const control_msgs::JointControllerState::ConstPtr & neck_pan_msg,
                    const control_msgs::JointControllerState::ConstPtr & neck_tilt_msg,
                    const control_msgs::JointControllerState::ConstPtr & eyes_tilt_msg,
                    const geometry_msgs::PointStamped::ConstPtr& fixation_point_msg);

protected:
    moveit::planning_interface::MoveGroupInterface* oculocephalic_group;
    planning_scene_monitor::CurrentStateMonitorPtr state_monitor;

};

#endif // GAZE_H
