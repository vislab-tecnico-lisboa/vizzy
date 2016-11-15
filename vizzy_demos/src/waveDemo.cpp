#include "ros/ros.h"
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>

#define MAX_JOINT_VEL 0.5  //in radians/sec

bool moveArm(){
    ROS_INFO("Move arm");
    bool success=true;
    double goalPositionTolerance=0.1;
    double goalOrientationTolerance=10.0*M_PI/180.0;


    double rate=10.0;

    ros::Rate r(rate);

    ros::AsyncSpinner spinner(2);
    spinner.start();


    // create the action client
    // true causes the client to spin its own thread
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/vizzy/right_arm_trajectory_controller/follow_joint_trajectory", true);

    ROS_INFO("Waiting for action server to start.");
    // wait for the action server to start
    ac.waitForServer(); //will wait for infinite time

    moveit::planning_interface::MoveGroup group("right_arm");
    group.setPlannerId("RRTConnectkConfigDefault");
    group.setPlanningTime(5);
    group.setStartState(*group.getCurrentState());

    group.setGoalPositionTolerance(goalPositionTolerance);
    group.setGoalOrientationTolerance(goalOrientationTolerance);
    group.startStateMonitor();

    success=group.setNamedTarget("right_arm_wave");
    group.move();

    //success=group.setJointValueTarget(test);
    std::vector<std::string> joint_names=group.getJoints();
    std::map<std::string, int> joint_names_map;
    for(int i=0; i<joint_names.size(); ++i)
    {
        joint_names_map[joint_names[i]]=i;
        std::cout << joint_names_map[joint_names[i]] << std::endl;
    }

    std::vector<double> current_joint_values=group.getCurrentJointValues();

    joint_names.pop_back();

    group.setStartState(*group.getCurrentState());
    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory.joint_names = joint_names;

    int i=0;
    double time_from_start = 0.01;

    ROS_INFO("FINISHED GOING WAVE HOME");

    while(i<5)
    {
        std::vector<double> set_joint_values=current_joint_values;

        // 20 degree waving amplitude
        double angle=10.0*M_PI/180.0;
        double max_joint_move=angle*2.0;
        if(i%2==0)
        {
            set_joint_values[joint_names_map['name']]+=angle;
        }
        else
        {
            set_joint_values[joint_names_map['name']]-=angle;
        }
        ++i;

        double seconds = max_joint_move/MAX_JOINT_VEL;
        ROS_INFO("max_joint_move: %0.3f, seconds: %0.3f", max_joint_move, seconds);
        time_from_start += seconds;
        trajectory_msgs::JointTrajectoryPoint point;
        point.time_from_start=ros::Duration(time_from_start);
        point.positions=set_joint_values;
        set_joint_values.pop_back();
        joint_trajectory.points.push_back(point);
    }

    // send a goal to the action
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory=joint_trajectory;
    ac.sendGoal(goal);
    ROS_INFO("DONE");

    //wait for the action to return
    bool finished_before_timeout = ac.waitForResult(ros::Duration(30.0));

    if (finished_before_timeout)
    {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s",state.toString().c_str());
    }
    else
    {
        ROS_INFO("Action did not finish before the time out.");
    }



    return success;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "wave_demo");
    ros::NodeHandle n;

    moveArm();
    return 0;
}



