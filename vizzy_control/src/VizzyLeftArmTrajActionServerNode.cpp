/**
 *  File: VizzyLeftArmTrajActionServerNode.cpp
 *  Desc: Class for interfacing moveIt with Vizzy's left arm.
 *  Auth: Plinio Moreno
 *
 *
 */
#include "VizzyLeftArmTrajectoryActionServer.h"

int main(int argc, char **argv)
{
    ROS_INFO("Step -2");
    ros::init(argc, argv, "vizzy_left_arm_traj_act_server");
    ROS_INFO("Step -1");
    ros::NodeHandle nh("~");
    ROS_INFO("Step 0");
    bool is_first_init = true;
    while (ros::ok()){
	ROS_INFO("Step 0.1");
	VizzyLeftArmTrajectoryActionServer act_server(nh);
	ros::spin();
    }
}
