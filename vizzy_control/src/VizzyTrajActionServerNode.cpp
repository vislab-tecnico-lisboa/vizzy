/**
 *  File: VizzyLeftArmTrajActionServerNode.cpp
 *  Desc: Class for interfacing moveIt with Vizzy's left arm.
 *  Auth: Plinio Moreno
 *
 *
 */
#include "VizzyTrajectoryActionServer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv,"follow_joint_trajectory");//, "left_arm_trajectory_controller");
    ros::NodeHandle nh;
    bool is_first_init = true;
    while (ros::ok()){
    VizzyFollowTrajectoryActionServer act_server(ros::this_node::getName(),nh);
	ros::spin();
    }
}
