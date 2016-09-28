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
    ros::init(argc, argv,"");//, "left_arm_trajectory_controller");
    ros::NodeHandle nh("vizzy");
    bool is_first_init = true;
    while (ros::ok()){
	VizzyLeftArmTrajectoryActionServer act_server(nh);
	ros::spin();
    }
}
