/**
 *  File: VizzyCartesoamActionServerNode.cpp
 *  Desc: Class for interfacing cartesian YARP control.
 *  Auth: Plinio Moreno
 *        João Avelino
 *
 */
#include "VizzyCartesianActionServer.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv,"cartesian_action_server");
    ros::NodeHandle nh;
    bool is_first_init = true;
    while (ros::ok()){
    VizzyCartesianActionServer act_server(ros::this_node::getName(),nh);
	ros::spin();
    }
}
