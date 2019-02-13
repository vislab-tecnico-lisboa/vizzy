/*Copyright 2019, Joao Avelino, All rights reserved.*/

#include <charging_action_server.hpp>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "charging_action_server");

  ros::NodeHandle nh;
  ChargingActionServer actionServer(nh, "charging_action");

  ros::spin();


  return 0;
}