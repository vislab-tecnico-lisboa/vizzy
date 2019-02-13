/*Copyright 2019, Joao Avelino, All rights reserved.*/

#include <charging_action_server.hpp>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "charging_action_server");

  ros::NodeHandle nh;
  ChargingActionServer actionServer(nh, "charging_action");

<<<<<<< HEAD
  ros::spin();
=======
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
>>>>>>> 710f1e179debdddcc9d197ac88bbacd2fadc7068


  return 0;
}